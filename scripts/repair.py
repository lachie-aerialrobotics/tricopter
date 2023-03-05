#! /usr/bin/env python3
import open3d as o3d
import rospy
import copy
import os
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import ros_numpy as rnp
import toppra as ta
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header, String
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, PoseWithCovarianceStamped, TransformStamped, Transform, TwistStamped, Twist, Vector3, Quaternion, WrenchStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from mavros_msgs.msg import State
from tricopter.srv import *
    
class DamageDetection:
    def __init__(self):
        # get config values from parameter server
        map_topic_name = rospy.get_param('/damage_detection/map_topic_name')
        mesh_path = rospy.get_param('/damage_detection/model_dir')
        model_samples = rospy.get_param('/damage_detection/model_samples')
        self.voxel_size = rospy.get_param('/damage_detection/icp_voxel_size')
        self.max_icp_iterations = rospy.get_param('/damage_detection/icp_max_iterations')
        self.eps = rospy.get_param('/damage_detection/length_scale')
        self.min_points = rospy.get_param('/damage_detection/cluster_min_points')

        self.tip_frame = rospy.get_param('/trajectory_planner/tooltip_frame')
        self.drone_frame = rospy.get_param('/trajectory_planner/drone_frame')
        self.map_frame = 'camera_init'
        self.trajectory_frequency = rospy.get_param('/trajectory_planner/frequency')
        self.max_vel = rospy.get_param('/trajectory_planner/max_vel')
        self.max_acc = rospy.get_param('/trajectory_planner/max_acc')
        self.max_yawrate = rospy.get_param('/trajectory_planner/max_yawrate')
        self.max_yawrate_dot = rospy.get_param('/trajectory_planner/max_yawrate_dot')

        self.hover_time = 10.0

        # init publishers and subscribers
        self.pub_result = rospy.Publisher('/model_registration_result', PointCloud2, queue_size=1)
        self.pub_init = rospy.Publisher('/model_registration_init', PointCloud2, queue_size=1)
        self.pub_cropped_map = rospy.Publisher('/cropped_map', PointCloud2, queue_size=1)
        self.pub_damage = rospy.Publisher('/damage_points', PointCloud2, queue_size=1)
        self.pub_markers = rospy.Publisher('/damage_markers', MarkerArray, queue_size=1)
        self.pub_poses = rospy.Publisher('/damage_poses', PoseArray, queue_size=1)
        self.pub_tip_trajectory = rospy.Publisher('/repair_trajectory/tooltip', MultiDOFJointTrajectory, queue_size=1)
        self.pub_tip_trajectory_viz = rospy.Publisher('/repair_trajectory/tooltip/viz', Path, queue_size=1)
        self.pub_drone_trajectory = rospy.Publisher('/repair_trajectory/drone', MultiDOFJointTrajectory, queue_size=1)
        self.pub_drone_trajectory_viz = rospy.Publisher('/repair_trajectory/drone/viz', Path, queue_size=1)
        self.pub_drone_sp = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.pub_tip_sp = rospy.Publisher('/tooltip_setpoint/pose', PoseStamped, queue_size=1)
        self.pub_tooltip_state = rospy.Publisher('/manipulator/state',  String, queue_size=1, tcp_nodelay=True)

        sub_map = rospy.Subscriber(map_topic_name, PointCloud2, self.map_cb, queue_size=1)
        sub_drone_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavros_pose_cb, queue_size=1)
        sub_init_pose = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose_cb, queue_size=1)
        sub_state = rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=5, tcp_nodelay=True)
        
        
        rospy.wait_for_message('/mavros/state', State)
        self.pub_sp_position = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1, tcp_nodelay=True)

        # change working directory to this script - for saving .pcd files
        abspath = os.path.abspath(__file__)
        dname = os.path.dirname(abspath)
        os.chdir(dname)
        
        # generate pointcloud from undamaged cad model
        self.model_pc = get_pc_from_mesh(mesh_path, model_samples)

        # perception filter - todo: get rid of the need for this
        diameter = np.linalg.norm(np.asarray(self.model_pc.get_max_bound()) - np.asarray(self.model_pc.get_min_bound()))
        radius = diameter * 100
        camera = [5,0,1.5]
        _, pt_map = self.model_pc.hidden_point_removal(camera, radius)
        self.model_pc = self.model_pc.select_by_index(pt_map)

        # check that damage detection is done before trajectory generation can start
        self.damage_detection_done = False
        self.flying_trajectory = False
        self.i = 0

        self.drone_trajectory = MultiDOFJointTrajectory()
        self.tooltip_trajectory = MultiDOFJointTrajectory()

        #don't start services until map and drone pose are available
        rospy.wait_for_message(map_topic_name, PointCloud2) 
        rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        rospy.loginfo('Services are ready!')

        # init services
        detection_service = rospy.Service('detect_damage', detectDamage, self.handle_detect_damage)
        planning_service = rospy.Service('plan_repair_trajectory', repairTrajectory, self.handle_repair_trajectory)
        transform_service = rospy.Service('get_drone_trajectory', droneTrajectory, handle_drone_trajectory)
        TOPPRA_service = rospy.Service('get_TOPPRA_trajectory', TOPPRATrajectory, handle_TOPPRA_trajectory)

        sp_timer = rospy.Timer(rospy.Duration(1.0/30), self.sp_timer_cb, reset=True)
        rospy.spin()


    def sp_timer_cb(self, event):
        drone_pose = PoseStamped()
        tip_pose = PoseStamped()
        if self.mavros_state.mode == 'OFFBOARD':
            if self.flying_trajectory is False: 
                self.flying_trajectory = True
                self.trajectory_start_time = rospy.Time.now()
            if (self.i > (len(self.drone_trajectory.points) - 1)) or (self.i > (len(self.tooltip_trajectory.points) - 1)):
                self.i = 0
                self.flying_trajectory = False
            else:
                while (self.drone_trajectory.points[self.i].time_from_start.to_sec() + self.trajectory_start_time.to_sec()) < rospy.Time.now().to_sec():
                    self.i += 1  
                drone_pose = PoseStamped()
                drone_pose.header.frame_id = self.drone_trajectory.header.frame_id
                drone_pose.header.stamp = rospy.Time.now()
                drone_pose.pose.position = self.drone_trajectory.points[self.i].transforms[0].translation
                drone_pose.pose.orientation = self.drone_trajectory.points[self.i].transforms[0].rotation

                tip_pose = PoseStamped()
                tip_pose.header.frame_id = self.tooltip_trajectory.header.frame_id
                tip_pose.header.stamp = rospy.Time.now()
                tip_pose.pose.position = self.tooltip_trajectory.points[self.i].transforms[0].translation
                tip_pose.pose.orientation = self.tooltip_trajectory.points[self.i].transforms[0].rotation

                rospy.loginfo('Speed: '+ str(np.linalg.norm(rnp.numpify(self.drone_trajectory.points[self.i+1].transforms[0].translation) - rnp.numpify(self.drone_trajectory.points[self.i].transforms[0].translation)) * self.trajectory_frequency))
        else:
            drone_pose = PoseStamped()
            drone_pose.header.frame_id = self.map_frame
            drone_pose.header.stamp = rospy.Time.now()
            drone_pose.pose = self.mavros_pose.pose

            tip_pose = PoseStamped()
            tip_pose.header.frame_id = self.map_frame
            tip_pose.header.stamp = rospy.Time.now()
            tip_pose.pose = self.mavros_pose.pose

        self.pub_drone_sp.publish(drone_pose)
        self.pub_tip_sp.publish(tip_pose)

    def state_cb(self, state_msg):
        self.mavros_state = state_msg
        if self.mavros_state.mode == 'OFFBOARD':
            self.pub_tooltip_state.publish(String("STAB_6DOF"))
        elif self.mavros_state.mode == 'POSCTL':
            self.pub_tooltip_state.publish(String("STAB_3DOF"))
        else:
            self.pub_tooltip_state.publish(String("RETRACTED"))

    def init_pose_cb(self, msg):
        # generate initial pose extimate and do damage detection from rviz
        init_pose = PoseStamped()
        init_pose.header = msg.header
        init_pose.pose = msg.pose.pose
         # call damage detection service
        rospy.wait_for_service('detect_damage')
        get_damage = rospy.ServiceProxy('detect_damage', detectDamage)
        request = detectDamageRequest()
        request.init_pose = init_pose
        response = get_damage(request)

    def map_cb(self, msg):
        self.map_pc2 = msg #save map to object - only bother converting when service is called

    def mavros_pose_cb(self, msg):
        self.mavros_pose = msg #save mavros pose to object

    def handle_repair_trajectory(self, req):
        rospy.loginfo('Trajectory planner called!')

        # get repair pose according to desired index
        repair_pose = self.pose_array.poses[req.damage_index]

        # get current tooltip pose by tf lookup
        # get tf from drone frame to tooltip frame
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        got_tf = False
        while not got_tf:
            try:
                tf = tfBuffer.lookup_transform(self.map_frame, self.tip_frame, time=rospy.Time.now(), timeout=rospy.Duration(5))
                got_tf = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("tf dropped - retrying..")

        start_pose = Pose()
        start_pose.position = tf.transform.translation
        start_pose.orientation = tf.transform.rotation

        pose_array = PoseArray()
        pose_array.header.frame_id = self.map_frame
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses.append(start_pose)
        pose_array.poses.append(repair_pose)

        # calculate trajectory parametrisation with toppra
        rospy.wait_for_service('get_TOPPRA_trajectory')
        get_traj = rospy.ServiceProxy('get_TOPPRA_trajectory', TOPPRATrajectory)
        request = TOPPRATrajectoryRequest()
        request.frequency = self.trajectory_frequency
        request.max_vel = self.max_vel
        request.max_acc = self.max_acc
        request.max_yawrate = self.max_yawrate
        request.max_yawrate_dot = self.max_yawrate_dot
        request.poses = pose_array
        toppra_response = get_traj(request)

        outbound_trajectory = toppra_response.trajectory

        pose_array = PoseArray()
        pose_array.header.frame_id = self.map_frame
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses.append(repair_pose)
        pose_array.poses.append(start_pose)

        get_traj = rospy.ServiceProxy('get_TOPPRA_trajectory', TOPPRATrajectory)
        request = TOPPRATrajectoryRequest()
        request.frequency = self.trajectory_frequency
        request.max_vel = self.max_vel
        request.max_acc = self.max_acc
        request.max_yawrate = self.max_yawrate
        request.max_yawrate_dot = self.max_yawrate_dot
        request.poses = pose_array
        toppra_response = get_traj(request)

        inbound_trajectory = toppra_response.trajectory

        delay = rospy.Duration(self.hover_time) + outbound_trajectory.points[len(outbound_trajectory.points)-1].time_from_start
        for i in range(len(inbound_trajectory.points)-1):
            inbound_trajectory.points[i].time_from_start += delay

        trajectory = MultiDOFJointTrajectory()
        trajectory.header = outbound_trajectory.header
        trajectory.joint_names = outbound_trajectory.joint_names
        trajectory.points = outbound_trajectory.points + inbound_trajectory.points

        self.pub_tip_trajectory.publish(trajectory)

        # output a path message for visualisation
        viz_path = Path()
        viz_path.header = trajectory.header
        for i in range(len(trajectory.points)):
            pose = PoseStamped()
            pose.header.frame_id = trajectory.header.frame_id
            pose.header.stamp = viz_path.header.stamp
            pose.pose.position.x = trajectory.points[i].transforms[0].translation.x
            pose.pose.position.y = trajectory.points[i].transforms[0].translation.y
            pose.pose.position.z = trajectory.points[i].transforms[0].translation.z
            pose.pose.orientation.x = trajectory.points[i].transforms[0].rotation.x
            pose.pose.orientation.y = trajectory.points[i].transforms[0].rotation.y
            pose.pose.orientation.z = trajectory.points[i].transforms[0].rotation.z
            pose.pose.orientation.w = trajectory.points[i].transforms[0].rotation.w
            viz_path.poses.append(pose)
        self.pub_tip_trajectory_viz.publish(viz_path)

        # calculate an offset drone trajectory
        #get offset drone trajectory
        rospy.wait_for_service('get_drone_trajectory')
        get_traj = rospy.ServiceProxy('get_drone_trajectory', droneTrajectory)
        request = droneTrajectoryRequest()
        request.drone_body_frame_id = self.drone_frame
        request.tooltip_frame_id = self.tip_frame
        request.toolpath_trajectory = trajectory
        drone_trajectory_response = get_traj(request)
        self.pub_drone_trajectory.publish(drone_trajectory_response.drone_trajectory)

        self.drone_trajectory = drone_trajectory_response.drone_trajectory
        self.tooltip_trajectory = trajectory

        # output a second path message
        viz_path = Path()
        viz_path.header = drone_trajectory_response.drone_trajectory.header
        for i in range(len(drone_trajectory_response.drone_trajectory.points)):
            pose = PoseStamped()
            pose.header.frame_id = drone_trajectory_response.drone_trajectory.header.frame_id
            pose.header.stamp = viz_path.header.stamp
            pose.pose.position.x = drone_trajectory_response.drone_trajectory.points[i].transforms[0].translation.x
            pose.pose.position.y = drone_trajectory_response.drone_trajectory.points[i].transforms[0].translation.y
            pose.pose.position.z = drone_trajectory_response.drone_trajectory.points[i].transforms[0].translation.z
            pose.pose.orientation.x = drone_trajectory_response.drone_trajectory.points[i].transforms[0].rotation.x
            pose.pose.orientation.y = drone_trajectory_response.drone_trajectory.points[i].transforms[0].rotation.y
            pose.pose.orientation.z = drone_trajectory_response.drone_trajectory.points[i].transforms[0].rotation.z
            pose.pose.orientation.w = drone_trajectory_response.drone_trajectory.points[i].transforms[0].rotation.w
            viz_path.poses.append(pose)
        self.pub_drone_trajectory_viz.publish(viz_path)

        resp = trajectory
        return resp
            
    def handle_detect_damage(self, req):
        rospy.loginfo("Detection service called!")
        # convert ros msgs to o3d friendly formats
        map_pc = pc2_to_o3d(self.map_pc2)
        init_transformation = posestamped_to_trans_matrix(req.init_pose)

        # transform undamaged model pointcloud to location of initial guess
        init_model_pc = copy.deepcopy(self.model_pc).transform(init_transformation)

        #publish visualisation of initial pose
        self.pub_init.publish(o3d_to_pc2(init_model_pc))

        # perform icp registration
        rospy.loginfo("Attempting alignment...")
        result = o3d.pipelines.registration.registration_icp(
            voxel_down_sample(init_model_pc, self.voxel_size), voxel_down_sample(map_pc, self.voxel_size),
            1.0, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.max_icp_iterations))
        
        transformation = result.transformation
        rospy.loginfo('Tranformation matrix is:')
        rospy.loginfo(transformation)

        # publish visualisation of transformed pointcloud (and write to file)
        aligned_model_pc = copy.deepcopy(init_model_pc).transform(transformation)
        o3d.io.write_point_cloud(os.path.abspath('../pointclouds/model_aligned.pcd'), aligned_model_pc)
        self.pub_result.publish(o3d_to_pc2(aligned_model_pc))
        rospy.loginfo("Alignment done!")
        
        # crop map pointcloud based on model limits (and also publish and write to file)
        min = aligned_model_pc.get_min_bound()
        max = aligned_model_pc.get_max_bound()
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min, max_bound=max)
        map_pc_cropped = copy.deepcopy(map_pc).crop(bbox)
        o3d.io.write_point_cloud(os.path.abspath('../pointclouds/map_cropped.pcd'), map_pc_cropped)
        self.pub_cropped_map.publish(o3d_to_pc2(map_pc_cropped))

        # find distance from map points to undamaged model, keep any above a certain distance threshold
        threshold = self.eps #setting threshold to cluster radiius seems to yield good results
        distance = np.asarray(map_pc_cropped.compute_point_cloud_distance(aligned_model_pc))
        outliers = np.where(distance > threshold)[0]
        rospy.loginfo("Damaged points are at:")
        rospy.loginfo(outliers)
        outlier_pcd = map_pc_cropped.select_by_index(outliers)

        # publish 'damaged' points
        self.pub_damage.publish(o3d_to_pc2(outlier_pcd))

        # run DBSCAN clustering algorithm
        rospy.loginfo('Clustering...')
        labels = np.array(outlier_pcd.cluster_dbscan(eps=self.eps, min_points=self.min_points))
        num_clusters = labels.max() + 1
        rospy.loginfo('Found '+str(num_clusters)+' clusters.')

        # for each cluster, a robot pose for repair must be determined
        marker_array = MarkerArray()
        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = self.map_frame
        self.pose_array.header.stamp = rospy.Time.now()
        rospy.loginfo('Calculating poses...')
        for i in range(num_clusters):
            # extract points belonging to the i-th cluster
            cluster_indices = np.where(labels == i)[0]
            cluster_pcd = outlier_pcd.select_by_index(cluster_indices)

            # compute the centroid of the cluster
            centroid = np.asarray(cluster_pcd.get_center())

            # we now have a single point for each identified damaged area
            # next we compute the closest distance to the model and then calculate normal vector
            distances = np.asarray(aligned_model_pc.compute_point_cloud_distance(o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector([centroid]))))
            nearest_point_index = np.argmin(distances)
            aligned_model_pc.estimate_normals()
            normal = aligned_model_pc.normals[nearest_point_index]

            # get euler angles for pitch and yaw rotation from map pose - we don't care about roll
            x_vector = np.asarray([1, 0, 0])
            normal_xy = np.asarray([normal[0], normal[1], 0])
            yaw = np.arccos(np.dot(normal_xy, x_vector,) / (np.linalg.norm(x_vector) * np.linalg.norm(normal_xy)))
            pitch = np.arccos(np.dot(normal_xy, normal) / (np.linalg.norm(normal_xy) * np.linalg.norm(normal)))

            # transform into manipulator tooltip frame
            q = quaternion_from_euler(np.pi/2, pitch, yaw-np.pi/2)

            normal_pose = Pose()
            normal_pose.position.x = aligned_model_pc.points[nearest_point_index][0]
            normal_pose.position.y = aligned_model_pc.points[nearest_point_index][1]
            normal_pose.position.z = aligned_model_pc.points[nearest_point_index][2]
            normal_pose.orientation.x = q[0]
            normal_pose.orientation.y = q[1]
            normal_pose.orientation.z = q[2]
            normal_pose.orientation.w = q[3]
            self.pose_array.poses.append(normal_pose)
            
            # publish marker message to enable the operator to choose a numbered point
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = self.map_frame
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = 0.15
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.pose.position.x = centroid[0]
            marker.pose.position.y = centroid[1]
            marker.pose.position.z = centroid[2]
            marker.text = str(i)
            marker_array.markers.append(marker)
        
        self.pub_markers.publish(marker_array)   
        rospy.loginfo("Done!")

        # output poses in service response
        resp = detectDamageResponse()
        resp.target_poses = self.pose_array
        self.pub_poses.publish(self.pose_array)
        return resp

def handle_drone_trajectory(req):
    # service generates a drone trajectory offset from a supplied tooltip trajectory. x/y/z and yaw commands
    # are preserved and pitch/roll angles are ignored
    rospy.loginfo("Offset drone trajectory requested")

    drone_frame = req.drone_body_frame_id
    tip_frame = req.tooltip_frame_id

    # get tf from drone frame to tooltip frame
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    got_tf = False
    while not got_tf:
        try:
            tf = tfBuffer.lookup_transform(tip_frame, drone_frame, time=rospy.Time.now(), timeout=rospy.Duration(5))
            got_tf = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("tf dropped - retrying..")

    tf_rotation = TransformStamped()
    tf_rotation.header = tf.header
    tf_rotation.child_frame_id = tf.child_frame_id
    tf_rotation.transform.rotation = tf.transform.rotation
    tf_rotation.transform.translation = Vector3(0,0,0)

    tf_inv = tfBuffer.lookup_transform(drone_frame, tip_frame, time=rospy.Time.now(), timeout=rospy.Duration(5))
    tf_inv_rotation = TransformStamped()
    tf_inv_rotation.header = tf_inv.header
    tf_inv_rotation.child_frame_id = tf_inv.child_frame_id
    tf_inv_rotation.transform.rotation = tf_inv.transform.rotation
    tf_inv_rotation.transform.translation = Vector3(0,0,0)

    resp = droneTrajectoryResponse()
    resp.drone_trajectory = MultiDOFJointTrajectory()
    resp.drone_trajectory.header.frame_id = req.toolpath_trajectory.header.frame_id
    resp.drone_trajectory.header.stamp = rospy.Time.now()

    for i in range(len(req.toolpath_trajectory.points)): 
        traj = do_transform_transform(tf_rotation, req.toolpath_trajectory.points[i].transforms[0])
        (roll, pitch, yaw) = euler_from_quaternion([traj.rotation.x,
                                                    traj.rotation.y,
                                                    traj.rotation.z,
                                                    traj.rotation.w])
        q = quaternion_from_euler(yaw,0,0,'szyx')
        traj.rotation = Quaternion(q[0],q[1],q[2],q[3])
        traj = do_transform_transform(tf_inv_rotation, traj)
        drone_transform = do_transform_transform(tf, traj)
        
        drone_trajectory_point = MultiDOFJointTrajectoryPoint()
        drone_trajectory_point.time_from_start = req.toolpath_trajectory.points[i].time_from_start
        drone_trajectory_point.transforms.append(drone_transform)
        drone_trajectory_point.velocities.append(req.toolpath_trajectory.points[i].velocities[0])
        drone_trajectory_point.accelerations.append(req.toolpath_trajectory.points[i].accelerations[0])
        resp.drone_trajectory.points.append(drone_trajectory_point)
    rospy.loginfo("Trajectory ready")
    return resp
 
def handle_TOPPRA_trajectory(req):
    # service generates a smooth, interpolated, time-optimal trajectory from an array of poses using TOPPRA package
    num_poses = len(req.poses.poses)  
    rospy.loginfo("Trajectory requested. Interpolating " + str(num_poses) + " poses at " + str(req.frequency) + "Hz.")

    way_pts = np.zeros((num_poses, 6))
    for i in range(num_poses):
        (roll, pitch, yaw) = euler_from_quaternion([req.poses.poses[i].orientation.x,req.poses.poses[i].orientation.y,req.poses.poses[i].orientation.z,req.poses.poses[i].orientation.w])                                             
        way_pts[i,:] = [req.poses.poses[i].position.x, req.poses.poses[i].position.y, req.poses.poses[i].position.z, roll, pitch, yaw]    
    ss = np.linspace(0, 1, num_poses)
    
    amax = req.max_acc
    vmax = req.max_vel
    max_yawrate = np.deg2rad(req.max_yawrate)
    max_yawrate_dot = np.deg2rad(req.max_yawrate_dot)
    vlims = [vmax, vmax, vmax, max_yawrate, max_yawrate, max_yawrate]
    alims = [amax, amax, amax, max_yawrate_dot, max_yawrate_dot, max_yawrate_dot]
    
    # cretae spline interpolant of way points
    path = ta.SplineInterpolator(ss, way_pts)

    # populate toppra constraints
    pc_vel = ta.constraint.JointVelocityConstraint(vlims)
    pc_acc = ta.constraint.JointAccelerationConstraint(alims)
    instance = ta.algorithm.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")

    # compute parameterisation
    traj = instance.compute_trajectory(0, 0)

    n_samples = int(traj.duration * req.frequency)
    ts_sample = np.linspace(0, traj.duration, n_samples)
    qs_sample = traj(ts_sample, 0) #position setpoints
    qds_sample = traj(ts_sample, 1) #velocity setpoints
    qdds_sample = traj(ts_sample, 2) #acceleration setpoints

    # populate a trajectory message to be published
    resp = TOPPRATrajectoryResponse()
    resp.trajectory = MultiDOFJointTrajectory()
    resp.trajectory.header.frame_id = req.poses.header.frame_id
    resp.trajectory.header.stamp = rospy.Time.now()

    for i in range(n_samples):
        trans = Transform()
        trans.translation.x = qs_sample[i,0]
        trans.translation.y = qs_sample[i,1]
        trans.translation.z = qs_sample[i,2]
        q = quaternion_from_euler(qs_sample[i,3], qs_sample[i,4], qs_sample[i,5])
        trans.rotation.x = q[0]
        trans.rotation.y = q[1]
        trans.rotation.z = q[2]
        trans.rotation.w = q[3]

        vel = Twist()
        vel.linear.x = qds_sample[i,0]
        vel.linear.y = qds_sample[i,1]
        vel.linear.z = qds_sample[i,2]
        vel.angular.x = qds_sample[i,3]
        vel.angular.y = qds_sample[i,4]
        vel.angular.z = qds_sample[i,5]

        accel = Twist()
        accel.linear.x = qdds_sample[i,0]
        accel.linear.y = qdds_sample[i,1]
        accel.linear.z = qdds_sample[i,2]
        accel.angular.x = qdds_sample[i,3]
        accel.angular.y = qdds_sample[i,4]
        accel.angular.z = qdds_sample[i,5]

        trajectory_point = MultiDOFJointTrajectoryPoint()
        trajectory_point.transforms.append(trans)
        trajectory_point.velocities.append(vel)
        trajectory_point.accelerations.append(accel)
        trajectory_point.time_from_start = rospy.Duration(i / req.frequency)

        resp.trajectory.points.append(trajectory_point)
    
    rospy.loginfo("Trajectory ready")
    return resp

def get_pc_from_mesh(dir, samples):
    mesh = o3d.io.read_triangle_mesh(dir)
    pc = mesh.sample_points_uniformly(samples)
    return pc

def pc2_to_o3d(pc2=PointCloud2()):
    pc_np = rnp.numpify(pc2)
    pc_arr = np.zeros([len(pc_np), 3])
    pc_arr[:,0] = pc_np['x']
    pc_arr[:,1] = pc_np['y']
    pc_arr[:,2] = pc_np['z']
    pc_arr = pc_arr[~np.isinf(pc_arr).any(axis=1)]
    pc_arr = pc_arr[~np.isnan(pc_arr).any(axis=1)]
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pc_arr)
    return pc

def o3d_to_pc2(o3d_pc=o3d.geometry.PointCloud()):
    #convert back to structured array
    pc_np = np.asarray(o3d_pc.points)
    pc_arr = np.zeros(np.shape(pc_np)[0], dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        # ('rgb', np.float32),
        ])
    pc_arr['x'] = pc_np[:, 0]
    pc_arr['y'] = pc_np[:, 1]
    pc_arr['z'] = pc_np[:, 2]
    #msgify back to pointcloud2
    pc_header = Header(stamp=rospy.Time.now(), frame_id='camera_init')
    pc2 = rnp.msgify(PointCloud2, pc_arr, stamp=pc_header.stamp, frame_id=pc_header.frame_id)
    return pc2

def posestamped_to_trans_matrix(pose_stamped_msg=PoseStamped()):
    mat = rnp.numpify(pose_stamped_msg.pose)
    return mat

def trans_matrix_to_posestamped(trans_matrix=np.zeros((4,4))):
    pose = rnp.msgify(Pose, trans_matrix)
    header = Header(stamp=rospy.Time.now(), frame_id='camera_init')
    posestamped = PoseStamped(header=header, pose=pose)
    return posestamped

def voxel_down_sample(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    return pcd_down

def do_transform_transform(transform1, transform2):
    #if either input is not ...Stamped() version of message, do conversion:
    if isinstance(transform1, Transform):
        transform1 = TransformStamped(transform=transform1)
    if isinstance(transform2, Transform):
        transform2 = TransformStamped(transform=transform2)
    #check that type is now correct, if not output an error readable by humans:
    if isinstance(transform1, TransformStamped) and isinstance(transform2, TransformStamped):
        pose = PoseStamped()
        pose.pose.position = transform1.transform.translation
        pose.pose.orientation = transform1.transform.rotation
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform2)
        transform1_transformed = Transform()
        transform1_transformed.translation = pose_transformed.pose.position
        transform1_transformed.rotation = pose_transformed.pose.orientation
        return transform1_transformed
    else:
        rospy.logerr("Incorrect types used in do_transform_transform()")
        rospy.logerr("Must be Tranform() or TransformStamped()")

def do_transform_pose(pose, transform):
    #if either input is not ...Stamped() version of message, do conversion:
    if isinstance(pose, Pose):
        pose = PoseStamped(pose=pose)
    if isinstance(transform, Transform):
        transform = TransformStamped(transform=transform)
    #check that type is now correct, if not output an error readable by humans:
    if isinstance(pose, PoseStamped) and isinstance(transform, TransformStamped):
        posestamped_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
        pose_transformed = Pose()
        pose_transformed = posestamped_transformed.pose
        return pose_transformed
    else:
        rospy.logerr("Incorrect types used in do_transform_pose()")
        rospy.logerr("Must be Pose() or PoseStamped()")   

def do_transform_twist(twist, transform):
    #if either input is not ...Stamped() version of message, do conversion:
    if isinstance(twist, Twist):
        twist = TwistStamped(twist=twist)
    if isinstance(transform, Transform):
        transform = TransformStamped(transform=transform)
    #check that type is now correct, if not output an error readable by humans:
    if isinstance(twist, TwistStamped) and isinstance(transform, TransformStamped):
        wrench = WrenchStamped()
        wrench.wrench.force = twist.twist.linear
        wrench.wrench.torque = twist.twist.angular
        wrench_transformed = tf2_geometry_msgs.do_transform_wrench(
            wrench, transform)
        twist_transformed = Twist()
        twist_transformed.linear = wrench_transformed.wrench.force
        twist_transformed.angular = wrench_transformed.wrench.torque
        return twist_transformed
    else:
        rospy.logerr("Incorrect types used in do_transform_twist()")
        rospy.logerr("Must be Twist()/TwistStamped() and Transform()/TransformStamped()")

if __name__ == "__main__":
    rospy.init_node('damage_detection_service_client')
    dD = DamageDetection()