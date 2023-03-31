#! /usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import ros_numpy as rnp
import toppra as ta
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, TransformStamped, Transform, Twist, Vector3, Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Path
from tricopter.srv import *
    
class TrajectoryGeneration:
    def __init__(self):
        # get config values from parameter server
        self.tip_frame = rospy.get_param('/trajectory_planner/tooltip_frame')
        self.drone_frame = rospy.get_param('/trajectory_planner/drone_frame')
        self.map_frame = rospy.get_param('/damage_detection/map_frame')
        self.odom_frame = rospy.get_param('/damage_detection/odom_frame')
        self.trajectory_frequency = rospy.get_param('/trajectory_planner/frequency')
        self.max_vel = rospy.get_param('/trajectory_planner/max_vel')
        self.max_acc = rospy.get_param('/trajectory_planner/max_acc')
        self.max_yawrate = rospy.get_param('/trajectory_planner/max_yawrate')
        self.max_yawrate_dot = rospy.get_param('/trajectory_planner/max_yawrate_dot')

        self.hover_time = rospy.get_param('/trajectory_planner/hover_time')

        # init publishers and subscribers
        self.pub_poses = rospy.Publisher('/damage_poses', PoseArray, queue_size=1)
        self.pub_tip_trajectory = rospy.Publisher('/repair_trajectory/tooltip', MultiDOFJointTrajectory, queue_size=1)
        self.pub_tip_trajectory_viz = rospy.Publisher('/repair_trajectory/tooltip/viz', Path, queue_size=1)
        self.pub_drone_trajectory = rospy.Publisher('/repair_trajectory/drone', MultiDOFJointTrajectory, queue_size=1)
        self.pub_drone_trajectory_viz = rospy.Publisher('/repair_trajectory/drone/viz', Path, queue_size=1)

        sub_drone_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavros_pose_cb, queue_size=1)
        sub_repair_poses = rospy.Subscriber('/damage_poses', PoseArray, self.repair_poses_cb, queue_size=1)

        #don't start services until drone pose is available
        rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        rospy.loginfo('Drone position estimate is ready!')

        # init services
        planning_service = rospy.Service('plan_repair_trajectory', repairTrajectory, self.handle_repair_trajectory)
        transform_service = rospy.Service('get_drone_trajectory', droneTrajectory, handle_drone_trajectory)
        TOPPRA_service = rospy.Service('get_TOPPRA_trajectory', TOPPRATrajectory, handle_TOPPRA_trajectory)
        rospy.spin()

    def mavros_pose_cb(self, msg):
        self.mavros_pose = msg #save mavros pose to object

    def repair_poses_cb(self, msg):
        self.pose_array = msg

    def handle_repair_trajectory(self, req):
        rospy.loginfo('Trajectory planner called!')

        # get repair pose according to desired index
        repair_pose = self.pose_array.poses[req.damage_index]
        repair_pose.position.z += 0.2

        # get current tooltip pose by tf lookup
        # get tf from drone frame to tooltip frame
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        got_tf = False
        while not got_tf:
            try:
                tf = tfBuffer.lookup_transform(self.odom_frame, self.tip_frame, time=self.mavros_pose.header.stamp, timeout=rospy.Duration(1))
                got_tf = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("tf dropped - retrying..")

        start_pose = Pose()
        start_pose.position = tf.transform.translation
        start_pose.orientation = tf.transform.rotation

        pose_array = PoseArray()
        pose_array.header.frame_id = self.odom_frame
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses.append(start_pose)
        pose_array.poses.append(repair_pose)

        # calculate trajectory parametrisation with toppra
        rospy.wait_for_service('get_TOPPRA_trajectory')
        get_traj = rospy.ServiceProxy('get_TOPPRA_trajectory', TOPPRATrajectory)
        request = TOPPRATrajectoryRequest(frequency=self.trajectory_frequency,
                                          max_vel = self.max_vel,
                                          max_acc = self.max_acc,
                                          max_yawrate = self.max_yawrate,
                                          max_yawrate_dot = self.max_yawrate_dot,
                                          poses = pose_array)
        toppra_response = get_traj(request)

        outbound_trajectory = toppra_response.trajectory

        pose_array = PoseArray()
        pose_array.header.frame_id = self.map_frame
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses.append(repair_pose)
        pose_array.poses.append(start_pose)

        get_traj = rospy.ServiceProxy('get_TOPPRA_trajectory', TOPPRATrajectory)
        request = TOPPRATrajectoryRequest(frequency=self.trajectory_frequency,
                                          max_vel = self.max_vel,
                                          max_acc = self.max_acc,
                                          max_yawrate = self.max_yawrate,
                                          max_yawrate_dot = self.max_yawrate_dot,
                                          poses = pose_array)
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
            
def handle_drone_trajectory(req):
    # service generates a drone trajectory offset from a supplied tooltip trajectory. x/y/z and yaw commands
    # are preserved and pitch/roll angles are ignored
    rospy.loginfo("Offset drone trajectory requested")

    drone_frame = req.drone_body_frame_id
    tip_frame = req.tooltip_frame_id

    # get (static) tf from drone frame to tooltip frame
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    got_tf = False
    while not got_tf:
        try:
            tf = tfBuffer.lookup_transform(tip_frame, drone_frame, time=rospy.Time.now(), timeout=rospy.Duration(1))
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
  
if __name__ == "__main__":
    rospy.init_node('trajectory_generation_service_client')
    tG = TrajectoryGeneration()