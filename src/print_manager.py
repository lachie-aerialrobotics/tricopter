#! /usr/bin/env python

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from std_msgs.msg import Float32, String
from controller_msgs.msg import FlatTarget
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped, TransformStamped, Transform, Twist, WrenchStamped
from mavros_msgs.msg import State, ExtendedState
from nav_msgs.msg import Path
# from mavros_msgs.srv import CommandTOL
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tricopter.srv import *

import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo

import statemachine as st

class droneStateMachine(st.StateMachine):
    #define all drone states
    idle = st.State("idle")
    takeoff = st.State("takeoff")
    landing = st.State("landing")
    hold = st.State("hold")
    printing = st.State("printing")
    manual = st.State("manual")

    #define state transitions
    startTakeoff = idle.to(takeoff)
    finishTakeoff = takeoff.to(hold)
    startLanding = hold.to(landing)
    finishLanding = landing.to(idle)

    #callbacks on state transitions
    def on_startTakeoff(self):
        rospy.loginfo("Takeoff initiated")


class printManager:
    def __init__(self):

        self.rate = 30
        # self.tol_speed = 1.0
        # self.transition_speed = 1.0
        # self.takeoff_hgt = 1.0
        # self.printer_state = "IDLE"
        # self.layer_complete = False

        # self.print_init_x = 1.0
        # self.print_init_y = 0.0
        # self.print_init_z = 2.0
        # self.print_yaw = 0.0

        # self.target = FlatTarget()
        # self.target.header.frame_id = "map"

        # publishers to geometric controller
        self.geo_pose_pub = rospy.Publisher(
            'reference/flatsetpoint', FlatTarget, queue_size=1, tcp_nodelay=True)
        self.geo_yaw_pub = rospy.Publisher(
            'reference/yaw', Float32, queue_size=1, tcp_nodelay=True)

        # publishers to manipulator
        self.pub_tooltip_state = rospy.Publisher(
            '/manipulator/state',  String, queue_size=1, tcp_nodelay=True)
        self.pub_tooltip_pose = rospy.Publisher(
            '/tooltip_setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.pub_tooltip_twist = rospy.Publisher(
            '/tooltip_setpoint/velocity', TwistStamped, queue_size=1, tcp_nodelay=True)

        # publishers for visualisation only
        self.pub_toolpath_viz = rospy.Publisher('/viz/toolpath', Path, queue_size=1)

        self.pub_dronepath_viz = rospy.Publisher('/viz/dronepath', Path, queue_size=1)

        # drone state subscriber
        state_sub = rospy.Subscriber(
            '/mavros/state', State, self.state_cb, queue_size=1, tcp_nodelay=True)
        ext_state_sub = rospy.Subscriber(
            '/mavros/extended_state', ExtendedState, self.ext_state_cb, queue_size=1, tcp_nodelay=True)
        local_position_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self.local_pos_cb, queue_size=1, tcp_nodelay=True)

        # services for trajectory generation
        TOPPRA_service = rospy.Service('get_TOPPRA_trajectory', TOPPRATrajectory, handle_TOPPRA_trajectory)
        transform_trajectory_service = rospy.Service('get_transformed_trajectory', transformTrajectory, handle_transform_trajectory)
        drone_trajectory_service = rospy.Service('get_drone_trajectory', droneTrajectory, handle_drone_trajectory)

        rospy.wait_for_message('/mavros/state', State)
        rospy.wait_for_message('/mavros/extended_state', ExtendedState)
        rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)

        #temp:
        self.end_pose = Pose()
        self.end_pose.position.x = 0.0
        self.end_pose.position.y = 1.0
        self.end_pose.position.z = 4.0
        self.end_pose.orientation.x = 0.0
        self.end_pose.orientation.y = 0.0
        self.end_pose.orientation.z = 0.0
        self.end_pose.orientation.w = 1.0

        # while True:
        #     rospy.wait_for_service('get_TOPPRA_trajectory')
        #     calc_toolpath = rospy.ServiceProxy('get_TOPPRA_trajectory', TOPPRATrajectory)
        #     request = TOPPRATrajectoryRequest()
        #     request.max_yawrate = 20
        #     request.max_yawrate_dot = 50
        #     request.max_vel = 1.0
        #     request.max_acc = 0.2
        #     request.frequency = 10
        #     request.poses.poses.append(self.local_pose)
        #     request.poses.poses.append(self.end_pose)
        #     response = calc_toolpath(request)

        #     rospy.wait_for_service('get_transformed_trajectory')
        #     calc_toolpath = rospy.ServiceProxy('get_transformed_trajectory', transformTrajectory)
        #     request = transformTrajectoryRequest()
        #     request.frame_id = "map"
        #     request.transformed_frame_id = "map"
        #     request.toolpath_trajectory = response.trajectory
        #     response = calc_toolpath(request)
            
        #     publish_viz_path(response.toolpath_trajectory, self.pub_toolpath_viz)

        #     rospy.wait_for_service('get_drone_trajectory')
        #     calc_toolpath = rospy.ServiceProxy('get_drone_trajectory', droneTrajectory)
        #     request = droneTrajectoryRequest()
        #     request.drone_body_frame_id = "base_link"
        #     request.tooltip_frame_id = "tooltip_init"
        #     request.toolpath_trajectory = response.toolpath_trajectory
        #     response = calc_toolpath(request)

        #     publish_viz_path(response.drone_trajectory, self.pub_dronepath_viz)

        #     rospy.sleep(2)


        # rospy.wait_for_service('get_toolpath')
        # calc_toolpath = rospy.ServiceProxy('get_toolpath', layerTrajectory)
        # trajectory_request = layerTrajectoryRequest()
        # trajectory_request.frame_id = "print_origin"
        # trajectory_request.layer = 0
        # calc_toolpath(trajectory_request)

        #timer callback to ensure drone setpoints are broadcasted at a constant rate
        timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.timer_cb, reset=True)

        # while not rospy.is_shutdown():
        #     if self.printer_state == "IDLE":
        #         pass

        #     elif self.printer_state == "TAKEOFF":
        #         pass

        #     elif self.printer_state == "HOVER_HOME":
        #         rospy.wait_for_service('get_TOPPRA_trajectory')
        #         calc_toolpath = rospy.ServiceProxy('get_TOPPRA_trajectory', TOPPRATrajectory)
        #         request = TOPPRATrajectoryRequest()
        #         request.max_yawrate = 20
        #         request.max_yawrate_dot = 50
        #         request.max_vel = 1.0
        #         request.max_acc = 0.2
        #         request.frequency = 10
        #         request.poses.poses.append(self.local_pose)
        #         request.poses.poses.append(self.end_pose)
        #         reponse = calc_toolpath(request)


        #     elif self.printer_state == "MOVE_TO_PRINT":
        #         pass

        #     elif self.printer_state == "HOVER_PRINT":
        #         pass

        #     elif self.printer_state == "PRINTING":
        #         pass

        #     elif self.printer_state == "RETURN TO HOME":
        #         pass
            
        #     elif self.printer_state == "LANDING":
        #         pass
                    


    def timer_cb(self, event):
        self.target.header.stamp = rospy.Time.now()

        if self.mavros_state.mode != "OFFBOARD":
            if self.mavros_state.armed:
                rospy.loginfo("MANUAL TAKEOVER!")
                self.printer_state = "MANUAL"
            else:
                self.printer_state = "IDLE"

        if self.printer_state == "IDLE":
            self.pub_tooltip_state.publish(String("RETRACTED"))
            self.target.position = self.local_position
            self.yaw = self.local_yaw
            if not self.mavros_state.armed:
                rospy.loginfo("Ready to arm")
            else:
                rospy.loginfo("Armed! Taking off...")
                self.printer_state = "TAKEOFF"

        elif self.printer_state == "MANUAL":
            self.pub_tooltip_state.publish(String("HOME"))
            self.target.position = self.local_position
            self.yaw = self.local_yaw
            if self.mavros_state.mode == "OFFBOARD":
                rospy.loginfo("SWITCHING TO OFFBOARD")
                self.printer_state = "HOLD"

        elif self.printer_state == "TAKEOFF":
            self.pub_tooltip_state.publish(String("HOME"))
            if self.target.position.z < self.takeoff_hgt:
                self.target.position.z += self.tol_speed / self.rate
            else:
                self.target.position.z = self.takeoff_hgt
                if not self.landed:
                    rospy.loginfo("TAKEOFF DETECTED - HOLDING STATION")
                    self.printer_state = "HOLD"

        elif self.printer_state == "HOLD":
            self.pub_tooltip_state.publish(String("STAB_3DOF"))

            self.printer_state = "TRANSITION"

        elif self.printer_state == "TRANSITION":
            self.pub_tooltip_state.publish(String("STAB_3DOF"))
            

        elif self.printer_state == "PRINTING":
            self.pub_tooltip_state.publish(String("STAB_6DOF"))
            if not self.layer_complete:
                self.target.position.x = self.print_init_x
                self.target.position.y = self.print_init_y
                self.target.position.z = self.print_init_z
                self.yaw = self.print_yaw
                self.layer_complete = True
            else:
                self.printer_state = "HOLD_PRINT_SITE"

        elif self.printer_state == "LANDING":
            self.pub_tooltip_state.publish(String("HOME"))
            if self.target.position.z >= -0.5 and not self.landed:
                self.target.position.z += -self.tol_speed / self.rate
            if self.landed:
                rospy.loginfo("LANDING DETECTED -> PLEASE DISARM")
                self.printer_state == "IDLE"
                self.target.position = self.local_position
                self.yaw = self.local_yaw

        elif self.printer_state == "TRAJECTORY":
            pass


        self.geo_pose_pub.publish(self.target)
        self.geo_yaw_pub.publish(Float32(self.yaw))

    def state_cb(self, state_msg):
        rospy.wait_for_message('/mavros/extended_state', ExtendedState)
        self.mavros_state = state_msg

    def ext_state_cb(self, ext_state_msg):
        if ext_state_msg.landed_state == 1:
            self.landed = True
        elif ext_state_msg.landed_state == 2:
            self.landed = False

    def local_pos_cb(self, local_pose_msg):
        self.local_pose = local_pose_msg.pose
        self.local_position = local_pose_msg.pose.position
        (roll, pitch, yaw) = euler_from_quaternion([local_pose_msg.pose.orientation.x,
                                                    local_pose_msg.pose.orientation.y,
                                                    local_pose_msg.pose.orientation.z,
                                                    local_pose_msg.pose.orientation.w])
        self.local_yaw = yaw



# def handle_trajectory_generation(req):
#     layer = req.layer
#     frame_id = req.frame_id

#     rospy.loginfo("Trajectory for layer " + str(layer) + " requested")

#     #determine how many waypoints exist for this trajectory:
#     params = rospy.get_param_names()
#     filtered = list(filter(lambda k: ('/layer' + str(layer)) in k, params))
#     filtered = list(filter(lambda k: '/point' in k, filtered))
#     n_points = len(filtered)
#     if n_points == 0:
#         rospy.loginfo("No waypoints found, empty trajectory will be returned")

#     #fill in response
#     resp = layerTrajectoryResponse()
#     resp.trajectory = MultiDOFJointTrajectory()
#     resp.trajectory.header.frame_id = "map"
#     resp.trajectory.header.stamp = rospy.Time.now()
    
#     for i in range(n_points): #iterate through all the points in the parameter server
#         yaw = rospy.get_param('/layer' + str(layer) + '/yaw')
#         point = rospy.get_param('/layer' + str(layer) + '/point' + str(i))

#         trans = Transform()
#         trans.translation.x = point[0]
#         trans.translation.y = point[1]
#         trans.translation.z = point[2]
#         yaw_q = quaternion_from_euler(0.0, 0.0, np.deg2rad(yaw))
#         trans.rotation.x = yaw_q[0]
#         trans.rotation.y = yaw_q[1]
#         trans.rotation.z = yaw_q[2]
#         trans.rotation.w = yaw_q[3]

#         vel = Twist()
#         accel = Twist()

#         #transform point to desired print location     
#         tfBuffer = tf2_ros.Buffer()
#         tfListener = tf2_ros.TransformListener(tfBuffer)
#         tf_map2frame = tfBuffer.lookup_transform('map', frame_id, time=rospy.Time.now(), timeout=rospy.Duration(5))
#         trans_transformed = do_transform_transform(trans, tf_map2frame)
#         vel_transformed = do_transform_twist(vel, tf_map2frame)
#         accel_transformed = do_transform_twist(accel, tf_map2frame)

#         trajectory_point = MultiDOFJointTrajectoryPoint()
#         trajectory_point.transforms.append(trans_transformed)
#         trajectory_point.velocities.append(vel_transformed)
#         trajectory_point.accelerations.append(accel_transformed)
#         trajectory_point.time_from_start = rospy.Duration(0)

#         resp.trajectory.points.append(trajectory_point)
    
#     print(resp.trajectory.points[0].transforms)

#     rospy.loginfo("Trajectory returned")
#     return resp

def publish_viz_path(trajectory, publisher):
    # function to convert a MultiDOFJointTrajectory message to a Path message for visualisation in rviz
    if not (isinstance(trajectory, MultiDOFJointTrajectory) and isinstance(publisher, rospy.Publisher)):
        rospy.logerr("Incorrect input types for Path viz")
    else:
        viz_path = Path()
        viz_path.header.frame_id = trajectory.header.frame_id
        # print(trajectory)
        # print("TRAJECTORY HEADER FRAME_ID:")
        # print(trajectory.header.frame_id)
        viz_path.header.stamp = rospy.Time.now()
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
        publisher.publish(viz_path)

def handle_drone_trajectory(req):
    # service generates a drone trajectory offset from a supplied tooltip trajectory. x/y/z and yaw commands
    # are preserved and pitch/roll angles are ignored
    tip_traj = req.toolpath_trajectory

    drone_frame = req.drone_body_frame_id
    tip_frame = req.tooltip_frame_id

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    tf = tfBuffer.lookup_transform(drone_frame, tip_frame, time=rospy.Time.now(), timeout=rospy.Duration(5))

    drone_position_offset_local = Transform()
    drone_position_offset_local.translation.x = -tf.transform.translation.x
    drone_position_offset_local.translation.y = -tf.transform.translation.y
    drone_position_offset_local.translation.z = -tf.transform.translation.z
    drone_position_offset_local.rotation.x = 0
    drone_position_offset_local.rotation.y = 0
    drone_position_offset_local.rotation.z = 0
    drone_position_offset_local.rotation.w = 1

    resp = droneTrajectoryResponse()
    resp.drone_trajectory = MultiDOFJointTrajectory()
    resp.drone_trajectory.header.frame_id = "map"
    resp.drone_trajectory.header.stamp = rospy.Time.now()

    for i in range(len(tip_traj.points)):
        curr_tip_transform = Transform()
        curr_tip_transform = tip_traj.points[i].transforms[0]

        yaw_transform = Transform()
        #extract the yaw angle from the rotation setpoint
        (roll, pitch, yaw) = euler_from_quaternion([curr_tip_transform.rotation.x,
                                                    curr_tip_transform.rotation.y,
                                                    curr_tip_transform.rotation.z,
                                                    curr_tip_transform.rotation.w])
        yaw_q = quaternion_from_euler(0.0, 0.0, yaw)
        yaw_transform.rotation.x = yaw_q[0]
        yaw_transform.rotation.y = yaw_q[1]
        yaw_transform.rotation.z = yaw_q[2]
        yaw_transform.rotation.w = yaw_q[3]

        drone_position_offset_world = do_transform_transform(drone_position_offset_local, yaw_transform)

        drone_transform = do_transform_transform(curr_tip_transform, drone_position_offset_world)

        drone_trajectory_point = MultiDOFJointTrajectoryPoint()
        drone_trajectory_point.time_from_start = tip_traj.points[i].time_from_start
        drone_trajectory_point.transforms.append(drone_transform)
        drone_trajectory_point.velocities.append(tip_traj.points[i].velocities[0])
        drone_trajectory_point.accelerations.append(tip_traj.points[i].accelerations[0])
        resp.drone_trajectory.points.append(drone_trajectory_point)

    return resp

def handle_transform_trajectory(req):
    # service takes a desired toolpath trajectory (probably generated by TOPPRA service) and transforms 
    # to the printing surface in the drone's frame of reference.

    frame_id_init = req.frame_id
    frame_id_new = req.transformed_frame_id
    traj = req.toolpath_trajectory
    
    rospy.loginfo("Trajectory transform requested from frame_id=" + frame_id_init + " to frame_id=" + frame_id_new + ".")
    
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    tf = tfBuffer.lookup_transform(frame_id_init, frame_id_new, time=rospy.Time.now(), timeout=rospy.Duration(5))

    resp = transformTrajectoryResponse()

    resp.toolpath_trajectory = MultiDOFJointTrajectory()
    resp.toolpath_trajectory.header.frame_id = frame_id_new
    resp.toolpath_trajectory.header.stamp = rospy.Time.now()

    for i in range(len(traj.points)):
        curr_transform = Transform()
        curr_transform = traj.points[i].transforms[0]
        curr_vel = Twist()
        curr_vel = traj.points[i].velocities[0]
        curr_accel = Twist()
        curr_accel = traj.points[i].accelerations[0]

        curr_transform_transformed = do_transform_transform(curr_transform, tf)
        curr_vel_transformed = do_transform_twist(curr_vel, tf)
        curr_accel_transformed = do_transform_twist(curr_accel, tf)

        transformed_toolpath_point = MultiDOFJointTrajectoryPoint()
        transformed_toolpath_point.time_from_start = traj.points[i].time_from_start
        transformed_toolpath_point.transforms.append(curr_transform_transformed)
        transformed_toolpath_point.velocities.append(curr_vel_transformed)
        transformed_toolpath_point.accelerations.append(curr_accel_transformed)
        resp.toolpath_trajectory.points.append(transformed_toolpath_point)

    rospy.loginfo("Trajectory ready")
    return resp
    

def handle_TOPPRA_trajectory(req):
    # service generates a smooth, interpolated, time-optimal trajectory from an array of poses using TOPPRA

    num_poses = len(req.poses.poses)
    
    rospy.loginfo("Trajectory requested. Interpolating " + str(num_poses) + " poses at " + str(req.frequency) + "Hz.")

    way_pts = np.zeros((num_poses, 6))
    for i in range(num_poses):
        (roll, pitch, yaw) = euler_from_quaternion([req.poses.poses[i].orientation.x,
                                                    req.poses.poses[i].orientation.y,
                                                    req.poses.poses[i].orientation.z,
                                                    req.poses.poses[i].orientation.w])
        way_pts[i,:] = [req.poses.poses[i].position.x, req.poses.poses[i].position.y, req.poses.poses[i].position.z, roll, pitch, yaw]
    
    ss = np.linspace(0, 1, num_poses)
    
    amax = req.max_acc
    vmax = req.max_vel
    max_yawrate = np.deg2rad(req.max_yawrate)
    max_yawrate_dot = np.deg2rad(req.max_yawrate_dot)
    vlims = [vmax, vmax, vmax, max_yawrate, max_yawrate, max_yawrate]
    alims = [amax, amax, amax, max_yawrate_dot, max_yawrate_dot, max_yawrate_dot]
    
    path = ta.SplineInterpolator(ss, way_pts)
    pc_vel = constraint.JointVelocityConstraint(vlims)
    pc_acc = constraint.JointAccelerationConstraint(alims)
    instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
    traj = instance.compute_trajectory(0, 0)

    n_samples = int(traj.duration * req.frequency)
    ts_sample = np.linspace(0, traj.duration, n_samples)
    qs_sample = traj(ts_sample, 0) #position setpoints
    qds_sample = traj(ts_sample, 1) #velocity setpoints
    qdds_sample = traj(ts_sample, 2) #acceleration setpoints

    resp = TOPPRATrajectoryResponse()
    resp.trajectory = MultiDOFJointTrajectory()
    resp.trajectory.header.frame_id = "printing_plane"
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


if __name__ == '__main__':
    # initialize node
    rospy.init_node('print_manager', anonymous=True)
    pM = printManager()
    rospy.spin()
