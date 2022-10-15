#! /usr/bin/env python3

from json import tool
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from std_msgs.msg import Float32, String
from controller_msgs.msg import FlatTarget
from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from nav_msgs.msg import Path
from mavros_msgs.srv import CommandTOL
from trajectory_msgs.msg import MultiDOFJointTrajectory
from tf.transformations import euler_from_quaternion
from tricopter.srv import *

import statemachine as st

import getch

class droneStateMachine(st.StateMachine):
    #define all drone states
    idle = st.State("idle", initial=True)
    takeoff = st.State("takeoff")
    landing = st.State("landing")
    hold = st.State("hold")

    #define state transitions
    startTakeoff = idle.to(takeoff)
    finishTakeoff = takeoff.to(hold)
    startLanding = hold.to(landing)
    finishLanding = landing.to(idle)

    #callbacks on state transitions
    def on_startTakeoff(self):
        rospy.loginfo("Takeoff initiated")
        
    def on_enter_hold(self):
        rospy.loginfo("Holding position - would you like to:")
        rospy.loginfo("(l)and?")
        rospy.loginfo("(p)rint a new layer?")
        command = getch.getch() 
        if command == "l" or command == "L":
            droneState.startLanding()
        if command == "p" or command == "P":
            rospy.loginfo("generating trajectory for next layer")
            pM.toolpath, pM.dronepath = generate_layer_trajectory(0)
        else:
            rospy.loginfo("Invalid input: " + command)

    def on_enter_landing(self):
        rospy.loginfo("Landing initiated")
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            l = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            l(altitude = 0)
        except rospy.ServiceException:
            rospy.loginfo("Landing service call failed")

    def on_enter_idle(self):
        rospy.loginfo("Ready to fly! Please arm and switch to offboard")
   
class printManager:
    def __init__(self):
        self.rate = 30
        self.tol_speed = 0.5
        self.takeoff_hgt = 0.5
        self.target = FlatTarget()

        self.dronepath = MultiDOFJointTrajectory()
        self.toolpath = MultiDOFJointTrajectory()

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
        self.pub_print_viz = rospy.Publisher('/viz/print', Path, queue_size=1)

        # drone state subscriber
        state_sub = rospy.Subscriber(
            '/mavros/state', State, self.state_cb, queue_size=1, tcp_nodelay=True)
        ext_state_sub = rospy.Subscriber(
            '/mavros/extended_state', ExtendedState, self.ext_state_cb, queue_size=1, tcp_nodelay=True)
        local_position_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self.local_pos_cb, queue_size=1, tcp_nodelay=True)

        # wait for drone to come online
        rospy.wait_for_message('/mavros/state', State)
        rospy.wait_for_message('/mavros/extended_state', ExtendedState)
        rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)

        # timer callback to send setpoints at a reasonable rate    
        sp_timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.sp_timer_cb, reset=True)

        #timer callback to update trajectory visualisation - #temp: to be replaced with callback in state machine
        viz_timer = rospy.Timer(rospy.Duration(2.0), self.viz_timer_cb, reset=True)

    def viz_timer_cb(self, event):
        publish_viz_print(self.pub_print_viz)
        publish_viz_trajectory(self.toolpath, self.pub_toolpath_viz)
        publish_viz_trajectory(self.dronepath, self.pub_dronepath_viz)
                    
    def sp_timer_cb(self, event):
        self.target.header.stamp = rospy.Time.now()

        if droneState.current_state == droneState.idle:
            self.pub_tooltip_state.publish(String("RETRACTED"))
            self.target.position = self.local_position
            self.yaw = self.local_yaw
            if (self.mavros_state.mode == "OFFBOARD") and self.mavros_state.armed:
                droneState.startTakeoff()

        elif droneState.current_state == droneState.takeoff:
            self.pub_tooltip_state.publish(String("HOME"))
            if self.target.position.z < self.takeoff_hgt:
                self.target.position.z += self.tol_speed / self.rate
            else:
                self.target.position.z = self.takeoff_hgt
                if self.mavros_ext_state.landed_state == 2:
                    droneState.finishTakeoff()

        elif droneState.current_state == droneState.hold:
            self.pub_tooltip_state.publish(String("STAB_3DOF"))
            

        elif droneState.current_state == droneState.landing:
            self.pub_tooltip_state.publish(String("HOME"))
            # if self.target.position.z >= -0.5 and not self.mavros_ext_state.landed_state == 1:
            #     self.target.position.z += -self.tol_speed / self.rate
            if self.mavros_ext_state.landed_state == 1:
                # self.target.position = self.local_position
                droneState.finishLanding()

        self.geo_pose_pub.publish(self.target)
        self.geo_yaw_pub.publish(Float32(self.yaw))

    def state_cb(self, state_msg):
        self.mavros_state = state_msg

    def ext_state_cb(self, ext_state_msg):
        self.mavros_ext_state = ext_state_msg

    def local_pos_cb(self, local_pose_msg):
        self.local_pose = local_pose_msg.pose
        self.local_position = local_pose_msg.pose.position
        (roll, pitch, yaw) = euler_from_quaternion([local_pose_msg.pose.orientation.x,
                                                    local_pose_msg.pose.orientation.y,
                                                    local_pose_msg.pose.orientation.z,
                                                    local_pose_msg.pose.orientation.w])
        self.local_yaw = yaw

def generate_layer_trajectory(layer_number):
    # get poses from file
    rospy.wait_for_service('fetch_poses')
    get_poses = rospy.ServiceProxy('fetch_poses', fetchPoses)
    request = fetchPosesRequest()
    request.prefix = "waypoints"
    request.frame_id = "printing_plane"
    request.layer_number = layer_number
    response = get_poses(request)

    #transform to world coordinates system
    rospy.wait_for_service('get_transformed_trajectory')
    transform_poses = rospy.ServiceProxy('get_transformed_trajectory', transformTrajectory)
    request = transformTrajectoryRequest()
    request.poses = response.poses
    request.frame_id = "printing_plane"
    request.transformed_frame_id = "map"
    response = transform_poses(request)

    #interpolate with TOPPRA
    rospy.wait_for_service('get_TOPPRA_trajectory')
    get_traj = rospy.ServiceProxy('get_TOPPRA_trajectory', TOPPRATrajectory)
    request = TOPPRATrajectoryRequest()
    request.frequency = 30
    request.max_vel = 1
    request.max_acc = 0.5
    request.max_yawrate = 90
    request.max_yawrate_dot = 180
    request.poses = response.poses_transformed
    response = get_traj(request)

    tooltip_trajectory = response.trajectory

    #get offset drone trajectory
    rospy.wait_for_service('get_drone_trajectory')
    get_traj = rospy.ServiceProxy('get_drone_trajectory', droneTrajectory)
    request = droneTrajectoryRequest()
    request.drone_body_frame_id = "base_link"
    request.tooltip_frame_id = "tooltip_init"
    request.toolpath_trajectory = response.trajectory
    response = get_traj(request)

    drone_trajectory = response.drone_trajectory

    return tooltip_trajectory, drone_trajectory

def publish_viz_trajectory(trajectory, publisher):
    # function to convert a MultiDOFJointTrajectory message to a Path message for visualisation in rviz
    if not (isinstance(trajectory, MultiDOFJointTrajectory) and isinstance(publisher, rospy.Publisher)):
        rospy.logerr("Incorrect input types")
    else:
        viz_path = Path()
        viz_path.header.frame_id = trajectory.header.frame_id
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

def publish_viz_poses(pose_array, publisher):
    # function to convert a PoseArray message to a Path message for visualisation in rviz
    if not (isinstance(pose_array, PoseArray) and isinstance(publisher, rospy.Publisher)):
        rospy.logerr("Incorrect input types")
    else:
        viz_path = Path()
        viz_path.header.frame_id = pose_array.header.frame_id
        viz_path.header.stamp = rospy.Time.now()
        for i in range(len(pose_array.poses)):
            pose = PoseStamped()
            pose.header.frame_id = viz_path.header.frame_id
            pose.header.stamp = viz_path.header.stamp            
            pose.pose.position.x = pose_array.poses[i].position.x
            pose.pose.position.y = pose_array.poses[i].position.y
            pose.pose.position.z = pose_array.poses[i].position.z
            pose.pose.orientation.x = pose_array.poses[i].orientation.x
            pose.pose.orientation.y = pose_array.poses[i].orientation.y
            pose.pose.orientation.z = pose_array.poses[i].orientation.z
            pose.pose.orientation.w = pose_array.poses[i].orientation.w
            viz_path.poses.append(pose)
        publisher.publish(viz_path)

def publish_viz_print(publisher):
    # function to visualise all layers in a print
    rospy.wait_for_service('fetch_poses')
    get_poses = rospy.ServiceProxy('fetch_poses', fetchPoses)
    request = fetchPosesRequest()
    request.prefix = "waypoints"
    pose_array = PoseArray()
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = "map"
    for i in range(rospy.get_param("/"+str(request.prefix)+"/n_layers")):
        request.layer_number = i
        response = get_poses(request)
        for j in range(len(response.poses.poses)):
            pose_array.poses.append(response.poses.poses[j])
    publish_viz_poses(pose_array, publisher)





if __name__ == '__main__':
    # initialize node
    rospy.init_node('print_manager', anonymous=True)
    pM = printManager()
    # create state machine instance
    droneState = droneStateMachine(start_value='idle')
    rospy.spin()
