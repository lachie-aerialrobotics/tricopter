#! /usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from std_msgs.msg import Float32, String
from controller_msgs.msg import FlatTarget
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from nav_msgs.msg import Path
from mavros_msgs.srv import CommandTOL
from trajectory_msgs.msg import MultiDOFJointTrajectory
from tf.transformations import euler_from_quaternion
from tricopter.srv import *

import statemachine as st

from viz_functions import *

class droneStateMachine(st.StateMachine):
    #define all drone states
    Idle = st.State("Idle", initial=True)
    Takeoff = st.State("Takeoff")
    Landing = st.State("Landing")
    Hover = st.State("Hover")
    Move = st.State("Move")
    # HoldAtPrint = st.State("HoldAtPrint")
    Print = st.State("Print")

    #define state transitions
    startTakeoff = Idle.to(Takeoff)
    finishTakeoff = Takeoff.to(Hover) 
    goToPrint = Hover.to(Move)
    arriveAtPrint = Move.to(Print)
    returnToPad = Print.to(Move)
    arriveAtPad = Move.to(Hover)
    startLanding = Hover.to(Landing)
    finishLanding = Landing.to(Idle)

    layer = 0

    #callbacks on state transitions
    def on_enter_Hover(self):    
        self.pad_pose = pM.local_pose_stamped
        rospy.loginfo("generating trajectory for next layer")
        pM.toolpath, pM.dronepath = generate_layer_trajectory(self.layer)
        
        rospy.loginfo("generating trajectory to beginning of print")
        end_pose = PoseStamped()
        end_pose.header.stamp = rospy.Time.now()
        end_pose.header.frame_id = pM.dronepath.header.frame_id
        end_pose.pose.position = pM.dronepath.points[0].transforms[0].translation
        end_pose.pose.orientation = pM.dronepath.points[0].transforms[0].rotation
        start_pose = pM.local_pose_stamped
        pM.transitionpath = generate_transition_trajectory(start_pose, end_pose)
        self.goToPrint()

    def on_enter_Move(self):
        self.tH = trajectoryHandler()

    def on_enter_Print(self):
        self.tH_drone = trajectoryHandler()
        self.tH_tooltip = trajectoryHandler()
        
    def on_returnToPad(self):
        rospy.loginfo("generating trajectory to home")
        start_pose = pM.local_pose_stamped
        end_pose = self.pad_pose
        pM.transitionpath = generate_transition_trajectory(start_pose, end_pose)   

    def on_enter_Landing(self):
        rospy.loginfo("Landing initiated")
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            l = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            l(altitude = 0)
        except rospy.ServiceException:
            rospy.loginfo("Landing service call failed")
   
class printManager:
    def __init__(self):
        self.rate = 30
        self.tol_speed = 0.5
        self.takeoff_hgt = 0.5
        self.target = FlatTarget()

        self.dronepath = MultiDOFJointTrajectory()
        self.toolpath = MultiDOFJointTrajectory()
        self.transitionpath = MultiDOFJointTrajectory()

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
        self.pub_transitionpath_viz = rospy.Publisher('/viz/transitionpath', Path, queue_size=1)
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
        publish_viz_trajectory(self.transitionpath, self.pub_transitionpath_viz)
                    
    def sp_timer_cb(self, event):
        self.target.header.stamp = rospy.Time.now()

        if droneState.current_state == droneState.Idle:
            self.pub_tooltip_state.publish(String("RETRACTED"))
            #set target position to current landed position to prevent big jumps
            self.target.position = self.local_position
            self.yaw = self.local_yaw
            #wait for arming and offboard then takeoff
            if (self.mavros_state.mode == "OFFBOARD") and self.mavros_state.armed:
                droneState.startTakeoff()

        elif droneState.current_state == droneState.Takeoff:
            self.pub_tooltip_state.publish(String("HOME"))
            #increase target z to deined loiter height
            if self.target.position.z < self.takeoff_hgt:
                self.target.position.z += self.tol_speed / self.rate
            else: #when target has reached loiter height and drone knows its flying, move to next state 
                self.target.position.z = self.takeoff_hgt
                if self.mavros_ext_state.landed_state == 2:
                    droneState.finishTakeoff()

        elif droneState.current_state == droneState.Hover:
            self.pub_tooltip_state.publish(String("STAB_3DOF"))

        elif droneState.current_state == droneState.Move:
            self.pub_tooltip_state.publish(String("STAB_3DOF"))
            pose, velocity, acceleration, complete = droneState.tH.follow(self.transitionpath)
            self.target, self.yaw = flat_target_msg_conversion(pose, velocity, acceleration)
            if complete:
                droneState.arriveAtPrint()

        elif droneState.current_state == droneState.Print:
            self.pub_tooltip_state.publish(String("STAB_3DOF"))
            pose, velocity, acceleration, complete_drone = droneState.tH_drone.follow(self.dronepath)
            self.target, self.yaw = flat_target_msg_conversion(pose, velocity, acceleration)
            tooltip_pose, tooltip_twist, tooltip_accel, complete_tooltip = droneState.tH_tooltip.follow(self.toolpath)
            self.pub_tooltip_pose.publish(tooltip_pose)
            self.pub_tooltip_twist.publish(tooltip_twist)

            if complete_drone and complete_tooltip:
                droneState.returnToPad()
                
        elif droneState.current_state == droneState.Landing:
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
        self.local_pose_stamped = local_pose_msg
        self.local_pose = local_pose_msg.pose
        self.local_position = local_pose_msg.pose.position
        (roll, pitch, yaw) = euler_from_quaternion([local_pose_msg.pose.orientation.x,
                                                    local_pose_msg.pose.orientation.y,
                                                    local_pose_msg.pose.orientation.z,
                                                    local_pose_msg.pose.orientation.w])
        self.local_yaw = yaw

class trajectoryHandler:
    def __init__(self):
        self.point_count = 0
        self.complete = False
        
        self.pose = PoseStamped()
        self.velocity = TwistStamped()
        self.acceleration = TwistStamped()       

    def follow(self, trajectory):
        if self.point_count < len(trajectory.points):
            self.pose.header.stamp = rospy.Time.now()
            self.pose.header.frame_id = trajectory.header.frame_id
            self.velocity.header = self.pose.header
            self.acceleration.header = self.pose.header

            self.pose.pose.position = trajectory.points[self.point_count].transforms[0].translation
            self.pose.pose.orientation = trajectory.points[self.point_count].transforms[0].rotation
            self.velocity.twist = trajectory.points[self.point_count].velocities[0]
            self.acceleration.twist = trajectory.points[self.point_count].accelerations[0]
            self.point_count += 1
        else:
            self.complete = True

        return self.pose, self.velocity, self.acceleration, self.complete

def flat_target_msg_conversion(pose, velocity, acceleration):
    target = FlatTarget()
    target.header = pose.header
    target.type_mask = 2
    target.position = pose.pose.position
    target.velocity = velocity.twist.linear
    target.acceleration = acceleration.twist.linear
    (roll, pitch, yaw) = euler_from_quaternion([pose.pose.orientation.x,
                                                pose.pose.orientation.y,
                                                pose.pose.orientation.z,
                                                pose.pose.orientation.w])                                           
    return target, yaw


def generate_transition_trajectory(pose_start, pose_end):
    if pose_start.header.frame_id != pose_end.header.frame_id:
        rospy.logerr("Cannot interpolate between poses in different reference frames.")
    else:
        mid_pose_1 = Pose()
        mid_pose_1.position.x = pose_start.pose.position.x
        mid_pose_1.position.y = pose_start.pose.position.y
        mid_pose_1.position.z = pose_end.pose.position.z
        mid_pose_1.orientation = pose_start.pose.orientation

        poses = PoseArray()
        poses.header.stamp = rospy.Time.now()
        poses.header.frame_id = pose_start.header.frame_id
        poses.poses.append(pose_start.pose)
        poses.poses.append(mid_pose_1)
        poses.poses.append(pose_end.pose)

        rospy.wait_for_service('get_TOPPRA_trajectory')
        get_traj = rospy.ServiceProxy('get_TOPPRA_trajectory', TOPPRATrajectory)
        request = TOPPRATrajectoryRequest()
        request.frequency = 30
        request.max_vel = 2
        request.max_acc = 1
        request.max_yawrate = 30
        request.max_yawrate_dot = 30
        request.poses = poses
        response = get_traj(request)
        return response.trajectory


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

if __name__ == '__main__':
    # initialize node
    rospy.init_node('print_manager', anonymous=True)
    pM = printManager()
    # create state machine instance
    droneState = droneStateMachine(start_value='Idle')
    rospy.spin()
