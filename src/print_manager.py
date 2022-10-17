#! /usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from std_msgs.msg import Float32, String, Header
from controller_msgs.msg import FlatTarget
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from nav_msgs.msg import Path
from mavros_msgs.srv import CommandTOL
from trajectory_msgs.msg import MultiDOFJointTrajectory
from tf.transformations import euler_from_quaternion
from tricopter.srv import *

import statemachine as st

from trajectory_handler import *
from viz_functions import *
   
class printManager:
    def __init__(self):
        self.rate = 30
        self.tol_speed = 0.5
        self.takeoff_hgt = 0.5
        self.target = FlatTarget()

        self.dronepath = MultiDOFJointTrajectory()
        self.toolpath = MultiDOFJointTrajectory()
        self.transitionpath = MultiDOFJointTrajectory()

        # create state machine instance
        self.droneState = droneStateMachine(start_value='Idle')

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

        if self.droneState.current_state == self.droneState.Idle:
            #set target position to current landed position to prevent big jumps
            header = Header(stamp=rospy.Time.now(), frame_id="map")
            self.target, self.yaw = flat_target_msg_conversion(self.local_pose, TwistStamped(header=header), TwistStamped(header=header), 4)
            #wait for arming and offboard then takeoff
            if (self.mavros_state.mode == "OFFBOARD") and self.mavros_state.armed:
                self.droneState.startTakeoff()

        elif self.droneState.current_state == self.droneState.Takeoff:
            #increase target z to deined loiter height
            if self.target.position.z < self.takeoff_hgt:
                self.target.position.z += self.tol_speed / self.rate
            else: #when target has reached loiter height and drone knows its flying, move to next state 
                self.target.position.z = self.takeoff_hgt
                if self.mavros_ext_state.landed_state == 2:
                    self.droneState.finishTakeoff()

        elif self.droneState.current_state == self.droneState.Hover:
            pass

        elif self.droneState.current_state == self.droneState.Move:
            pose, velocity, acceleration, complete = tH.follow(self.transitionpath)
            self.target, self.yaw = flat_target_msg_conversion(pose, velocity, acceleration, 2)
            if complete:
                self.droneState.arriveAtPrint()

        elif self.droneState.current_state == self.droneState.Print:
            pose, velocity, acceleration, complete_drone = tH.follow(self.dronepath)
            self.target, self.yaw = flat_target_msg_conversion(pose, velocity, acceleration, 2)
            tooltip_pose, tooltip_twist, tooltip_accel, complete_tooltip = tH_tooltip.follow(self.toolpath)
            self.pub_tooltip_pose.publish(tooltip_pose)
            self.pub_tooltip_twist.publish(tooltip_twist)

            if complete_drone: #and complete_tooltip:
                self.droneState.returnToPad()
                
        elif self.droneState.current_state == self.droneState.Landing:
            # if self.target.position.z >= -0.5 and not self.mavros_ext_state.landed_state == 1:
            #     self.target.position.z += -self.tol_speed / self.rate
            if self.mavros_ext_state.landed_state == 1:
                # self.target.position = self.local_position
                self.droneState.finishLanding()

        self.geo_pose_pub.publish(self.target)
        self.geo_yaw_pub.publish(Float32(self.yaw))

        if self.droneState.current_state == self.droneState.Idle:
            tooltip_state = "RETRACTED"
        elif self.droneState.current_state == self.droneState.Hover:
            tooltip_state = "STAB_3DOF"
        elif self.droneState.current_state == self.droneState.Move:
            tooltip_state = "STAB_3DOF"
        elif self.droneState.current_state == self.droneState.Print:
            tooltip_state = "STAB_6DOF"
        else:
            tooltip_state = "HOME"
        
        self.pub_tooltip_state.publish(String(tooltip_state))

    def state_cb(self, state_msg):
        self.mavros_state = state_msg

    def ext_state_cb(self, ext_state_msg):
        self.mavros_ext_state = ext_state_msg

    def local_pos_cb(self, local_pose_msg):
        self.local_pose = local_pose_msg


def flat_target_msg_conversion(pose, velocity, acceleration, mask):
    # uint8 IGNORE_SNAP = 1	- Position Velocity Acceleration Jerk Reference
    # uint8 IGNORE_SNAP_JERK = 2	- Position Velocity Acceleration Reference
    # uint8 IGNORE_SNAP_JERK_ACC = 4	- Position Reference
    target = FlatTarget()
    target.header = pose.header
    target.type_mask = mask
    target.position = pose.pose.position
    target.velocity = velocity.twist.linear
    target.acceleration = acceleration.twist.linear
    (roll, pitch, yaw) = euler_from_quaternion([pose.pose.orientation.x,
                                                pose.pose.orientation.y,
                                                pose.pose.orientation.z,
                                                pose.pose.orientation.w])                                           
    return target, yaw

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

    #callbacks on entering states
    def on_enter_takeoff(self):
        self.pad_pose = pM.local_pose
        self.pad_pose.pose.position.z = pM.takeoff_hgt

    def on_enter_Hover(self):              
        rospy.loginfo("generating trajectory for next layer")
        pM.dronepath, pM.toolpath = tH.generate_print_layer(self.layer)
        
        rospy.loginfo("generating trajectory to beginning of print")    
        pM.transitionpath = tH.generate_transition(pM.local_pose, tH.get_print_start_pose(pM.dronepath))
        self.goToPrint()
    
    def on_enter_Landing(self):
        rospy.loginfo("Landing initiated")
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            l = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            l(altitude = 0)
        except rospy.ServiceException:
            rospy.loginfo("Landing service call failed")   

    # callbacks on state transitions
    def on_returnToPad(self):
        rospy.loginfo("generating trajectory to home")
        pM.transitionpath = tH.generate_transition(pM.local_pose, self.pad_pose)   

    
if __name__ == '__main__':
    # initialize node
    rospy.init_node('print_manager', anonymous=True)
    pM = printManager()
    tH = trajectoryHandler()
    tH.frequency = 30
    tH.max_vel = 0.5
    tH.max_acc = 0.5
    tH.max_yawrate = 40
    tH.max_yawrate_dot = 10
    tH_tooltip = trajectoryHandler()
    tH_tooltip.frequency = 30
    tH_tooltip.max_vel = 0.5
    tH_tooltip.max_acc = 0.5
    tH_tooltip.max_yawrate = 40
    tH_tooltip.max_yawrate_dot = 10
    rospy.spin()
