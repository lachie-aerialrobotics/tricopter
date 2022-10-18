#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float32, String, Header
from controller_msgs.msg import FlatTarget
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandTOL
from tf.transformations import euler_from_quaternion
from tricopter.srv import *
from transitions import Machine
from trajectory_handler import *

class printStateMachine(object):
    states = ['Idle', 'Takeoff', 'Landing', 'Hover', 'MoveToPrint', 'MoveToPad', 'Print', 'Manual']

    transitions = [
        {'trigger': 'startTakeoff',     'source': 'Idle',           'dest': 'Takeoff',      'before':   'on_Takeoff'},
        {'trigger': 'finishTakeoff',    'source': 'Takeoff',        'dest': 'Hover',        'after':    'on_Hover'},
        {'trigger': 'goToPrint',        'source': 'Hover',          'dest': 'MoveToPrint',  'before':   'on_goToPrint'},
        {'trigger': 'arriveAtPrint',    'source': 'MoveToPrint',    'dest': 'Print'},
        {'trigger': 'returnToPad',      'source': 'Print',          'dest': 'MoveToPad',    'before':   'on_return'},
        {'trigger': 'arriveAtPad',      'source': 'MoveToPad',      'dest': 'Hover'},
        {'trigger': 'startLanding',     'source': 'Hover',          'dest': 'Landing',      'before':   'on_Landing'},
        {'trigger': 'finishLanding',    'source': 'Landing',        'dest': 'Idle'}
    ]

    def __init__(self):
        self.rate = 30
        self.tol_speed = 0.5
        self.takeoff_hgt = 0.5
        self.max_vel = 0.5
        self.max_acc = 0.5
        self.max_yawrate = 40
        self.max_yawrate_dot = 10
        
        # initial values
        self.target = FlatTarget()
        self.yaw = 0.0
        self.layer = 0
        self.tooltip_pose = PoseStamped()
        self.tooltip_twist = TwistStamped()

        # initiate state machine model with states and transitions listed above
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial = 'Idle')

        # initiate trajectoryHandler object instance - used to store and generate new trajectories #todo: maybe split transition handler and print handler?
        self.tH = trajectoryHandler(
            frequency=self.rate, max_vel=self.max_vel, max_acc=self.max_acc, 
            max_yawrate=self.max_yawrate, max_yawrate_dot=self.max_yawrate_dot)

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

        # drone state subscriber
        state_sub = rospy.Subscriber(
            '/mavros/state', State, self._state_cb, queue_size=1, tcp_nodelay=True)
        ext_state_sub = rospy.Subscriber(
            '/mavros/extended_state', ExtendedState, self._ext_state_cb, queue_size=1, tcp_nodelay=True)
        local_position_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self._local_pos_cb, queue_size=1, tcp_nodelay=True)

        # wait for drone to come online
        rospy.wait_for_message('/mavros/state', State)
        rospy.wait_for_message('/mavros/extended_state', ExtendedState)
        rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)

        # timer callback to send setpoints at a reasonable rate    
        sp_timer = rospy.Timer(rospy.Duration(1.0/self.rate), self._timer_cb, reset=True)

    #----------------------------------------------------------------------------------------------
    #callbacks on state transitions
    def on_Takeoff(self):
        rospy.loginfo("Takeoff initiated")
        self.pad_pose = PoseStamped()
        self.pad_pose = self.local_pose
        self.pad_pose.pose.position.z = self.takeoff_hgt

    def on_Hover(self):              
        rospy.loginfo("generating trajectory for next layer")
        self.tH.generate_print_layer(self.layer)    
        self.goToPrint()
    
    def on_Landing(self):
        rospy.loginfo("Landing initiated")
          
    def on_return(self):
        rospy.loginfo("generating trajectory to home")
        self.tH.generate_transition(self.local_pose, self.pad_pose)  

    def on_goToPrint(self): 
        rospy.loginfo("generating trajectory to beginning of print")    
        self.tH.generate_transition(self.local_pose, self.tH.get_print_start_pose())

    #----------------------------------------------------------------------------------------------
    # callbacks to occur on timer event
    def during_Hover(self):
        self.tooltip_state = "STAB_3DOF"
        pass

    def during_Takeoff(self):
        self.tooltip_state = "STAB_3DOF"
        #increase target z to deined loiter height
        if self.target.position.z < self.takeoff_hgt:
            self.target.position.z += self.tol_speed / self.rate
        else: #when target has reached loiter height and drone knows its flying, move to next state 
            self.target.position.z = self.takeoff_hgt
            if self.mavros_ext_state.landed_state == 2:
                self.finishTakeoff()

    def during_Idle(self):
        self.tooltip_state = "RETRACTED"
        #set target position to current landed position to prevent big jumps
        header = Header(stamp=rospy.Time.now(), frame_id="map")
        self.target, self.yaw = flat_target_msg_conversion(4, self.local_pose)
        #wait for arming and offboard then takeoff
        if (self.mavros_state.mode == "OFFBOARD") and self.mavros_state.armed:
            self.startTakeoff()

    def during_MoveToPrint(self):
        self.tooltip_state = "STAB_3DOF"
        pose, velocity, acceleration, complete = self.tH.follow_transition_trajectory()
        self.target, self.yaw = flat_target_msg_conversion(2, pose, velocity, acceleration)
        if complete:
            self.arriveAtPrint()

    def during_MoveToPad(self):
        self.tooltip_state = "STAB_3DOF"
        pose, velocity, acceleration, complete = self.tH.follow_transition_trajectory()
        self.target, self.yaw = flat_target_msg_conversion(2, pose, velocity, acceleration)
        if complete:
            self.arriveAtPad()

    def during_Print(self):
        self.tooltip_state = "STAB_6DOF"
        pose, velocity, acceleration, tooltip_pose, tooltip_twist, tooltip_accel, complete = self.tH.follow_print_trajectory()
        self.target, self.yaw = flat_target_msg_conversion(2, pose, velocity, acceleration)
        
        if complete:
            self.returnToPad()

    def during_Landing(self):
        self.tooltip_state = "STAB_3DOF"
        if self.target.position.z > 0 and not self.mavros_ext_state.landed_state == 1:
            self.target.position.z += -self.tol_speed / self.rate
        else:
            rospy.wait_for_service('/mavros/cmd/land')
            try:
                l = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
                l(altitude = 0)
            except rospy.ServiceException:
                rospy.loginfo("Landing service call failed") 
            self.finishLanding()

    #----------------------------------------------------------------------------------------------
    #ros callback functions
    def _timer_cb(self, event):
        exec("self.during_" + str(self.state) + "()")

        self.geo_pose_pub.publish(self.target)
        self.geo_yaw_pub.publish(Float32(self.yaw))
        self.pub_tooltip_state.publish(String(self.tooltip_state))
        self.pub_tooltip_pose.publish(self.tooltip_pose)
        self.pub_tooltip_twist.publish(self.tooltip_twist)

    def _state_cb(self, state_msg):
        self.mavros_state = state_msg

    def _ext_state_cb(self, ext_state_msg):
        self.mavros_ext_state = ext_state_msg

    def _local_pos_cb(self, local_pose_msg):
        self.local_pose = local_pose_msg


def flat_target_msg_conversion(mask, pose=PoseStamped(), velocity=TwistStamped(), acceleration=TwistStamped()):
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

    
if __name__ == '__main__':
    # initialize node
    rospy.init_node('print_state_machine', anonymous=True)
    pSM = printStateMachine()
    rospy.spin()
