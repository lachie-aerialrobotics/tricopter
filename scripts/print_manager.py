#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float32, String
from controller_msgs.msg import FlatTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from tricopter.srv import *
from transitions import Machine
from trajectory_handler import *
from misc_functions import *

class printStateMachine(object):
    states = ['Takeoff', 'Landing', 'Home', 'Move', 'Print', 'TolCheck', 'Manual']

    transitions = [
        {'trigger': 'startTakeoff',     'source': 'Manual',                         'dest': 'Takeoff',  'after':   'on_Takeoff'      },
        {'trigger': 'arriveAtPrint',    'source': 'Move',                           'dest': 'TolCheck', 'after':   'on_arriveAtPrint'},
        {'trigger': 'startPrint',       'source': 'TolCheck',                       'dest': 'Print',    'before':  'on_startPrint'   },
        {'trigger': 'arriveAtHome',     'source': 'Move',                           'dest': 'Home',     'after':   'on_arriveAtHome' },
        {'trigger': 'goToHome',         'source': ['Takeoff', 'Print', 'Manual'],   'dest': 'Move',     'before':  'on_goToHome'     },
        {'trigger': 'goToPrint',        'source': 'Home',                           'dest': 'Move',     'before':  'on_goToPrint'    },
        {'trigger': 'goToPad',          'source': 'Home',                           'dest': 'Move',     'before':  'on_goToPad'      },
        {'trigger': 'startLanding',     'source': 'Move',                           'dest': 'Landing',  'after':   'on_Landing'      },
        {'trigger': 'finishLanding',    'source': 'Landing',                        'dest': 'Manual'                                 },
        {'trigger': 'manualTakeover',   'source': '*',                              'dest': 'Manual',   'before':   'on_Takeover'    }     
        ]
    
    def __init__(self):
        # get config parameters from parameter server
        self.rate = rospy.get_param('/print_planner/setpoint_rate')
        self.tol_speed = rospy.get_param('/print_planner/tol_speed')
        self.takeoff_hgt = rospy.get_param('/print_planner/tol_height')   
        self.max_vel = rospy.get_param('/print_planner/print_vel')
        self.max_acc = rospy.get_param('/print_planner/max_accel')
        self.max_yawrate = rospy.get_param('/print_planner/max_yawrate')
        self.max_yawrate_dot = rospy.get_param('/print_planner/max_yawrate_dot')       
        self.vel_tol = rospy.get_param('/print_planner/start_speed_tolerance')
        self.pos_tol = rospy.get_param('/print_planner/start_position_tolerance')       
        self.layer = rospy.get_param('/print_planner/first_layer')

        # initial values
        self.target = FlatTarget()
        self.target.header.frame_id = "map"
        self.yaw = 0.0
        self.tooltip_state = "RETRACTED"
        self.tooltip_pose = PoseStamped()
        self.tooltip_pose.header.frame_id = "map"
        self.tooltip_twist = TwistStamped()
        self.tooltip_twist.header.frame_id = "map"

        # initiate state machine model with states and transitions listed above
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial = 'Manual')

        # initiate trajectoryHandler object instance - used to store and generate new trajectories 
        # #todo: maybe split transition handler and print handler?
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
        local_velocity_sub = rospy.Subscriber(
            '/mavros/local_position/velocity_body', TwistStamped, self._local_vel_cb, queue_size=1, tcp_nodelay=True)

        # wait for drone to come online
        rospy.wait_for_message('/mavros/state', State)
        rospy.wait_for_message('/mavros/extended_state', ExtendedState)
        rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        
        # timer callback to send setpoints at a reasonable rate    
        sp_timer = rospy.Timer(rospy.Duration(1.0/self.rate), self._timer_cb, reset=True)

        # initiate landing position at location where node is started
        self.pad_pose = PoseStamped()
        self.pad_pose = self.local_pose
        self.pad_pose.pose.position.z = self.takeoff_hgt
        rospy.loginfo("Landing site initiated at x=" + str(self.pad_pose.pose.position.x) +
            ", y=" + str(self.pad_pose.pose.position.y) + ".")

        # determine loiter point above print
        self.home_pose = self.tH.get_loiter_point()

    #--------------------------------------------------------------------------------------------------------------
    #callbacks on state transitions

    def on_Takeoff(self):
        rospy.loginfo("Takeoff initiated")

    def on_arriveAtHome(self):
        if self.layer < rospy.get_param(str(self.tH.waypoint_prefix) + '/n_layers'):
            rospy.loginfo("Generating trajectory for next layer")
            self.tH.generate_print_layer(self.layer)    
            self.goToPrint()
        else:
            self.goToPad()
    
    def on_Landing(self):
        rospy.loginfo("Landing initiated")
   
    def on_goToPad(self):
        rospy.loginfo("Generating trajectory to pad")
        self.tH.generate_transition(self.local_pose, self.pad_pose)  
        self.moveCompletionTransition = self.startLanding

    def on_goToPrint(self): 
        rospy.loginfo("Generating trajectory to beginning of print")  
        self.print_start_pose = self.tH.get_print_start_pose()
        self.tH.generate_transition(self.local_pose, self.print_start_pose)
        self.moveCompletionTransition = self.arriveAtPrint

    def on_startPrint(self):
        rospy.loginfo("Printing...")

    def on_goToHome(self):
        rospy.loginfo("Moving to home position")
        self.tH.generate_transition(self.local_pose, self.home_pose)
        self.moveCompletionTransition = self.arriveAtHome

    def on_arriveAtPrint(self):
        rospy.loginfo("Arrived at print start - waiting for velocity/position tolerances")

    def on_Takeover(self):
        rospy.loginfo("Manual takeover")

    #---------------------------------------------------------------------------------------------------------------
    # callbacks to occur on timer event - need to be defined for every state that is called

    def during_Home(self):
        self.tooltip_state = "STAB_3DOF"
        self.target, self.yaw = flat_target_msg_conversion(4, self.home_pose)

    def during_Takeoff(self):
        self.tooltip_state = "HOME"
        #increase target z to deined loiter height
        if self.target.position.z < self.takeoff_hgt:
            self.target.position.z += self.tol_speed / self.rate
        else: #when target has reached loiter height and drone knows its flying, move to next state 
            self.target.position.z = self.takeoff_hgt
            if self.mavros_ext_state.landed_state == 2:
                self.goToHome()

    def during_Move(self):
        self.tooltip_state = "STAB_3DOF"
        pose, velocity, acceleration, complete = self.tH.follow_transition_trajectory()
        self.target, self.yaw = flat_target_msg_conversion(2, pose, velocity, acceleration)
        if complete:
            self.moveCompletionTransition()

    def during_TolCheck(self):
        self.tooltip_state = "STAB_3DOF"
        self.target, self.yaw = flat_target_msg_conversion(4, self.print_start_pose)
        tolerances_satisfied = tolerance_checker(self.local_pose, self.local_velocity, self.print_start_pose, self.pos_tol, self.vel_tol)
        if tolerances_satisfied:
            self.startPrint()

    def during_Print(self):
        self.tooltip_state = "STAB_6DOF"
        pose, velocity, acceleration, self.tooltip_pose, self.tooltip_twist, self.tooltip_accel, complete = self.tH.follow_print_trajectory()
        self.target, self.yaw = flat_target_msg_conversion(2, pose, velocity, acceleration)
        if complete:
            self.layer += 1
            self.goToHome()

    def during_Landing(self):
        self.tooltip_state = "HOME"
        #reduce height of z setpoint until altitude is zero
        if self.target.position.z > 0 and not (self.mavros_ext_state.landed_state == 1):
            self.target.position.z += -self.tol_speed / self.rate
        else:
        #make double sure the drone is on the ground by calling the mavros landing service
            call_landing_service()
            self.finishLanding()

    def during_Manual(self):
        #if landed -> takeoff. If flying -> hold position
        self.tooltip_state = "HOME"
        if not self.mavros_state.armed:
            self.tooltip_state = "RETRACTED"
        elif self.mavros_state.mode == "OFFBOARD":
            self.tooltip_state = "HOME"
            if self.mavros_ext_state.landed_state == 2:
                self.goToHome()
            if self.mavros_ext_state.landed_state == 1:
                self.startTakeoff()
        self.target, self.yaw = flat_target_msg_conversion(4, self.local_pose)
    
    def during_always(self): #this callback always runs to check if not in offboard mode
        if self.mavros_state.mode != "OFFBOARD" and self.state != 'Manual':
            self.manualTakeover()

    #----------------------------------------------------------------------------------------------
    #ros callbacks

    def _timer_cb(self, event): #timer callback runs at specified rate to output setpoints
        # print(self.state)

        self.during_always()

        exec("self.during_" + str(self.state) + "()") #execute the function name corresponding to the current state

        # update time stamps and publish current values of drone and manipulator commands
        self.target.header.stamp = rospy.Time.now()
        self.tooltip_pose.header.stamp = rospy.Time.now()
        self.tooltip_twist.header.stamp = rospy.Time.now()
        self.geo_pose_pub.publish(self.target)
        self.geo_yaw_pub.publish(Float32(self.yaw))
        self.pub_tooltip_state.publish(String(self.tooltip_state))
        self.pub_tooltip_pose.publish(self.tooltip_pose)
        self.pub_tooltip_twist.publish(self.tooltip_twist)

    #callbacks save topics to current object for use later
    def _state_cb(self, state_msg):
        self.mavros_state = state_msg

    def _ext_state_cb(self, ext_state_msg):
        self.mavros_ext_state = ext_state_msg

    def _local_pos_cb(self, local_pose_msg):
        self.local_pose = local_pose_msg

    def _local_vel_cb(self, local_vel_msg):
        self.local_velocity = local_vel_msg

    
if __name__ == '__main__':
    # initialize node
    rospy.init_node('print_state_machine', anonymous=True)
    pSM = printStateMachine()
    rospy.spin()