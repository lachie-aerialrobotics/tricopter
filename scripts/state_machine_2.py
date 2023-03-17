#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float32, String
from controller_msgs.msg import FlatTarget
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from tricopter.srv import *
from transitions import Machine
from trajectory_handler import *
from misc_functions import *


class printStateMachine(object):
    states = ['Takeoff', 'Landing', 'Home', 'Move', 'Print', 'TolCheck', 'Ground', 'Manual']

    transitions = [
        {'trigger': 'startTakeoff',     'source': ['Ground', 'Manual'],             'dest': 'Takeoff',  'after':   'on_startTakeoff'  },
        {'trigger': 'arriveAtPrint',    'source': 'Move',                           'dest': 'TolCheck', 'after':   'on_arriveAtPrint' },
        {'trigger': 'startPrint',       'source': 'TolCheck',                       'dest': 'Print',    'before':  'on_startPrint'    },
        {'trigger': 'arriveAtHome',     'source': 'Move',                           'dest': 'Home',     'after':   'on_arriveAtHome'  },
        {'trigger': 'goToHome',         'source': ['Takeoff', 'Print', 'Manual'],   'dest': 'Move',     'before':  'on_goToHome'      },
        {'trigger': 'goToPrint',        'source': 'Home',                           'dest': 'Move',     'before':  'on_goToPrint'     },
        {'trigger': 'goToPad',          'source': 'Home',                           'dest': 'Move',     'before':  'on_goToPad'       },
        {'trigger': 'startLanding',     'source': 'Move',                           'dest': 'Landing',  'after':   'on_startLanding'  },
        {'trigger': 'manualTakeover',   'source': '*',                              'dest': 'Manual',   'before':  'on_manualTakeover'},
        {'trigger': 'switchToGround',   'source': ['Manual', 'Landing'],            'dest': 'Ground'                                  }     
        ]
    
    def __init__(self):
        # initiate state machine model with states and transitions listed above
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial = 'Ground')

        # publish current state
        self.pub_tooltip_state = rospy.Publisher('/print_state_machine/state',  String, queue_size=1)

        state_machine_service = rospy.Service('state_transition', stateMachine, self.handle_state_transition)
        

    def handle_state_transition(self, req):
        req.current_state
        req.target_state

    
    
if __name__ == '__main__':
    # initialize node
    rospy.init_node('print_state_machine', anonymous=True)
    pSM = printStateMachine()
    rospy.spin()