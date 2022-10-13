#! /usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped

class Setpoint:
    def __init__(self):   
        #retrieve params from parameter server
        self.theta = 0.0

        #Publish target positions for drone and manipulator tooltip
        self.pos_sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.vel_sp_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1, tcp_nodelay=True)
        self.acc_sp_pub = rospy.Publisher('/mavros/setpoint_accel/accel', Vector3Stamped, queue_size=1, tcp_nodelay=True)

        #Calculate setpoints
        rospy.Timer(rospy.Duration(1.0/rate), self.setpoint_callback, reset=True)

    def setpoint_callback(self, event):

        pos_sp = PoseStamped()
        vel_sp = TwistStamped()
        acc_sp = Vector3Stamped()
        
        pos_sp.pose.position.x = r * np.sin(self.theta)
        pos_sp.pose.position.y = r * np.cos(self.theta)
        pos_sp.pose.position.z = h

        vel_sp.twist.linear.x = v * np.cos(self.theta)
        vel_sp.twist.linear.y = v * -np.sin(self.theta)
        vel_sp.twist.linear.z = 0.0

        acc_sp.vector.x = v**2 / r * -np.cos(self.theta)
        acc_sp.vector.y = v**2 / r * -np.sin(self.theta)
        acc_sp.vector.z = 0.0

        self.theta += dtheta

        pos_sp.header.frame_id = "map"
        pos_sp.header.stamp = rospy.Time.now()

        vel_sp.header.frame_id = "map"
        vel_sp.header.stamp = rospy.Time.now()

        acc_sp.header.frame_id = "map"
        acc_sp.header.stamp = rospy.Time.now()

        self.pos_sp_pub.publish(pos_sp)
        self.vel_sp_pub.publish(vel_sp)
        # self.acc_sp_pub.publish(acc_sp)
        
if __name__ == '__main__': #initialise node
    rate = 30
    r = 1.0
    v = 1.8
    h = 2.0
    dtheta = v / (r * rate)
    rospy.init_node('joystick_node', anonymous=True)
    s = Setpoint()
    rospy.spin()