#!/usr/bin/env python
from ast import Sub
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_conjugate
from message_filters import ApproximateTimeSynchronizer, Subscriber

#code to generate end-effector setpoints accounting for random drone perturbations
class velocityStabilisation:
    def __init__(self):
        #init tf listener
        self.tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(self.tfBuffer)

        #obtain transforms from platform to tooltip and drone base_link to platform_base
        PT_tf = self.tfBuffer.lookup_transform('platform', 'tooltip', time=rospy.Time(0), timeout=rospy.Duration(5))
        DB_tf = self.tfBuffer.lookup_transform('base_link', 'stewart_base', time=rospy.Time(0), timeout=rospy.Duration(5))

        #tooltip vector in platform frame, code assumes tooltip is not rotated relative to platform
        self.PT_p = np.asarray([PT_tf.transform.translation.x, PT_tf.transform.translation.y, PT_tf.transform.translation.z]) 
        #vector from base_link to stewart_base in drone coordinates
        self.DB_d = np.asarray([DB_tf.transform.translation.x, DB_tf.transform.translation.y, DB_tf.transform.translation.z]) 
        #rotation from drone frame to stewart_base frame
        self.d_q_b = np.asarray([DB_tf.transform.rotation.x, DB_tf.transform.rotation.y, DB_tf.transform.rotation.z, DB_tf.transform.rotation.w])

        #init publishers and subscribers
        self.pub_platform_twist = rospy.Publisher('/platform_setpoint/velocity', TwistStamped, queue_size=1, tcp_nodelay=True)

        sub_drone_twist = Subscriber('/mavros/local_position/velocity', TwistStamped)
        sub_tooltip_twist = Subscriber('/tooltip_setpoint/velocity', TwistStamped)

        ts = ApproximateTimeSynchronizer([sub_drone_twist, sub_tooltip_twist], queue_size=1, slop=0.05)
        ts.registerCallback(self.callback)

    def callback(self, sub_drone_twist, sub_tooltip_twist):
        #initialise position vectors and rotation quaternions

        #setpoint tooltip velocity in world frame
        V_sp = np.asarray([sub_tooltip_twist.linear.x, sub_tooltip_twist.linear.y, sub_tooltip_twist.linear.z])
        #setpoint tooltip angular velocity in world frame
        Omega_sp = np.asarray([sub_tooltip_twist.angular.x, sub_tooltip_twist.angular.y, sub_tooltip_twist.angular.z])
        #measured drone velocity in world frame
        V_drone = np.asarray([sub_drone_twist.linear.x, sub_drone_twist.linear.y, sub_drone_twist.linear.z])
        #measured drone angular velocity in world frame
        Omega_drone = np.asarray([sub_drone_twist.angular.x, sub_drone_twist.angular.y, sub_drone_twist.angular.z])

        w_tf_p = self.tfBuffer.lookup_transform('map', 'platform', time=sub_drone_twist.header.stamp)
        w_q_p = np.asarray([w_tf_p.transform.rotation.x, w_tf_p.transform.rotation.y, w_tf_p.transform.rotation.z, w_tf_p.transform.rotation.w])
        p_q_w = quaternion_conjugate(w_q_p)

        PT_w = quaternion_rotation(self.PT_p, p_q_w) #tooltip vector in world frame

        V_w = V_sp - V_drone - np.cross(Omega_sp, PT_w)
        V_p = quaternion_rotation(V_w, w_q_p)

        Omega_w = Omega_sp - Omega_drone - np.cross(PT_w, V_sp) / np.linalg.norm(PT_w)**2
        Omega_p = quaternion_rotation(Omega_w, w_q_p)
        
        platform_twist = TwistStamped()
        platform_twist.header.frame_id = 'base'
        platform_twist.header.stamp = sub_drone_twist.header.stamp
        platform_twist.twist.linear.x = V_p[0]
        platform_twist.twist.linear.y = V_p[1]
        platform_twist.twist.linear.z = V_p[2]
        platform_twist.twist.angular.x = Omega_p[0]
        platform_twist.twist.angular.y = Omega_p[1]
        platform_twist.twist.angular.z = Omega_p[2]
        self.pub_platform_twist.publish(platform_twist)

def quaternion_rotation(vector, quaternion):
    #assumes quaternion is already unit magnitude (not a problem when dealing with orientation messages)
    vector = np.append(vector, [0.0])
    quaternion_inverse = quaternion_conjugate(quaternion)
    rotated_vector = quaternion_multiply(quaternion, quaternion_multiply(vector, quaternion_inverse))
    return rotated_vector[0:3]

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('velocity_stabilisation')
    vS = velocityStabilisation()
    rospy.spin()