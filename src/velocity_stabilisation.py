#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import TwistStamped
from tf.transformations import quaternion_multiply, quaternion_conjugate
from message_filters import ApproximateTimeSynchronizer, Subscriber

#code to generate end-effector setpoints accounting for random drone perturbations
class velocityStabilisation:
    def __init__(self):
        #init tf listener
        self.tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(self.tfBuffer)

        #init publishers and subscribers
        self.pub_platform_twist = rospy.Publisher('/platform_setpoint/velocity', TwistStamped, queue_size=1, tcp_nodelay=True)

        sub_drone_twist = Subscriber('/mavros/local_position/velocity_local', TwistStamped)
        sub_tooltip_twist = Subscriber('/tooltip_setpoint/velocity', TwistStamped)

        ts = ApproximateTimeSynchronizer([sub_drone_twist, sub_tooltip_twist], queue_size=1, slop=0.05)
        ts.registerCallback(self.callback)

    def callback(self, sub_drone_twist, sub_tooltip_twist):
        try: #attempt to lookup transform to platform and do velocity kinematics. If not available, pass and try again
            w_tf_t = self.tfBuffer.lookup_transform('map', 'tooltip', time=sub_drone_twist.header.stamp, timeout=rospy.Duration(0.01))
            w_tf_d = self.tfBuffer.lookup_transform('map', 'base_link', time=sub_drone_twist.header.stamp, timeout=rospy.Duration(0.01))
            w_tf_b = self.tfBuffer.lookup_transform('map', 'stewart_base', time=sub_drone_twist.header.stamp, timeout=rospy.Duration(0.01))
            
            #setpoint tooltip velocity in world frame
            V_sp = np.asarray([sub_tooltip_twist.twist.linear.x, sub_tooltip_twist.twist.linear.y, sub_tooltip_twist.twist.linear.z])
            #setpoint tooltip angular velocity in world frame
            Omega_sp = np.asarray([sub_tooltip_twist.twist.angular.x, sub_tooltip_twist.twist.angular.y, sub_tooltip_twist.twist.angular.z])
            #measured drone velocity in world frame
            V_d_w = np.asarray([sub_drone_twist.twist.linear.x, sub_drone_twist.twist.linear.y, sub_drone_twist.twist.linear.z])
            #measured drone angular velocity in world frame
            Omega_d_w = np.asarray([sub_drone_twist.twist.angular.x, sub_drone_twist.twist.angular.y, sub_drone_twist.twist.angular.z])
            
            #initialise position vectors and rotation quaternions
            OT_w = np.asarray([w_tf_t.transform.translation.x, w_tf_t.transform.translation.y, w_tf_t.transform.translation.z]) 
            OD_w = np.asarray([w_tf_d.transform.translation.x, w_tf_d.transform.translation.y, w_tf_d.transform.translation.z]) 
            DT_w = OT_w - OD_w

            V_t_w = -V_d_w - np.cross(DT_w, Omega_d_w) + V_sp

            # Omega_t_w = -Omega_d_w + Omega_sp 
    
            w_q_b = np.asarray([w_tf_b.transform.rotation.x, w_tf_b.transform.rotation.y, w_tf_b.transform.rotation.z, w_tf_b.transform.rotation.w])

            V_t_p = quaternion_rotation(V_t_w, quaternion_conjugate(w_q_b))
            # Omega_t_p = quaternion_rotation(Omega_t_w, quaternion_conjugate(w_q_b))
            Omega_t_p = np.asarray([0.0, 0.0, 0.0])
            
            platform_twist = TwistStamped()
            platform_twist.header.frame_id = 'stewart_base'
            platform_twist.header.stamp = sub_drone_twist.header.stamp
            platform_twist.twist.linear.x = V_t_p[0]
            platform_twist.twist.linear.y = V_t_p[1]
            platform_twist.twist.linear.z = V_t_p[2]
            platform_twist.twist.angular.x = Omega_t_p[0]
            platform_twist.twist.angular.y = Omega_t_p[1]
            platform_twist.twist.angular.z = Omega_t_p[2]
            self.pub_platform_twist.publish(platform_twist)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

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