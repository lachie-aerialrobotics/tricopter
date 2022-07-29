#!/usr/bin/env python
from ast import Sub
import rospy
import numpy as np
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from tf.transformations import quaternion_multiply, quaternion_conjugate
from message_filters import ApproximateTimeSynchronizer, Subscriber

#code to generate end-effector setpoints accounting for random drone perturbations
class positionStabilisation:
    def __init__(self):
        #init tf listener
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        #obtain transforms from platform to tooltip and drone base_link to platform_base
        PT_tf = tfBuffer.lookup_transform('platform', 'tooltip', time=rospy.Time(0), timeout=rospy.Duration(5))
        DB_tf = tfBuffer.lookup_transform('base_link', 'stewart_base', time=rospy.Time(0), timeout=rospy.Duration(5))

        #tooltip vector in platform frame, code assumes tooltip is not rotated relative to platform
        self.PT_p = np.asarray([PT_tf.transform.translation.x, PT_tf.transform.translation.y, PT_tf.transform.translation.z]) 
        #vector from base_link to stewart_base in drone coordinates
        self.DB_d = np.asarray([DB_tf.transform.translation.x, DB_tf.transform.translation.y, DB_tf.transform.translation.z]) 
        #rotation from drone frame to stewart_base frame
        self.d_q_b = np.asarray([DB_tf.transform.rotation.x, DB_tf.transform.rotation.y, DB_tf.transform.rotation.z, DB_tf.transform.rotation.w])

        #init publishers and subscribers
        self.pub_platform_pose = rospy.Publisher('/platform_setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)

        sub_drone_pose = Subscriber('/mavros/local_position/pose', PoseStamped)
        sub_tooltip_pose = Subscriber('/tooltip_setpoint/pose', PoseStamped)

        #time sync drone and tooltip setpoints
        ts = ApproximateTimeSynchronizer([sub_drone_pose, sub_tooltip_pose], queue_size=1, slop=0.05)
        ts.registerCallback(self.callback)

    def callback(self, sub_drone_pose, sub_tooltip_pose):
        #initialise position vectors and rotation quaternions
        #tooltip setpoint position in world frame
        OT_w = np.asarray([sub_tooltip_pose.pose.position.x, sub_tooltip_pose.pose.position.y, sub_tooltip_pose.pose.position.z]) 
        #rotation of tooltip/platform in world frame
        w_q_p = np.asarray([sub_tooltip_pose.pose.orientation.x, sub_tooltip_pose.pose.orientation.y, sub_tooltip_pose.pose.orientation.z, sub_tooltip_pose.pose.orientation.w]) 
         #measured drone base_link position in world frame
        OD_w = np.asarray([sub_drone_pose.pose.position.x, sub_drone_pose.pose.position.y, sub_drone_pose.pose.position.z])
        #measured drone base_link orientation in world frame
        w_q_d = np.asarray([sub_drone_pose.pose.orientation.x, sub_drone_pose.pose.orientation.y, sub_drone_pose.pose.orientation.z, sub_drone_pose.pose.orientation.w])
        
        w_q_b = quaternion_multiply(w_q_d, self.d_q_b) #rotation from world frame to stewart_base frame
        b_q_w = quaternion_conjugate(w_q_b) #rotation from stewart_base frame to world_frame
        b_q_p = quaternion_multiply(b_q_w, w_q_p) #rotation from base_frame to platform_frame

        DB_w = quaternion_rotation(self.DB_d, w_q_d) #drone base_link to stewart_base in world frame

        PT_w = quaternion_rotation(self.PT_p, w_q_p) #tooltip vector in world frame
        BP_w = OT_w - OD_w - DB_w - PT_w #stewart_base to platform vector in world frame

        BP_b = quaternion_rotation(BP_w, b_q_w) #stewart_base to platform vector in stewart_base frame
        
        #fill in pose msg and publish
        platform_pose = PoseStamped()
        platform_pose.header.frame_id = 'stewart_base'
        platform_pose.header.stamp = sub_drone_pose.header.stamp
        platform_pose.pose.position.x = BP_b[0]
        platform_pose.pose.position.y = BP_b[1]
        platform_pose.pose.position.z = BP_b[2]
        platform_pose.pose.orientation.x = b_q_p[0]
        platform_pose.pose.orientation.y = b_q_p[1]
        platform_pose.pose.orientation.z = b_q_p[2]
        platform_pose.pose.orientation.w = b_q_p[3]
        self.pub_platform_pose.publish(platform_pose)

def quaternion_rotation(vector, quaternion):
    #assumes quaternion is already unit magnitude (not a problem when dealing with orientation messages)
    vector = np.append(vector, [0.0])
    quaternion_inverse = quaternion_conjugate(quaternion)
    rotated_vector = quaternion_multiply(quaternion, quaternion_multiply(vector, quaternion_inverse))
    return rotated_vector[0:3]

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('position_stabilisation')
    pS = positionStabilisation()
    rospy.spin()