#! /usr/bin/env python

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from std_msgs.msg import Float32
from controller_msgs.msg import FlatTarget
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_conjugate

class trajectoryFollower:
    tip_offset = Vector3Stamped()
    tip_offset.vector.x = 0.0
    tip_offset.vector.y = 0.0
    tip_offset.vector.z = 0.4
    drone_yaw_sp = 0.0

    target_msg = FlatTarget()
    target_msg.header.stamp = rospy.Time(0)
    target_msg.position.x = 0.0
    target_msg.position.y = 0.0
    target_msg.position.z = 0.0
    target_msg.velocity.x = 0.0
    target_msg.velocity.y = 0.0
    target_msg.velocity.z = 0.0
    drone_time = target_msg.header.stamp
    drone_pos_sp = target_msg.position
    drone_vel_sp = target_msg.velocity

    def __init__(self): 

        self.rate = 100

        #init tf listener
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        #obtain transforms from platform to tooltip and drone base_link to platform_base
        self.DB_tf = tfBuffer.lookup_transform('base_link', 'stewart_base', time=rospy.Time(0), timeout=rospy.Duration(5))

        #publish to geometric controller
        self.geo_pose_sub = rospy.Subscriber('reference/flatsetpoint', FlatTarget, self.target_cb, queue_size=1, tcp_nodelay=True)
        self.geo_yaw_sub = rospy.Subscriber('reference/yaw', Float32, self.yaw_cb, queue_size=1, tcp_nodelay=True)

        self.tooltip_pose_pub = rospy.Publisher('/tooltip_setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.tooltip_vel_pub = rospy.Publisher('/tooltip_setpoint/velocity', TwistStamped, queue_size=1, tcp_nodelay=True)

        rospy.Timer(rospy.Duration(1.0/self.rate), self.reference_cb, reset=True)

    def yaw_cb(self, yaw_msg):
        if yaw_msg.data is not None:
            self.drone_yaw_sp = yaw_msg.data
        else:
            self.drone_yaw_sp = 0.0

    def target_cb(self, target_msg):
        self.drone_pos_sp = target_msg.position
        self.drone_vel_sp = target_msg.velocity

    def reference_cb(self, event):
        DT_d = tf2_geometry_msgs.do_transform_vector3(self.tip_offset, self.DB_tf) #transform tooltip offset to pose relative to base
        DT_d = np.asarray([DT_d.vector.x, DT_d.vector.y, DT_d.vector.z]) + np.asarray([self.DB_tf.transform.translation.x, self.DB_tf.transform.translation.y, self.DB_tf.transform.translation.z])
        q_yaw = quaternion_from_euler(0, 0, self.drone_yaw_sp)

        OD_w = np.asarray([self.drone_pos_sp.x, self.drone_pos_sp.y, self.drone_pos_sp.z])
        OT_w = OD_w + quaternion_rotation(DT_d, q_yaw)

        tip_pose = PoseStamped()
        tip_pose.pose.position.x = OT_w[0]
        tip_pose.pose.position.y = OT_w[1]
        tip_pose.pose.position.z = OT_w[2]
        tip_pose.pose.orientation.x = q_yaw[0]
        tip_pose.pose.orientation.y = q_yaw[1]
        tip_pose.pose.orientation.z = q_yaw[2]
        tip_pose.pose.orientation.w = q_yaw[3]
        tip_pose.header.stamp = rospy.Time.now()
        tip_pose.header.frame_id = "map"
        self.tooltip_pose_pub.publish(tip_pose)

        tip_vel = TwistStamped()
        tip_vel.twist.linear.x = self.drone_vel_sp.x
        tip_vel.twist.linear.y = self.drone_vel_sp.y
        tip_vel.twist.linear.z = self.drone_vel_sp.z
        tip_vel.twist.angular.x = 0.0
        tip_vel.twist.angular.y = 0.0
        tip_vel.twist.angular.z = 0.0
        tip_vel.header.stamp = rospy.Time.now()
        tip_vel.header.frame_id = "map"
        self.tooltip_vel_pub.publish(tip_vel)

def quaternion_rotation(vector, quaternion):
    #assumes quaternion is already unit magnitude (not a problem when dealing with orientation messages)
    vector = np.append(vector, [0.0])
    quaternion_inverse = quaternion_conjugate(quaternion)
    rotated_vector = quaternion_multiply(quaternion, quaternion_multiply(vector, quaternion_inverse))
    return rotated_vector[0:3]


if __name__ == '__main__':
    #initialize node
    rospy.init_node('trajectory_follower', anonymous=True)
    tF = trajectoryFollower()
    rospy.spin()