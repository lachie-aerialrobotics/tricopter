#! /usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion
from tricopter.srv import *
from scipy.spatial.transform import Rotation as R

class ViconServer:
    def __init__(self):
        rospy.init_node('vicon_service')
        # services for trajectory generation
        vicon_service = rospy.Service(
            'align_vicon', alignVicon, self.align_vicon)
        rospy.spin()

    def align_vicon(self, req):
        rospy.loginfo("Vicon alignment requested")
        resp = alignViconResponse

        mavros_pose_topic = "/mavros/local_position/pose"
        vicon_pose_topic = str("/vicon/"+req.object_name+"/pose")
        print("Topic is "+vicon_pose_topic)
        rospy.loginfo("Waiting for pose data...")
        rospy.wait_for_message(mavros_pose_topic, PoseStamped)
        rospy.wait_for_message(vicon_pose_topic, PoseStamped)
        rospy.loginfo("Got pose data!")

        
        
        self.vicon_sub = rospy.Subscriber(vicon_pose_topic, PoseStamped, self.vicon_cb)
        self.mavros_sub = rospy.Subscriber(mavros_pose_topic, PoseStamped, self.mavros_cb)
        rospy.wait_for_message(mavros_pose_topic, PoseStamped)
        rospy.wait_for_message(vicon_pose_topic, PoseStamped)

        mavros_vec = np.asarray([self.mavros_pose_msg.pose.position.x, self.mavros_pose_msg.pose.position.y, self.mavros_pose_msg.pose.position.z])
        mavros_quat = [self.mavros_pose_msg.pose.orientation.x, self.mavros_pose_msg.pose.orientation.y, self.mavros_pose_msg.pose.orientation.z, self.mavros_pose_msg.pose.orientation.w]
        mavros_rot = R.from_quat(mavros_quat)

        vicon_vec = np.asarray([self.vicon_pose_msg.pose.position.x, self.vicon_pose_msg.pose.position.y, self.vicon_pose_msg.pose.position.z])
        vicon_quat = [self.vicon_pose_msg.pose.orientation.x, self.vicon_pose_msg.pose.orientation.y, self.vicon_pose_msg.pose.orientation.z, self.vicon_pose_msg.pose.orientation.w]
        vicon_rot = R.from_quat(vicon_quat)

        mav2vic_vec = -vicon_vec + mavros_vec
        mav2vic_rot = mavros_rot.inv() * vicon_rot
        mav2vic_quat = mav2vic_rot.as_quat()

        br_static = tf2_ros.StaticTransformBroadcaster()
        tf = TransformStamped()
        tf.header.frame_id = "map"
        tf.header.stamp = rospy.Time.now()
        tf.child_frame_id = "mocap"
        tf.transform.translation = Vector3(mav2vic_vec[0], mav2vic_vec[1], mav2vic_vec[2])
        tf.transform.rotation = Quaternion(mav2vic_quat[0], mav2vic_quat[1], mav2vic_quat[2], mav2vic_quat[3])
        br_static.sendTransform(tf)

        rospy.loginfo("Alignment complete - static tf has been published")
         
        # return resp

    def mavros_cb(self, msg):
        rospy.loginfo("Retreived pose in odom frame")
        self.mavros_pose_msg = msg
        self.mavros_sub.unregister()

    def vicon_cb(self, msg):
        rospy.loginfo("Retreived pose in mocap frame")
        self.vicon_pose_msg = msg
        self.vicon_sub.unregister()
        
    
if __name__ == "__main__":
    vS = ViconServer()