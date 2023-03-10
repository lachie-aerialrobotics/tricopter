#! /usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion
from nav_msgs.msg import Odometry
from tricopter.srv import *
from scipy.spatial.transform import Rotation as R

class ViconServer:
    def __init__(self):
        rospy.init_node('vicon_service')
        # services for trajectory generation
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(2.0))
        listener = tf2_ros.TransformListener(self.tfBuffer)
        vicon_service = rospy.Service(
            'align_vicon', alignVicon, self.align_vicon)
        rospy.spin()

    def align_vicon(self, req):
        rospy.loginfo("Vicon alignment requested")

        mavros_pose_frame = "base_link"
        rospy.loginfo("Object frame is "+req.object_name)
        vicon_pose_frame = str(req.object_name)

        # print("Topic is "+vicon_pose_topic)
        # self.vicon_sub = rospy.Subscriber(vicon_pose_topic, PoseStamped, self.vicon_cb)
        # self.mavros_sub = rospy.Subscriber(mavros_pose_topic, PoseStamped, self.mavros_cb)
        
        # rospy.loginfo("Waiting for pose data...")
        # rospy.wait_for_message(mavros_pose_topic, PoseStamped)
        # rospy.wait_for_message(vicon_pose_topic, PoseStamped)
        # rospy.wait_for_message('/Odometry', Odometry)
        # rospy.loginfo("Got pose data!")
        # rospy.loginfo("Waiting 5s for odom data to settle...")
        # rospy.sleep(5)
        got_tf = False
        # tf_lidar2mav = TransformStamped()
        while not got_tf:
            try:
                tf_vicon2mav = self.tfBuffer.lookup_transform(vicon_pose_frame, "mocap", rospy.Time.now(), timeout=rospy.Duration(1))
                tf_lidar2mav = self.tfBuffer.lookup_transform("map", mavros_pose_frame, rospy.Time.now(), timeout=rospy.Duration(1))
                got_tf = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("tf dropped - retrying..")

        mavros_vec = np.asarray([tf_lidar2mav.transform.translation.x, tf_lidar2mav.transform.translation.y, tf_lidar2mav.transform.translation.z])
        mavros_quat = [tf_lidar2mav.transform.rotation.x,tf_lidar2mav.transform.rotation.y, tf_lidar2mav.transform.rotation.z, tf_lidar2mav.transform.rotation.w]
        mavros_rot = R.from_quat(mavros_quat)

        vicon_vec = np.asarray([tf_vicon2mav.transform.translation.x, tf_vicon2mav.transform.translation.y, tf_vicon2mav.transform.translation.z])
        vicon_quat = [tf_vicon2mav.transform.rotation.x, tf_vicon2mav.transform.rotation.y, tf_vicon2mav.transform.rotation.z, tf_vicon2mav.transform.rotation.w]
        vicon_rot = R.from_quat(vicon_quat)

        mav2vic_vec = vicon_vec + mavros_vec
        mav2vic_rot = mavros_rot.inv() * vicon_rot
        mav2vic_quat = mav2vic_rot.as_quat()

        br_static = tf2_ros.StaticTransformBroadcaster()
        tf = TransformStamped()
        tf.header.frame_id = "map"
        tf.header.stamp = rospy.Time.now()
        tf.child_frame_id = "mocap"
        # tf.transform = tf_mav2vicon.transform
        tf.transform.translation = Vector3(mav2vic_vec[0], mav2vic_vec[1], mav2vic_vec[2])
        tf.transform.rotation = Quaternion(mav2vic_quat[0], mav2vic_quat[1], mav2vic_quat[2], mav2vic_quat[3])
        br_static.sendTransform(tf)

        rospy.loginfo("Alignment complete - static tf has been published")

        resp = alignViconResponse()
        resp.transform = tf
         
        return resp

    # def mavros_cb(self, msg):
    #     rospy.loginfo("Retreived pose in odom frame")
    #     self.mavros_pose_msg = msg

    # def vicon_cb(self, msg):
    #     rospy.loginfo("Retreived pose in mocap frame")
    #     self.vicon_pose_msg = msg
        
    
if __name__ == "__main__":
    vS = ViconServer()