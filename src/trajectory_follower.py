#! /usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Float32
from controller_msgs.msg import FlatTarget
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler

class trajectoryFollower:
    def __init__(self): 
        self.rate = 30

        #initialize node
        rospy.init_node('trajectory_follower', anonymous=True)

        #publish to geometric controller
        self.geo_pose_pub = rospy.Publisher('reference/flatsetpoint', FlatTarget, queue_size=1, tcp_nodelay=True)
        self.geo_yaw_pub = rospy.Publisher('reference/yaw', Float32, queue_size=1, tcp_nodelay=True)
        # self.geo_pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.tooltip_pose_pub = rospy.Publisher('/tooltip_setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)

        rospy.Timer(rospy.Duration(1.0/self.rate), self.reference_cb, reset=True)

        rospy.spin()

    def reference_cb(self, event):
        traj = MultiDOFJointTrajectory()
        traj.header.frame_id = "map"

        pose = FlatTarget()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.7
        pose.velocity.x = 0.0
        pose.velocity.y = 0.0
        pose.velocity.z = 0.0
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = traj.header.frame_id
        self.geo_pose_pub.publish(pose)

        yaw = Float32()
        yaw.data = 0.0
        self.geo_yaw_pub.publish(yaw)

        # pose = PoseStamped()
        # pose.pose.position.x = 0.0
        # pose.pose.position.y = 0.0
        # pose.pose.position.z = 0.7
        # pose.pose.orientation.x = 0.0
        # pose.pose.orientation.y = 0.0
        # pose.pose.orientation.z = 0.0
        # pose.pose.orientation.w = 1.0
        # pose.header.stamp = rospy.Time.now()
        # pose.header.frame_id = traj.header.frame_id
        # self.geo_pose_pub.publish(pose)

        tip_quat = quaternion_from_euler(np.pi/2, 0, np.pi/2)

        tip_pose = PoseStamped()
        tip_pose.pose.position.x = pose.position.x + 0.6
        tip_pose.pose.position.y = pose.position.y
        tip_pose.pose.position.z = pose.position.z + 0.3
        tip_pose.pose.orientation.x = tip_quat[0]
        tip_pose.pose.orientation.y = tip_quat[1]
        tip_pose.pose.orientation.z = tip_quat[2]
        tip_pose.pose.orientation.w = tip_quat[3]
        tip_pose.header.stamp = pose.header.stamp
        tip_pose.header.frame_id = traj.header.frame_id
        self.tooltip_pose_pub.publish(tip_pose)


if __name__ == '__main__':
    tF = trajectoryFollower()