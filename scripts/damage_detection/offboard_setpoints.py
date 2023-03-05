#! /usr/bin/env python3
import open3d as o3d
import rospy
import copy
import os
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import ros_numpy as rnp
import toppra as ta
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header, String
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Transform, TwistStamped, Twist, Vector3, Quaternion, WrenchStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory
from mavros_msgs.msg import State
from tricopter.srv import *
    
class OffboardSetpoints:
    def __init__(self):
        self.map_frame = rospy.get_param('/damage_detection/map_frame')
        self.trajectory_frequency = rospy.get_param('/trajectory_planner/frequency')

        # init publishers and subscribers
        
        self.pub_drone_sp = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.pub_tip_sp = rospy.Publisher('/tooltip_setpoint/pose', PoseStamped, queue_size=1)
        self.pub_tooltip_state = rospy.Publisher('/manipulator/state',  String, queue_size=1, tcp_nodelay=True)
        
        rospy.wait_for_message('/mavros/state', State)
        sub_tip_trajectory = rospy.Subscriber('/repair_trajectory/tooltip', MultiDOFJointTrajectory, self.tooltip_trajectory_cb, queue_size=1)
        sub_drone_trajectory = rospy.Subscriber('/repair_trajectory/drone', MultiDOFJointTrajectory, self.drone_trajectory_cb, queue_size=1)
        sub_drone_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavros_pose_cb, queue_size=1)
        sub_state = rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=5, tcp_nodelay=True)
        
        self.flying_trajectory = False
        self.i = 0

        self.drone_trajectory = MultiDOFJointTrajectory()
        self.tooltip_trajectory = MultiDOFJointTrajectory()

        sp_timer = rospy.Timer(rospy.Duration(1/30), self.sp_timer_cb, reset=True)
        rospy.spin()

    def sp_timer_cb(self, event):
        drone_pose = PoseStamped()
        tip_pose = PoseStamped()
        if self.mavros_state.mode == 'OFFBOARD':
            if self.flying_trajectory is False: 
                self.flying_trajectory = True
                self.trajectory_start_time = rospy.Time.now()
            if (self.i > (len(self.drone_trajectory.points) - 1)) or (self.i > (len(self.tooltip_trajectory.points) - 1)):
                self.i = 0
                self.flying_trajectory = False
            else:
                while (self.drone_trajectory.points[self.i].time_from_start.to_sec() + self.trajectory_start_time.to_sec()) < rospy.Time.now().to_sec():
                    self.i += 1  
                drone_pose = PoseStamped()
                drone_pose.header.frame_id = self.drone_trajectory.header.frame_id
                drone_pose.header.stamp = rospy.Time.now()
                drone_pose.pose.position = self.drone_trajectory.points[self.i].transforms[0].translation
                drone_pose.pose.orientation = self.drone_trajectory.points[self.i].transforms[0].rotation

                tip_pose = PoseStamped()
                tip_pose.header.frame_id = self.tooltip_trajectory.header.frame_id
                tip_pose.header.stamp = rospy.Time.now()
                tip_pose.pose.position = self.tooltip_trajectory.points[self.i].transforms[0].translation
                tip_pose.pose.orientation = self.tooltip_trajectory.points[self.i].transforms[0].rotation

                rospy.loginfo('Speed: '+ str(np.linalg.norm(rnp.numpify(self.drone_trajectory.points[self.i+1].transforms[0].translation) - rnp.numpify(self.drone_trajectory.points[self.i].transforms[0].translation)) * self.trajectory_frequency))
                rospy.loginfo('Time: '+str(self.drone_trajectory.points[self.i].time_from_start.to_sec() + self.trajectory_start_time.to_sec()) )
        else:
            drone_pose = PoseStamped()
            drone_pose.header.frame_id = self.map_frame
            drone_pose.header.stamp = rospy.Time.now()
            drone_pose.pose = self.mavros_pose.pose

            tip_pose = PoseStamped()
            tip_pose.header.frame_id = self.map_frame
            tip_pose.header.stamp = rospy.Time.now()
            tip_pose.pose = self.mavros_pose.pose

        self.pub_drone_sp.publish(drone_pose)
        self.pub_tip_sp.publish(tip_pose)

    def state_cb(self, state_msg):
        self.mavros_state = state_msg
        if self.mavros_state.mode == 'OFFBOARD':
            self.pub_tooltip_state.publish(String("STAB_6DOF"))
        elif self.mavros_state.mode == 'POSCTL':
            self.pub_tooltip_state.publish(String("STAB_3DOF"))
        else:
            self.pub_tooltip_state.publish(String("RETRACTED"))

    def tooltip_trajectory_cb(self, msg):
        self.tooltip_trajectory = msg
        
    def drone_trajectory_cb(self, msg):
        self.drone_trajectory = msg

    def mavros_pose_cb(self, msg):
        self.mavros_pose = msg #save mavros pose to object

if __name__ == "__main__":
    rospy.init_node('setpoint_publisher')
    oS = OffboardSetpoints()