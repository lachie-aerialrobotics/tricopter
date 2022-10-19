#! /usr/bin/env python3

import rospy
import numpy as np
from controller_msgs.msg import FlatTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandTOL
from tf.transformations import euler_from_quaternion

#function to convert position/velocity/acceleration references to a FlatTarget() message for geometric controller
def flat_target_msg_conversion(mask, pose=PoseStamped(), velocity=TwistStamped(), acceleration=TwistStamped()):
    # uint8 IGNORE_SNAP = 1	- Position Velocity Acceleration Jerk Reference
    # uint8 IGNORE_SNAP_JERK = 2	- Position Velocity Acceleration Reference
    # uint8 IGNORE_SNAP_JERK_ACC = 4	- Position Reference
    target = FlatTarget()
    target.header = pose.header
    target.type_mask = mask
    target.position = pose.pose.position
    target.velocity = velocity.twist.linear
    target.acceleration = acceleration.twist.linear
    (roll, pitch, yaw) = euler_from_quaternion([pose.pose.orientation.x,
                                                pose.pose.orientation.y,
                                                pose.pose.orientation.z,
                                                pose.pose.orientation.w])                                           
    return target, yaw

#function outputs true if velocity and position errors are below a defined threshold
def tolerance_checker(pose_msg, vel_msg, target_pose, pos_tol, vel_tol):
    pos_err = np.asarray([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]) - np.asarray([target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z])
    vel_err = np.asarray([vel_msg.twist.linear.x, vel_msg.twist.linear.y, vel_msg.twist.linear.z])
    pos_err_norm = np.linalg.norm(pos_err)
    vel_err_norm = np.linalg.norm(vel_err)
    rospy.loginfo("Waiting for tolerances to be met:")
    rospy.loginfo("position error = "+str(pos_err_norm)+" m")
    rospy.loginfo("velocity error = "+str(vel_err_norm)+" m/s")
    if (pos_err_norm <= pos_tol) and (vel_err_norm <= vel_tol):
        satisfied = True
    else:
        satisfied = False
    return satisfied   

#function calls mavros landing service
def call_landing_service():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        l = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        l(altitude = 0)
    except rospy.ServiceException:
        rospy.loginfo("Landing service call failed") 