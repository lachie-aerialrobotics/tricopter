#! /usr/bin/env python3
import rospy
import numpy as np
import ros_numpy as rnp
import message_filters as mf
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from tricopter.cfg import PIDConfig

class TrajectoryController:
    def __init__(self):
        srv = Server(PIDConfig, self.config_cb)

        self.err = 0.0
        self.i_err = 0.0
        self.t = rospy.Time.now()

        #init publisher
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1, tcp_nodelay=True)
        
        #drone state subscribers
        sub_mav_pose = mf.Subscriber('/mavros/local_position/pose', PoseStamped, queue_size=1, tcp_nodelay=True)

        #setpoint subscribers
        sub_sp_pose = mf.Subscriber('/setpoint/pose', PoseStamped)
        sub_sp_vel = mf.Subscriber('/setpoint/vel', TwistStamped)

        ats = mf.ApproximateTimeSynchronizer([sub_mav_pose, sub_sp_pose, sub_sp_vel], queue_size=1, slop=0.1)
        ats.registerCallback
        ats.registerCallback(self.controller_cb)
        rospy.spin()

    def controller_cb(self, mav_pose_msg, sp_pose_msg, sp_vel_msg):
        sp_vel = twist_2_array(sp_vel_msg)
        sp_pose = pose_2_array(sp_pose_msg)
        mav_pose = pose_2_array(mav_pose_msg)

        vel_cmd = sp_vel + self.pos_ctrl(mav_pose, sp_pose)

        self.pub_vel.publish(array_2_twist(vel_cmd))

    def pos_ctrl(self, mav_pose, sp_pose):
        err = sp_pose - mav_pose

        dt = (rospy.Time.now() - self.t).to_sec()
        d_err = (err - self.err) / dt
        self.i_err += err * dt

        self.err = err
        self.t = rospy.Time.now()

        vel_cmd = self.P * err + self.I * self.i_err + self.D * d_err

        for i in range(len(vel_cmd)):
            if vel_cmd[i] > self.v_max[i]:
                vel_cmd[i] = self.v_max[i]
            elif vel_cmd[i] < -self.v_max[i]:
                vel_cmd[i] = -self.v_max[i]

        return vel_cmd
    
    def config_cb(self, config, level):
        self.v_max = np.asarray([config.vmax_xyz, 
                                 config.vmax_xyz, 
                                 config.vmax_xyz, 
                                 config.vmax_yaw])

        self.P = np.asarray([config.P_xy, 
                             config.P_xy, 
                             config.P_z, 
                             config.P_yaw])
        
        self.I = np.asarray([config.I_xy, 
                             config.I_xy, 
                             config.I_z, 
                             0])
        
        self.D = np.asarray([config.D_xy, 
                             config.D_xy, 
                             config.D_z, 
                             0])
        
        return config

def twist_2_array(msg):
    x = msg.twist.linear.x
    y = msg.twist.linear.y
    z = msg.twist.linear.z
    # r = msg.twist.angular.x
    # p = msg.twist.angular.y
    yaw = msg.twist.angular.z
    return np.asarray([x, y, z, yaw])

def pose_2_array(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    q = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    r, p, yaw = euler_from_quaternion(q)
    return np.asarray([x, y, z, yaw])

def array_2_twist(arr):
    msg = TwistStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'map'
    msg.twist.linear.x = arr[0]
    msg.twist.linear.y = arr[1]
    msg.twist.linear.z = arr[2]
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = arr[3]
    return msg
 
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('trajectory_controller')
    tC = TrajectoryController()
    