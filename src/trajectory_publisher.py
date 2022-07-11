#! /usr/bin/env python

from genpy import Duration
import rospy
import numpy as np

from tricopter.srv import CalcTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion

class trajectoryGenerator:
    def __init__(self):  
        #Publish target positions for drone and manipulator tooltip
        self.s = rospy.Service('calc_trajectory', CalcTrajectory, self.handle_calc_trajectory) 
        rospy.spin()

    def handle_calc_trajectory(self, req):
        rospy.loginfo("Calculating trajectory")
        
        self.rate = 10
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 2.0 
        self.nFrames = 50
        time = 0.0

        trajectory = MultiDOFJointTrajectory()

        for i in range(self.nFrames):
            point = MultiDOFJointTrajectoryPoint()
            position = Vector3(self.origin_x, self.origin_y, self.origin_z)
            orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
            point.transforms = [Transform(position, orientation)]

            time += 1.0 / self.rate
            point.time_from_start = rospy.Duration.from_sec(time)

            trajectory.points.append(point)

        trajectory_frame_id = "map"
        trajectory.header.frame_id = trajectory_frame_id
        trajectory.joint_names = ["drone"]
        trajectory.header.stamp = rospy.Time.now()
        print(trajectory)

        return trajectory

if __name__ == '__main__': #initialise node
    rospy.init_node('trajectory_publisher', anonymous=True)
    tG = trajectoryGenerator()
    rospy.spin()