#! /usr/bin/env python
import rospy
import numpy as np

from genpy import Duration
from tricopter.srv import CalcTrajectory, CalcTrajectoryResponse
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion

class trajectoryGenerator:
    def __init__(self):  
        self.s = rospy.Service('calc_trajectory', CalcTrajectory, self.handle_calc_trajectory) 
        self.rate = 10
        rospy.spin()

    def handle_calc_trajectory(self, req):
        # rospy.loginfo("Calculating trajectory...")
        
        time = 0.0

        trajectory = MultiDOFJointTrajectory()

        x, y, z, x_dot, y_dot, z_dot, self.nFrames = print_circle(self.rate, 0)

        for i in range(self.nFrames):

            x, y, z, x_dot, y_dot, z_dot, mFrames = print_circle(self.rate, i)

            point = MultiDOFJointTrajectoryPoint()

            point.transforms = [Transform(Vector3(x, y, z), 
                                    Quaternion(0.0, 0.0, 0.0, 1.0))]

            point.velocities = [Twist(Vector3(x_dot, y_dot, z_dot),
                                    Vector3(0.0, 0.0, 0.0))]
                                    
            time += 1.0 / self.rate
            point.time_from_start = rospy.Duration.from_sec(time)

            trajectory.points.append(point)

        trajectory_frame_id = "map"
        trajectory.header.frame_id = trajectory_frame_id
        trajectory.header.stamp = rospy.Time.now()

        return CalcTrajectoryResponse(trajectory)

def print_circle(rate, i):
    origin_x = 0.0
    origin_y = 0.0
    origin_z = 2.0
    speed = 0.5
    radius = 1.0

    delta_theta = speed / (radius * rate)

    x = radius * np.sin(i * delta_theta) + origin_x
    y = 0.0 + origin_y
    z = radius * np.cos(i * delta_theta) + origin_z

    x_dot = speed * np.cos(i * delta_theta)
    y_dot = 0.0
    z_dot = speed * np.sin(i * delta_theta)

    T = 2 * np.pi * radius / speed

    nFrames = int(T * rate)

    return x, y, z, x_dot, y_dot, z_dot, nFrames

if __name__ == '__main__': #initialise node
    rospy.init_node('trajectory_publisher', anonymous=True)
    tG = trajectoryGenerator()
    rospy.spin()