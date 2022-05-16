#! /usr/bin/env python
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, Imu

class Converter:
    def __init__(self):   
        #Publish pose
        self.lidar_pose_pub= rospy.Publisher('/os_cloud_node/points_stamped', PointCloud2, queue_size=1, tcp_nodelay=True)
        self.lidar_imu_pub= rospy.Publisher('/imu/data_rpy', Imu, queue_size=1, tcp_nodelay=True)
        
        #Subscribe to transform
        self.lidar_pose_sub = rospy.Subscriber('/os_cloud_node/points', PointCloud2, self.points_callback, tcp_nodelay=True)
        self.lidar_imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback, tcp_nodelay=True)
        
    def points_callback(self, lidar_point_msg):
        lidar_point_msg_stamped = lidar_point_msg
        lidar_point_msg_stamped.header.stamp = rospy.Time.now()

        self.lidar_pose_pub.publish(lidar_point_msg_stamped)

    def imu_callback(self, lidar_imu_msg):
        lidar_imu_rpy_msg = Imu
        lidar_imu_rpy_msg = lidar_imu_msg
        lidar_imu_rpy_msg.header.stamp = rospy.Time.now()
        # lidar_imu_rpy_msg.orientation.x=0.0
        # lidar_imu_rpy_msg.orientation.y=0.0
        # lidar_imu_rpy_msg.orientation.z=0.0
        # lidar_imu_rpy_msg.orientation.w = -lidar_imu_rpy_msg.orientation.w
        # lidar_imu_rpy_msg.angular_velocity.x=0.0
        # lidar_imu_rpy_msg.angular_velocity.y=0.0
        # lidar_imu_rpy_msg.angular_velocity.z=0.0
        # lidar_imu_rpy_msg.linear_acceleration.x=0.0
        # lidar_imu_rpy_msg.linear_acceleration.y=0.0
        # lidar_imu_rpy_msg.linear_acceleration.z=9.80665
        self.lidar_imu_pub.publish(lidar_imu_rpy_msg)

if __name__ == '__main__': #initialise node
    rospy.init_node('Lidar_msg_stamper', anonymous=True)
    c = Converter()
    rospy.spin()