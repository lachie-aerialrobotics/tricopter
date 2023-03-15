#! /usr/bin/env python3
import rospy
import numpy as np
import ros_numpy as rnp
import open3d as o3d

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

class MapConstructor:
    def __init__(self):
        #init publisher
        self.pub_map = rospy.Publisher('/map', PointCloud2, queue_size=1)
        self.pcd = o3d.geometry.PointCloud()
        #init subscriber
        sub_cloud = rospy.Subscriber('/cloud_registered', PointCloud2, self.cloud_cb, queue_size=1)

        rospy.spin()

        

    def cloud_cb(self, msg):
        self.pcd = self.pcd + pc2_to_o3d(msg)
        pcd = pcd.voxel_down_sample(voxel_size=0.02)
        pc2 = o3d_to_pc2(pcd, msg.header)
        self.pub_map.publish(pc2)

     
        
def o3d_to_pc2(o3d_pc=o3d.geometry.PointCloud(), pc_header=Header()):
    #convert back to structured array
    pc_np = np.asarray(o3d_pc.points)
    pc_arr = np.zeros(np.shape(pc_np)[0], dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        # ('rgb', np.float32),
        ])
    pc_arr['x'] = pc_np[:, 0]
    pc_arr['y'] = pc_np[:, 1]
    pc_arr['z'] = pc_np[:, 2]
    #msgify back to pointcloud2
    pc2 = rnp.msgify(PointCloud2, pc_arr, stamp=pc_header.stamp, frame_id=pc_header.frame_id)
    return pc2

def pc2_to_o3d(pc2=PointCloud2()):
    #numpify pointcloud2 message
    pc_np = rnp.numpify(pc2)

    #convert to unstructured array
    pc_arr = np.zeros([len(pc_np), 3])
    pc_arr[:, 0] = pc_np['x']
    pc_arr[:, 1] = pc_np['y']
    pc_arr[:, 2] = pc_np['z']
    # pc_arr[:, 3] = pc_np['rgb']
    pc_arr = pc_arr[~np.isinf(pc_arr).any(axis=1)] #remove all rows containing inf entries
    pc_arr = pc_arr[~np.isnan(pc_arr).any(axis=1)] #remove all rows containing nan entries

    o3d_pc = o3d.geometry.PointCloud()
    o3d_pc.points = o3d.utility.Vector3dVector(pc_arr)

    return o3d_pc
        
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('map_constructor')
    mC = MapConstructor()
    