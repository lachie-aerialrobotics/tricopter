#! /usr/bin/env python3
import open3d as o3d

import rospy
import numpy as np
import ros_numpy as rnp


from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from ctypes import * # convert float to uint32

import sensor_msgs.point_cloud2 as pc2

class MapConstructor:
    def __init__(self):
        self.pcd = o3d.geometry.PointCloud()
        self.pcd_new = PointCloud2()
        #init publisher
        self.pub_map = rospy.Publisher('/map', PointCloud2, queue_size=1)
        #init subscriber
        sub_cloud = rospy.Subscriber('/cloud_registered', PointCloud2, self.cloud_cb, queue_size=1)

        rospy.wait_for_message('/cloud_registered', PointCloud2)
        sp_timer = rospy.Timer(rospy.Duration(2.0), self.timer_cb, reset=True)

        rospy.spin()

    def cloud_cb(self, msg):
        self.pcd_new = msg

    def timer_cb(self, event):
        pcd_o3d_new = convertCloudFromRosToOpen3d(self.pcd_new)
        self.pcd = self.pcd + pcd_o3d_new.translate((0,0,-0.15))
        self.pcd = self.pcd.voxel_down_sample(voxel_size=0.002)
        # cl, ind = self.pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.1)
        # self.pcd = self.pcd.select_by_index(ind)
        pc2 = convertCloudFromOpen3dToRos(self.pcd, 'camera_init')
        self.pub_map.publish(pc2)

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points
    else: # XYZ + RGB
        fields=FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
        cloud_data=np.c_[points, colors]
    
    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)

def convertCloudFromRosToOpen3d(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        # print(cloud_data)
        xyz = [(x,y,z) for x,y,z,a,b,c,d,e in cloud_data ] # get xyz
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud
        
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('map_constructor')
    mC = MapConstructor()
    