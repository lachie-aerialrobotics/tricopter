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
        #init publisher
        self.pub_map = rospy.Publisher('/map', PointCloud2, queue_size=1)
        #init subscriber
        sub_cloud = rospy.Subscriber('/cloud_registered', PointCloud2, self.cloud_cb, queue_size=1)
        rospy.spin()

    def cloud_cb(self, msg):
        self.pcd = self.pcd + convertCloudFromRosToOpen3d(msg)
        pcd = pcd.voxel_down_sample(voxel_size=0.02)
        pc2 = convertCloudFromOpen3dToRos(pcd, 'map')
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
        open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud
        
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('map_constructor')
    mC = MapConstructor()
    