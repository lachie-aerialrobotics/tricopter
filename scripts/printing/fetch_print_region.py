import open3d as o3d
import rospy
import numpy as np
import scipy as sp
import trimesh as tm
import tf2_ros
import copy
import ros_numpy as rnp
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped, Transform, Twist
from nav_msgs.msg import Path
from tricopter.srv import *
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

def get_print_origin():
    print_origin_tf = TransformStamped()
    print_origin_tf.header.stamp = rospy.Time.now()
    print_origin_tf.header.frame_id = rospy.get_param("print_transform/frame_id")
    print_origin_tf.child_frame_id = rospy.get_param("print_transform/child_frame_id")
    print_origin_tf.transform.translation.x = rospy.get_param("print_transform/translation/x")
    print_origin_tf.transform.translation.y = rospy.get_param("print_transform/translation/y")
    print_origin_tf.transform.translation.z = rospy.get_param("print_transform/translation/z")
    print_origin_tf.transform.rotation.x = rospy.get_param("print_transform/orientation/x")
    print_origin_tf.transform.rotation.y = rospy.get_param("print_transform/orientation/y")
    print_origin_tf.transform.rotation.z = rospy.get_param("print_transform/orientation/z")
    print_origin_tf.transform.rotation.w = rospy.get_param("print_transform/orientation/w")

    br= tf2_ros.StaticTransformBroadcaster()
    br.sendTransform(print_origin_tf)

    return print_origin_tf

def transform_to_numpy(transform=TransformStamped(), quat_order='xyzw'):
    t_vec = np.asarray([transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z])
    if quat_order == 'xyzw':
        t_quat = np.asarray([transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w])
    elif quat_order == 'wxyz':
        t_quat = np.asarray([transform.transform.rotation.w,
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z])
    return t_vec, t_quat

def transform_o3d(pc, tf=TransformStamped()):
    v, quat_wxyz = transform_to_numpy(tf, quat_order='wxyz')      
    pc.translate(v)
    pc.rotate(pc.get_rotation_matrix_from_quaternion(quat_wxyz), center=v)
    return pc

def get_crop_region(mesh):
    mesh_scaled = copy.deepcopy(mesh)
    mesh_scaled = mesh_scaled.scale(1.5, center=np.asarray([0,0,0]))
    bbox = mesh_scaled.get_axis_aligned_bounding_box()
    bbox = o3d.geometry.OrientedBoundingBox.create_from_axis_aligned_bounding_box(bbox)
    return bbox

def get_mesh():
    mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=0.3, height=0.5, resolution=100, split=50)
    mesh = mesh.translate(np.asarray([0, 0, 0.23]))
    return mesh

def fetch_print_region():
    # get print location from config and broadcast static transform
    print_origin_tf = get_print_origin()
    print_frame = print_origin_tf.child_frame_id
    map_frame = print_origin_tf.header.frame_id

    # get mesh
    mesh = get_mesh()

    # get bounding box to crop pointcloud
    bbox = get_crop_region(mesh)
    bbox = transform_o3d(bbox, print_origin_tf)
    return bbox, print_frame, map_frame