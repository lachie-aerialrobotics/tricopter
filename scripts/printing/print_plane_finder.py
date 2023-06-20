#! /usr/bin/env python3
import open3d as o3d
import rospy
import numpy as np
import scipy as sp
import tf2_ros
import os
import ros_numpy as rnp
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped
from tricopter.srv import *
    
class PrintPlaneFinder:
    def __init__(self):
        # get config values from parameter server
        map_topic_name = "/map"
        self.drone_frame = "base_link"
        self.map_frame = "map"
        self.print_frame = "print_origin"
        self.min_bound = np.asarray([0, -1.5, -1.5])
        self.max_bound = np.asarray([3, 1.5, 1.5])

        self.plane_model = [0, 0, 1, 0]
        self.print_origin = PoseStamped()

        # init tf lookups
        
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(10.0))
        listener = tf2_ros.TransformListener(self.tfBuffer)

        # init publishers
        self.map_cropped_pub = rospy.Publisher('/map_cropped', PointCloud2, queue_size=1, tcp_nodelay=True)
        self.print_origin_pub = rospy.Publisher('/print_origin_current', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.print_origin_saved_pub = rospy.Publisher('/print_origin_saved', PoseStamped, queue_size=1)

        #don't start services until map is available
        rospy.wait_for_message(map_topic_name, PointCloud2) 
        rospy.loginfo('Map is ready!')

        # init subscribers
        sub_map = rospy.Subscriber(map_topic_name, PointCloud2, self.map_cb, queue_size=1)

        # start timer callback
        rospy.Timer(rospy.Duration(0.03), self.timer_cb, reset=True)

        # init services
        print_init_service = rospy.Service('init_print', initPrint, self.init_print)

        rospy.spin()

    def map_cb(self, msg):
        # get current map and crop based on drone position (cropping is done here as it will occur at lower rate and free up resources)
        map = pc2_to_o3d(msg)

        drone_t_vec, drone_t_quat = self.get_drone_tf()

        # create boundingbox, transform to drone position and crop point cloud
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=self.min_bound, max_bound=self.max_bound)
        bbox = o3d.geometry.OrientedBoundingBox.create_from_axis_aligned_bounding_box(bbox)
        bbox = bbox.translate(drone_t_vec)
        bbox = bbox.rotate(bbox.get_rotation_matrix_from_quaternion(np.asarray([drone_t_quat[3], drone_t_quat[0], drone_t_quat[1], drone_t_quat[2]])), center=drone_t_vec)
        map_cropped = map.crop(bbox)

        # fit a plane to the cropped points
        plane_model = get_best_plane(map_cropped)
        if plane_model is not None: self.plane_model = plane_model #complete assignment if None was not returned
        
        # publish cropped pointcloud for visualisation
        map_cropped_msg = o3d_to_pc2(map_cropped, Header(stamp=rospy.Time.now(), frame_id=self.map_frame))
        self.map_cropped_pub.publish(map_cropped_msg)

    def timer_cb(self, event):
        drone_t_vec, drone_t_quat = self.get_drone_tf()

        # find print origin on detected plane
        print_origin = closest_point_on_plane(self.plane_model, init_point=drone_t_vec)
        R = align_axis(drone_t_vec - print_origin)

        # create a tf frame representing the print origin - also publish a pose message to make visualisation easier
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.print_frame
        t.transform.translation.x = print_origin[0]
        t.transform.translation.y = print_origin[1]
        t.transform.translation.z = print_origin[2]
        q = quaternion_from_matrix(R)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)

        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = self.map_frame
        p.pose.position.x = print_origin[0]
        p.pose.position.y = print_origin[1]
        p.pose.position.z = print_origin[2]
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        self.print_origin = p
        self.print_origin_pub.publish(p)

    def get_drone_tf(self):
        # get drone position from tf tree
        try:
            drone_tf = self.tfBuffer.lookup_transform(self.map_frame, self.drone_frame, rospy.Time.now(), timeout=rospy.Duration(1))
            drone_t_vec, drone_t_quat = transform_to_numpy(drone_tf)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("tf dropped...")
            drone_t_vec = np.zeros(3)
            drone_t_quat = np.asarray([0,0,0,1])
        return drone_t_vec, drone_t_quat
    
    def init_print(self, req):
        # populate response with latest collected print origin
        resp = initPrintResponse()
        resp.print_origin = self.print_origin
        # publish saved origin for visualisation
        self.print_origin_saved_pub.publish(self.print_origin)
        # save data to file for re-use
        if req.save_to_file:
            write_pose_to_yaml(self.print_origin, self.print_frame)
        return resp

def write_pose_to_yaml(pose=PoseStamped(), child_frame_id="", dir="../../cfg/print_config/"):
    # change working directory to this script - for saving .pcd files
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)
    filename = "print_origin.yaml"
    try:
        f = open(dir+filename, 'w')
        print(dir+filename)
        f.write('#DO_NOT_MODIFY: This file was autogenerated! \n\n')
        f.write('print_transform:\n')
        f.write('  frame_id: '+str(pose.header.frame_id)+'\n')
        f.write('  child_frame_id: '+str(child_frame_id)+'\n')
        f.write('  translation:\n')
        f.write('    x: '+str(pose.pose.position.x)+'\n')
        f.write('    y: '+str(pose.pose.position.y)+'\n')
        f.write('    z: '+str(pose.pose.position.z)+'\n')
        f.write('  orientation:\n')
        f.write('    x: '+str(pose.pose.orientation.x)+'\n')
        f.write('    y: '+str(pose.pose.orientation.y)+'\n')
        f.write('    z: '+str(pose.pose.orientation.z)+'\n')
        f.write('    w: '+str(pose.pose.orientation.w)+'\n')
        rospy.loginfo("Print origin saved succesfully!")
    except:
        rospy.logwarn("Writing print origin to file failed!")

def pc2_to_o3d(pc2=PointCloud2()):
    pc_np = rnp.numpify(pc2)
    pc_arr = np.zeros([len(pc_np), 3])
    pc_arr[:,0] = pc_np['x']
    pc_arr[:,1] = pc_np['y']
    pc_arr[:,2] = pc_np['z']
    pc_arr = pc_arr[~np.isinf(pc_arr).any(axis=1)]
    pc_arr = pc_arr[~np.isnan(pc_arr).any(axis=1)]
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pc_arr)
    return pc

def o3d_to_pc2(o3d_pc=o3d.geometry.PointCloud(), header=Header()):
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
    # pc_header = Header(stamp=rospy.Time.now(), frame_id='camera_init')
    pc2 = rnp.msgify(PointCloud2, pc_arr, stamp=header.stamp, frame_id=header.frame_id)
    return pc2

def transform_to_numpy(transform=TransformStamped()):
    # t_mat = rnp.numpify(transform.transform)
    t_vec = np.asarray([transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z])
    t_quat = np.asarray([transform.transform.rotation.x,
                         transform.transform.rotation.y,
                         transform.transform.rotation.z,
                         transform.transform.rotation.w])
    return t_vec, t_quat

def closest_point_on_plane(plane_model, init_point=np.asarray([0,0,0])):
    # function to find the closest point on a plane defined by Ax + By + Cz + D = 0
    # relative to a query point
    [A, B, C, D] = plane_model
    [a, b, c] = init_point
    t = - (a*A + b*B + c*C + D) / (A**2 + B**2 + C**2)
    x = a + t*A
    y = b + t*B
    z = c + t*C
    return np.asarray([x,y,z])

def align_axis(dir=np.zeros(3)):
    # function to find the rotation matrix that aligns the z axis of a coordinate frame
    # with a direction vector (dir) and ensure that the x axis is parallel with the
    # x/y plane

    # R1 is the shortest rotation that aligns the z axis with the defined direction
    dir = dir/ np.linalg.norm(dir)
    axis_dir = np.asarray([0,0,1])
    rot1 = np.arccos(np.dot(dir, axis_dir))

    # if input vector has zero magnitude or is already aligned with z-axis, output an identity matrix
    if (np.linalg.norm(dir) == 0) or np.allclose(np.cross(dir, axis_dir), np.zeros(3)):
        R = np.eye(3)
    else:     
        rot_axis = np.cross(dir, axis_dir) / np.linalg.norm(np.cross(dir, axis_dir))
        R1 = rotation_matrix_from_axis_angle(-rot_axis*rot1)

        # given R1, R2 rotates about the new z axis to align the x axis with the x/y plane
        # this uses Rodruiges' axis-angle formula to solve for a rotation about the z-axis 
        # to ensure that the z-component is zero
        k = np.matmul(R1, np.asarray([0,0,1]))
        v = np.matmul(R1, np.asarray([1,0,0]))
        A = np.cross(k,v)[2]
        B = (v - k * np.dot(k,v))[2]
        C = (k * np.dot(k,v))[2]

        # this equation has two solutions - the first solution is tested later to ensure
        # the y-axis points up. If it doesn't, the other solution is substituted.
        rot2 = -np.arccos(C/np.sqrt(A**2 + B**2)) + np.arctan(A/B)
        R2 = rotation_matrix_from_axis_angle(k*rot2)

        # the final rotation R, performs R1 and R2 in sequence
        R = np.matmul(R2,R1)

        # to ensure that the y-axis points up:
        if np.matmul(R,np.asarray([0,1,0]))[2] < 0:
            rot2 = np.arccos(C/np.sqrt(A**2 + B**2)) + np.arctan(A/B)
            R2 = rotation_matrix_from_axis_angle(k*rot2)
            R = np.matmul(R2,R1)
    
    return R

def rotation_matrix_from_axis_angle(axis_angle=np.zeros(3)):
    # convert axis-angle rotation to matrix using scipy transform library
    r = sp.spatial.transform.Rotation.from_rotvec(axis_angle)
    R = r.as_matrix()
    return R

def quaternion_from_matrix(matrix=np.eye(3)):
    r = sp.spatial.transform.Rotation.from_matrix(matrix)
    q = r.as_quat()
    return q

def get_best_plane(pc, threshold=0.01, ransac_n=10, num_iterations=100):
    # fit a plane to a pointcloud using RANSAC

    #check sufficient points exist, else return None
    if len(pc.points) >= ransac_n:
        plane_model, inliers = pc.segment_plane(distance_threshold=threshold,
                                            ransac_n=ransac_n,
                                            num_iterations=num_iterations)
    else:
        plane_model = None
    return plane_model

if __name__ == "__main__":
    rospy.init_node('print_plane_init_service_client')
    pPF = PrintPlaneFinder()