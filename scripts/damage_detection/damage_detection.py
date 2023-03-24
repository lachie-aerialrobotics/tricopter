#! /usr/bin/env python3
import open3d as o3d
import rospy
import copy
import os
import numpy as np
import ros_numpy as rnp
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from tricopter.srv import *
    
class DamageDetection:
    def __init__(self):
        # get config values from parameter server
        map_topic_name = rospy.get_param('/damage_detection/map_topic_name')
        self.map_frame = rospy.get_param('/damage_detection/map_frame')
        mesh_path = rospy.get_param('/damage_detection/model_dir')
        model_samples = rospy.get_param('/damage_detection/model_samples')
        self.voxel_size = rospy.get_param('/damage_detection/icp_voxel_size')
        self.max_icp_iterations = rospy.get_param('/damage_detection/icp_max_iterations')
        self.eps = rospy.get_param('/damage_detection/damage_scale')
        self.cluster_size = rospy.get_param('/damage_detection/clustering_scale')
        self.min_points = rospy.get_param('/damage_detection/cluster_min_points')

        # init publishers
        self.pub_result = rospy.Publisher('/model_registration_result', PointCloud2, queue_size=1)
        self.pub_init = rospy.Publisher('/model_registration_init', PointCloud2, queue_size=1)
        self.pub_cropped_map = rospy.Publisher('/cropped_map', PointCloud2, queue_size=1)
        self.pub_damage = rospy.Publisher('/damage_points', PointCloud2, queue_size=1)
        self.pub_markers = rospy.Publisher('/damage_markers', MarkerArray, queue_size=1)
        self.pub_poses = rospy.Publisher('/damage_poses', PoseArray, queue_size=1)

        # init subscribers
        sub_map = rospy.Subscriber(map_topic_name, PointCloud2, self.map_cb, queue_size=1)
        sub_init_pose = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose_cb, queue_size=1)
        
        # change working directory to this script - for saving .pcd files
        abspath = os.path.abspath(__file__)
        dname = os.path.dirname(abspath)
        os.chdir(dname)
        
        # generate pointcloud from undamaged cad model
        # self.model_pc = get_pc_from_mesh(mesh_path, model_samples)

        # # perception filter - todo: get rid of the need for this
        # diameter = np.linalg.norm(np.asarray(self.model_pc.get_max_bound()) - np.asarray(self.model_pc.get_min_bound()))
        # radius = diameter * 100
        # camera = [5,0,1.5]
        # _, pt_map = self.model_pc.hidden_point_removal(camera, radius)
        # self.model_pc = self.model_pc.select_by_index(pt_map)

        # get pointcloud from file
        self.model_pc = o3d.io.read_point_cloud(mesh_path)


        #don't start services until map is available
        rospy.wait_for_message(map_topic_name, PointCloud2) 
        rospy.loginfo('Map is ready!')

        # init services
        detection_service = rospy.Service('detect_damage', detectDamage, self.handle_detect_damage)
        rospy.spin()


    def init_pose_cb(self, msg):
        # generate initial pose estimate and do damage detection from rviz
        init_pose = PoseStamped()
        init_pose.header = msg.header
        init_pose.pose = msg.pose.pose
         # call damage detection service
        rospy.wait_for_service('detect_damage')
        get_damage = rospy.ServiceProxy('detect_damage', detectDamage)
        request = detectDamageRequest()
        request.init_pose = init_pose
        response = get_damage(request)

    def map_cb(self, msg):
        self.map_pc2 = msg #save map to object - only bother converting when service is called
          
    def handle_detect_damage(self, req):
        rospy.loginfo("Detection service called!")
        # convert ros msgs to o3d friendly formats
        header = Header(stamp=rospy.Time.now(), frame_id=self.map_frame)
        map_pc = pc2_to_o3d(self.map_pc2)
        init_transformation = posestamped_to_trans_matrix(req.init_pose)

        # transform undamaged model pointcloud to location of initial guess
        init_model_pc = copy.deepcopy(self.model_pc).transform(init_transformation)

        #publish visualisation of initial pose
        header = Header(stamp=rospy.Time.now(), frame_id=self.map_frame)
        self.pub_init.publish(o3d_to_pc2(init_model_pc, header))

        # perform icp registration
        rospy.loginfo("Attempting alignment...")
        result = o3d.pipelines.registration.registration_icp(
            voxel_down_sample(init_model_pc, self.voxel_size), voxel_down_sample(map_pc, self.voxel_size),
            1.0, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.max_icp_iterations))
        
        transformation = result.transformation
        rospy.loginfo('Tranformation matrix is:')
        rospy.loginfo(transformation)

        # publish visualisation of transformed pointcloud (and write to file)
        aligned_model_pc = copy.deepcopy(init_model_pc).transform(transformation)
        o3d.io.write_point_cloud('/home/aam01/aam_ws/src/tricopter/scans/aligned_model_pc.pcd', aligned_model_pc)
        self.pub_result.publish(o3d_to_pc2(aligned_model_pc, header))
        rospy.loginfo("Alignment done!")
        
        # crop map pointcloud based on model limits (and also publish and write to file)
        min = aligned_model_pc.get_min_bound()
        max = aligned_model_pc.get_max_bound()
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min, max_bound=max)
        map_pc_cropped = copy.deepcopy(map_pc).crop(bbox)
        cl, ind = map_pc_cropped.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
        map_pc_cropped = map_pc_cropped.select_by_index(ind)
        o3d.io.write_point_cloud('/home/aam01/aam_ws/src/tricopter/scans/cropped_map_pc.pcd', map_pc_cropped)
        self.pub_cropped_map.publish(o3d_to_pc2(map_pc_cropped, header))

        # find distance from map points to undamaged model, keep any above a certain distance threshold
        threshold = self.eps #setting threshold to cluster radiius seems to yield good results
        distance = np.asarray(map_pc_cropped.compute_point_cloud_distance(aligned_model_pc))
        outliers = np.where(distance > threshold)[0]
        rospy.loginfo("Damaged points are at:")
        rospy.loginfo(outliers)
        outlier_pcd = map_pc_cropped.select_by_index(outliers)

        # publish 'damaged' points
        o3d.io.write_point_cloud('/home/aam01/aam_ws/src/tricopter/scans/damaged_points.pcd', outlier_pcd)
        self.pub_damage.publish(o3d_to_pc2(outlier_pcd, header))

        # run DBSCAN clustering algorithm
        rospy.loginfo('Clustering...')
        labels = np.array(outlier_pcd.cluster_dbscan(eps=self.cluster_size, min_points=self.min_points))
        if len(labels) > 0:
            num_clusters = labels.max() + 1
            rospy.loginfo('Found '+str(num_clusters)+' clusters.')

            # for each cluster, a robot pose for repair must be determined
            marker_array = MarkerArray()
            self.pose_array = PoseArray()
            self.pose_array.header.frame_id = self.map_frame
            self.pose_array.header.stamp = rospy.Time.now()
            rospy.loginfo('Calculating poses...')
            for i in range(num_clusters):
                # extract points belonging to the i-th cluster
                cluster_indices = np.where(labels == i)[0]
                cluster_pcd = outlier_pcd.select_by_index(cluster_indices)

                # compute the centroid of the cluster
                centroid = np.asarray(cluster_pcd.get_center())

                # we now have a single point for each identified damaged area
                # next we compute the closest distance to the model and then calculate normal vector
                distances = np.asarray(aligned_model_pc.compute_point_cloud_distance(o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector([centroid]))))
                nearest_point_index = np.argmin(distances)
                aligned_model_pc.estimate_normals()
                normal = aligned_model_pc.normals[nearest_point_index]

                # get euler angles for pitch and yaw rotation from map pose - we don't care about roll
                x_vector = np.asarray([1, 0, 0])
                normal_xy = np.asarray([normal[0], normal[1], 0])
                yaw = np.arccos(np.dot(normal_xy, x_vector,) / (np.linalg.norm(x_vector) * np.linalg.norm(normal_xy)))
                pitch = np.arccos(np.dot(normal_xy, normal) / (np.linalg.norm(normal_xy) * np.linalg.norm(normal)))

                # transform into manipulator tooltip frame
                q = quaternion_from_euler(np.pi/2, pitch, yaw+np.pi/2)

                normal_pose = Pose()
                normal_pose.position.x = aligned_model_pc.points[nearest_point_index][0]
                normal_pose.position.y = aligned_model_pc.points[nearest_point_index][1]
                normal_pose.position.z = aligned_model_pc.points[nearest_point_index][2]
                normal_pose.orientation.x = q[0]
                normal_pose.orientation.y = q[1]
                normal_pose.orientation.z = q[2]
                normal_pose.orientation.w = q[3]
                self.pose_array.poses.append(normal_pose)
                
                # publish marker message to enable the operator to choose a numbered point
                marker = Marker()
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = self.map_frame
                marker.id = i
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.scale.z = 0.15
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.pose.position.x = centroid[0]
                marker.pose.position.y = centroid[1]
                marker.pose.position.z = centroid[2]
                marker.text = str(i)
                marker_array.markers.append(marker)
            
            self.pub_markers.publish(marker_array)   
        else:
            rospy.loginfo('No damage found!')
            self.pose_array = PoseArray()

        rospy.loginfo("Done!")

        # output poses in service response
        resp = detectDamageResponse()
        resp.target_poses = self.pose_array
        self.pub_poses.publish(self.pose_array)
        return resp

def get_pc_from_mesh(dir, samples):
    mesh = o3d.io.read_triangle_mesh(dir)
    pc = mesh.sample_points_uniformly(samples)
    return pc

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

def posestamped_to_trans_matrix(pose_stamped_msg=PoseStamped()):
    mat = rnp.numpify(pose_stamped_msg.pose)
    return mat

# def trans_matrix_to_posestamped(trans_matrix=np.zeros((4,4)), header=Header()):
#     pose = rnp.msgify(Pose, trans_matrix)
#     # header = Header(stamp=rospy.Time.now(), frame_id='camera_init')
#     posestamped = PoseStamped(header=header, pose=pose)
#     return posestamped

def voxel_down_sample(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    return pcd_down

if __name__ == "__main__":
    rospy.init_node('damage_detection_service_client')
    dD = DamageDetection()