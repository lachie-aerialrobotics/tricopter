#! /usr/bin/env python3
import open3d as o3d
import rospy
import copy
import numpy as np
import ros_numpy as rnp


from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Pose
from tricopter.srv import *
    
class DamageDetection:
    def __init__(self):
        map_topic_name = '/LaserMap'
        mesh_path = '/home/aam01/aam_ws/src/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/ARLarena/meshes/FoamBoardNoHoles.stl'
        samples = 100000
        self.registration_threshold = 0.002
        self.voxel_size = 0.005
        self.max_icp_iterations = 500

        sub_map = rospy.Subscriber(map_topic_name, PointCloud2, self.map_cb, queue_size=1)
        self.pub_result = rospy.Publisher('/model_registration_result', PointCloud2, queue_size=1)
        self.pub_init = rospy.Publisher('/model_registration_init', PointCloud2, queue_size=1)
        self.pub_cropped_map = rospy.Publisher('/cropped_map', PointCloud2, queue_size=1)
        self.pub_damage = rospy.Publisher('/damage_points', PointCloud2, queue_size=1)
        
        self.model_pc = get_pc_from_mesh(mesh_path, samples)

        diameter = np.linalg.norm(np.asarray(self.model_pc.get_max_bound()) - np.asarray(self.model_pc.get_min_bound()))
        radius = diameter * 100
        camera = [5,0,1.5]
        _, pt_map = self.model_pc.hidden_point_removal(camera, radius)
        self.model_pc = self.model_pc.select_by_index(pt_map)

        rospy.wait_for_message(map_topic_name, PointCloud2) #don't start service until map is available
        rospy.loginfo('Map is available!')
        detection_service = rospy.Service('detect_damage', detectDamage, self.handle_detect_damage)
        rospy.spin()

    def map_cb(self, msg):
        self.map_pc2 = msg #save map to object - only bother converting when service is called

    def handle_detect_damage(self, req):
        rospy.loginfo("Detection service called!")
        # convert ros msgs to o3d friendly formats
        map_pc = pc2_to_o3d(self.map_pc2)

        init_transformation = posestamped_to_trans_matrix(req.init_pose)
        init_model_pc = copy.deepcopy(self.model_pc).transform(init_transformation)

        #publish initial pose
        self.pub_init.publish(o3d_to_pc2(init_model_pc))

        # perform registration
        rospy.loginfo("Attempting alignment...")
        result = o3d.pipelines.registration.registration_icp(
            voxel_down_sample(init_model_pc, self.voxel_size), voxel_down_sample(map_pc, self.voxel_size),
            1.0, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.max_icp_iterations))

        # result = o3d.pipelines.registration.registration_icp(
        #     init_model_pc, map_pc, self.registration_threshold, np.identity(4), 
        #         o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        #         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))

        # map_pc_down, map_pc_fpfh = preprocess_point_cloud(map_pc, self.voxel_size)
        # model_pc_down, model_pc_fpfh = preprocess_point_cloud(init_model_pc, self.voxel_size)

        # result = execute_global_registration(model_pc_down, map_pc_down, map_pc_fpfh,
        #                         model_pc_fpfh, self.voxel_size)
        
        transformation = result.transformation
        rospy.loginfo('Tranformation matrix is:')
        rospy.loginfo(transformation)

        # output registered pose
        target_pose = trans_matrix_to_posestamped(transformation)
        resp = detectDamageResponse()
        resp.target_pose = target_pose

        # publish transformed pointcloud for visualisation
        aligned_model_pc = copy.deepcopy(init_model_pc).transform(transformation)
        self.pub_result.publish(o3d_to_pc2(aligned_model_pc))
        rospy.loginfo("Alignment done!")
        
        min = aligned_model_pc.get_min_bound() #- [0.5, 0.5, 0.5]
        max = aligned_model_pc.get_max_bound() #+ [0.5, 0.5, 0.5]
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min, max_bound=max)
        map_pc_cropped = copy.deepcopy(map_pc).crop(bbox)
        self.pub_cropped_map.publish(o3d_to_pc2(map_pc_cropped))

        threshold = 0.05

        distance = np.asarray(map_pc_cropped.compute_point_cloud_distance(aligned_model_pc))
        outliers = np.where(distance > threshold)[0]
        rospy.loginfo("Damaged points are at:")
        rospy.loginfo(outliers)

        outlier_pcd = map_pc_cropped.select_by_index(outliers)
        self.pub_damage.publish(o3d_to_pc2(outlier_pcd))
        
        rospy.loginfo("Done!")
        print()
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

def o3d_to_pc2(o3d_pc=o3d.geometry.PointCloud()):
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
    pc_header = Header(stamp=rospy.Time.now(), frame_id='camera_init')
    pc2 = rnp.msgify(PointCloud2, pc_arr, stamp=pc_header.stamp, frame_id=pc_header.frame_id)
    return pc2

def posestamped_to_trans_matrix(pose_stamped_msg=PoseStamped()):
    mat = rnp.numpify(pose_stamped_msg.pose)
    return mat

def trans_matrix_to_posestamped(trans_matrix=np.zeros((4,4))):
    pose = rnp.msgify(Pose, trans_matrix)
    header = Header(stamp=rospy.Time.now(), frame_id='camera_init')
    posestamped = PoseStamped(header=header, pose=pose)
    return posestamped

def preprocess_point_cloud(pcd, voxel_size):
    rospy.loginfo(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    rospy.loginfo(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    rospy.loginfo(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    rospy.loginfo(":: RANSAC registration on downsampled point clouds.")
    rospy.loginfo("   Since the downsampling voxel size is %.3f," % voxel_size)
    rospy.loginfo("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])

def voxel_down_sample(pcd, voxel_size):
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down

if __name__ == "__main__":
    rospy.init_node('damage_detection_service_client')
    dD = DamageDetection()