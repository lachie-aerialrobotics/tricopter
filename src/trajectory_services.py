#! /usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped, TransformStamped, Transform, Twist, WrenchStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tricopter.srv import *

import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo

def handle_fetch_poses(req):
    # service fetches waypoints from the ros parameter server for a given print layer. The format
    # should be as follows:

    # prefix:
    #   n_layers: N
    #   layer0:
    #       point0: [x, y, z, roll, pitch, yaw]
    #       ...     [x, y, z, roll, pitch, yaw]
    #       pointN: [x, y, z, roll, pitch, yaw]
    #   ...
    #   layerN:
    #       point0: [x, y, z, roll, pitch, yaw]
    #       ...

    rospy.loginfo("Fetching poses from .yaml file")
    param_list = rospy.get_param_names()
    
    poses = PoseArray()
    poses.header.frame_id = req.frame_id
    poses.header.stamp = rospy.Time.now()

    if int(rospy.get_param("/"+str(req.prefix)+"/n_layers")) < req.layer_number:
        rospy.logwarn("Requested layer is greater than layers specified in print")

    points = list(filter(lambda k: str(req.prefix) in k, param_list))
    
    points = list(filter(lambda k: ("/layer" + str(req.layer_number)) in k, points))

    if not points:
        rospy.logwarn("No waypoints found - please check .yaml file") 
    else:
        for i in range(len(points)):
            try:
                point_ref = list(filter(lambda k: ("/point" + str(i)) in k, points))  
                point = rospy.get_param(str(point_ref[0]))
                pose = Pose()
                pose.position.x = point[0]
                pose.position.y = point[1]
                pose.position.z = point[2]
                q = quaternion_from_euler(point[3], point[4], point[5])
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                poses.poses.append(pose)
            except:
                rospy.logwarn("Error extracting waypoints - please check .yaml file")
                break

    resp = fetchPosesResponse()
    resp.poses = poses
    rospy.loginfo("Poses returned")
    return resp

def handle_drone_trajectory(req):
    # service generates a drone trajectory offset from a supplied tooltip trajectory. x/y/z and yaw commands
    # are preserved and pitch/roll angles are ignored
    rospy.loginfo("Offset drone trajectory requested")
    tip_traj = req.toolpath_trajectory

    drone_frame = req.drone_body_frame_id
    tip_frame = req.tooltip_frame_id

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    tf = tfBuffer.lookup_transform(drone_frame, tip_frame, time=rospy.Time.now(), timeout=rospy.Duration(5))

    drone_position_offset_local = Transform()
    drone_position_offset_local.translation.x = -tf.transform.translation.x
    drone_position_offset_local.translation.y = -tf.transform.translation.y
    drone_position_offset_local.translation.z = -tf.transform.translation.z
    drone_position_offset_local.rotation.x = 0

    drone_position_offset_local.rotation.y = 0
    drone_position_offset_local.rotation.z = 0
    drone_position_offset_local.rotation.w = 1

    resp = droneTrajectoryResponse()
    resp.drone_trajectory = MultiDOFJointTrajectory()
    resp.drone_trajectory.header.frame_id = req.tip_traj.header.frame_id
    resp.drone_trajectory.header.stamp = rospy.Time.now()

    for i in range(len(tip_traj.points)):
        curr_tip_transform = Transform()
        curr_tip_transform = tip_traj.points[i].transforms[0]

        yaw_transform = Transform()
        #extract the yaw angle from the rotation setpoint
        (roll, pitch, yaw) = euler_from_quaternion([curr_tip_transform.rotation.x,
                                                    curr_tip_transform.rotation.y,
                                                    curr_tip_transform.rotation.z,
                                                    curr_tip_transform.rotation.w])
        yaw_q = quaternion_from_euler(0.0, 0.0, yaw)
        yaw_transform.rotation.x = yaw_q[0]
        yaw_transform.rotation.y = yaw_q[1]
        yaw_transform.rotation.z = yaw_q[2]
        yaw_transform.rotation.w = yaw_q[3]

        drone_position_offset_world = do_transform_transform(drone_position_offset_local, yaw_transform)

        drone_transform = do_transform_transform(curr_tip_transform, drone_position_offset_world)

        drone_trajectory_point = MultiDOFJointTrajectoryPoint()
        drone_trajectory_point.time_from_start = tip_traj.points[i].time_from_start
        drone_trajectory_point.transforms.append(drone_transform)
        drone_trajectory_point.velocities.append(tip_traj.points[i].velocities[0])
        drone_trajectory_point.accelerations.append(tip_traj.points[i].accelerations[0])
        resp.drone_trajectory.points.append(drone_trajectory_point)
    rospy.loginfo("Trajectory ready")
    return resp

def handle_transform_trajectory(req):
    # service takes a set of desired toolpath poses and transforms 
    # to the printing surface in the drone's frame of reference.

    frame_id_init = req.frame_id
    frame_id_new = req.transformed_frame_id
    poses = req.poses
    
    rospy.loginfo("Trajectory transform requested from frame_id=" + frame_id_init + " to frame_id=" + frame_id_new + ".")
    
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    tf = tfBuffer.lookup_transform(frame_id_init, frame_id_new, time=rospy.Time.now(), timeout=rospy.Duration(5))

    resp = transformTrajectoryResponse()

    resp.poses_transformed = PoseArray()
    resp.poses_transformed.header.frame_id = frame_id_new
    resp.poses_transformed.header.stamp = rospy.Time.now()

    for i in range(len(poses.poses)):
        pose = Pose()
        pose = poses.poses[i]
        pose_transformed = do_transform_pose(pose, tf)
        resp.poses_transformed.poses.append(pose_transformed)

    rospy.loginfo("Trajectory ready")
    return resp
    

def handle_TOPPRA_trajectory(req):
    # service generates a smooth, interpolated, time-optimal trajectory from an array of poses using TOPPRA

    num_poses = len(req.poses.poses)
    
    rospy.loginfo("Trajectory requested. Interpolating " + str(num_poses) + " poses at " + str(req.frequency) + "Hz.")

    way_pts = np.zeros((num_poses, 6))
    for i in range(num_poses):
        (roll, pitch, yaw) = euler_from_quaternion([req.poses.poses[i].orientation.x,
                                                    req.poses.poses[i].orientation.y,
                                                    req.poses.poses[i].orientation.z,
                                                    req.poses.poses[i].orientation.w])
        way_pts[i,:] = [req.poses.poses[i].position.x, req.poses.poses[i].position.y, req.poses.poses[i].position.z, roll, pitch, yaw]
    
    ss = np.linspace(0, 1, num_poses)
    
    amax = req.max_acc
    vmax = req.max_vel
    max_yawrate = np.deg2rad(req.max_yawrate)
    max_yawrate_dot = np.deg2rad(req.max_yawrate_dot)
    vlims = [vmax, vmax, vmax, max_yawrate, max_yawrate, max_yawrate]
    alims = [amax, amax, amax, max_yawrate_dot, max_yawrate_dot, max_yawrate_dot]
    
    path = ta.SplineInterpolator(ss, way_pts)
    pc_vel = constraint.JointVelocityConstraint(vlims)
    pc_acc = constraint.JointAccelerationConstraint(alims)
    instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
    traj = instance.compute_trajectory(0, 0)

    n_samples = int(traj.duration * req.frequency)
    ts_sample = np.linspace(0, traj.duration, n_samples)
    qs_sample = traj(ts_sample, 0) #position setpoints
    qds_sample = traj(ts_sample, 1) #velocity setpoints
    qdds_sample = traj(ts_sample, 2) #acceleration setpoints

    resp = TOPPRATrajectoryResponse()
    resp.trajectory = MultiDOFJointTrajectory()
    resp.trajectory.header.frame_id = req.poses.header.frame_id
    resp.trajectory.header.stamp = rospy.Time.now()

    for i in range(n_samples):
        trans = Transform()
        trans.translation.x = qs_sample[i,0]
        trans.translation.y = qs_sample[i,1]
        trans.translation.z = qs_sample[i,2]
        q = quaternion_from_euler(qs_sample[i,3], qs_sample[i,4], qs_sample[i,5])
        trans.rotation.x = q[0]
        trans.rotation.y = q[1]
        trans.rotation.z = q[2]
        trans.rotation.w = q[3]

        vel = Twist()
        vel.linear.x = qds_sample[i,0]
        vel.linear.y = qds_sample[i,1]
        vel.linear.z = qds_sample[i,2]
        vel.angular.x = qds_sample[i,3]
        vel.angular.y = qds_sample[i,4]
        vel.angular.z = qds_sample[i,5]

        accel = Twist()
        accel.linear.x = qdds_sample[i,0]
        accel.linear.y = qdds_sample[i,1]
        accel.linear.z = qdds_sample[i,2]
        accel.angular.x = qdds_sample[i,3]
        accel.angular.y = qdds_sample[i,4]
        accel.angular.z = qdds_sample[i,5]

        trajectory_point = MultiDOFJointTrajectoryPoint()
        trajectory_point.transforms.append(trans)
        trajectory_point.velocities.append(vel)
        trajectory_point.accelerations.append(accel)
        trajectory_point.time_from_start = rospy.Duration(i / req.frequency)

        resp.trajectory.points.append(trajectory_point)

    rospy.loginfo("Trajectory ready")
    return resp

def do_transform_transform(transform1, transform2):
    #if either input is not ...Stamped() version of message, do conversion:
    if isinstance(transform1, Transform):
        transform1 = TransformStamped(transform=transform1)
    if isinstance(transform2, Transform):
        transform2 = TransformStamped(transform=transform2)
    #check that type is now correct, if not output an error readable by humans:
    if isinstance(transform1, TransformStamped) and isinstance(transform2, TransformStamped):
        pose = PoseStamped()
        pose.pose.position = transform1.transform.translation
        pose.pose.orientation = transform1.transform.rotation
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform2)
        transform1_transformed = Transform()
        transform1_transformed.translation = pose_transformed.pose.position
        transform1_transformed.rotation = pose_transformed.pose.orientation
        return transform1_transformed
    else:
        rospy.logerr("Incorrect types used in do_transform_transform()")
        rospy.logerr("Must be Tranform() or TransformStamped()")


def do_transform_pose(pose, transform):
    #if either input is not ...Stamped() version of message, do conversion:
    if isinstance(pose, Pose):
        pose = PoseStamped(pose=pose)
    if isinstance(transform, Transform):
        transform = TransformStamped(transform=transform)
    #check that type is now correct, if not output an error readable by humans:
    if isinstance(pose, PoseStamped) and isinstance(transform, TransformStamped):
        posestamped_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
        pose_transformed = Pose()
        pose_transformed = posestamped_transformed.pose
        return pose_transformed
    else:
        rospy.logerr("Incorrect types used in do_transform_pose()")
        rospy.logerr("Must be Pose() or PoseStamped()")
    

def do_transform_twist(twist, transform):
    #if either input is not ...Stamped() version of message, do conversion:
    if isinstance(twist, Twist):
        twist = TwistStamped(twist=twist)
    if isinstance(transform, Transform):
        transform = TransformStamped(transform=transform)
    #check that type is now correct, if not output an error readable by humans:
    if isinstance(twist, TwistStamped) and isinstance(transform, TransformStamped):
        wrench = WrenchStamped()
        wrench.wrench.force = twist.twist.linear
        wrench.wrench.torque = twist.twist.angular
        wrench_transformed = tf2_geometry_msgs.do_transform_wrench(
            wrench, transform)
        twist_transformed = Twist()
        twist_transformed.linear = wrench_transformed.wrench.force
        twist_transformed.angular = wrench_transformed.wrench.torque
        return twist_transformed
    else:
        rospy.logerr("Incorrect types used in do_transform_twist()")
        rospy.logerr("Must be Twist()/TwistStamped() and Transform()/TransformStamped()")

def trajectory_server():
    rospy.init_node('trajectory_services')
    # services for trajectory generation
    TOPPRA_service = rospy.Service(
        'get_TOPPRA_trajectory', TOPPRATrajectory, handle_TOPPRA_trajectory)
    transform_trajectory_service = rospy.Service(
        'get_transformed_trajectory', transformTrajectory, handle_transform_trajectory)
    drone_trajectory_service = rospy.Service(
        'get_drone_trajectory', droneTrajectory, handle_drone_trajectory)
    fetch_poses_service = rospy.Service(
        'fetch_poses', fetchPoses, handle_fetch_poses)
    rospy.spin()

if __name__ == "__main__":
    trajectory_server()