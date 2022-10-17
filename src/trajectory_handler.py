#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
from tricopter.srv import *

class trajectoryHandler:
    def __init__(self):
        self.frequency = 1
        self.max_vel = 0
        self.max_acc = 0
        self.max_yawrate = 0
        self.max_yawrate_dot = 0
        
        self.world_frame_id = "map"
        self.drone_frame_id = "base_link"
        self.tooltip_frame_id = "tooltip_init"
        self.print_frame_id = "printing_plane"
        self.waypoint_prefix = "waypoints"

        self._point_count = 0      

    def follow(self, trajectory):
        if self._point_count < len(trajectory.points):
            pose = PoseStamped()
            velocity = TwistStamped()
            acceleration = TwistStamped() 

            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = trajectory.header.frame_id
            velocity.header = pose.header
            acceleration.header = pose.header

            pose.pose.position = trajectory.points[self._point_count].transforms[0].translation
            pose.pose.orientation = trajectory.points[self._point_count].transforms[0].rotation
            velocity.twist = trajectory.points[self._point_count].velocities[0]
            acceleration.twist = trajectory.points[self._point_count].accelerations[0]
            self._point_count += 1
            complete = False
        else:
            pose = PoseStamped()
            velocity = TwistStamped()
            acceleration = TwistStamped()
            complete = True
            self._point_count = 0

        return pose, velocity, acceleration, complete

    def generate_transition(self, start_pose, end_pose):
        if start_pose.header.frame_id != end_pose.header.frame_id:
            rospy.logerr("Cannot interpolate between poses in different reference frames.")
        else:
            # mid_pose_1 = Pose()
            # mid_pose_1.position.x = start_pose.pose.position.x
            # mid_pose_1.position.y = start_pose.pose.position.y
            # mid_pose_1.position.z = end_pose.pose.position.z
            # mid_pose_1.orientation = start_pose.pose.orientation

            poses = PoseArray()
            poses.header.stamp = rospy.Time.now()
            poses.header.frame_id = start_pose.header.frame_id
            poses.poses.append(start_pose.pose)
            # poses.poses.append(mid_pose_1)
            poses.poses.append(end_pose.pose)

            trajectory = self._TOPPRA_interpolation(poses)
            return trajectory

    def generate_print_layer(self, layer_number):
        print_waypoints = self._fetch_waypoints_from_yaml(layer_number)
        print_waypoints_transformed = self._transform_trajectory(print_waypoints)
        tooltip_trajectory = self._TOPPRA_interpolation(print_waypoints_transformed)
        drone_trajectory = self._offset_drone_trajectory(tooltip_trajectory)
        return drone_trajectory, tooltip_trajectory

    def get_print_start_pose(self, trajectory):
        pose = PoseStamped()
        pose.header = trajectory.header
        pose.pose.position = trajectory.points[0].transforms[0].translation
        pose.pose.orientation = trajectory.points[0].transforms[0].rotation
        return pose

    def _fetch_waypoints_from_yaml(self, layer_number):
        # get poses from file
        rospy.wait_for_service('fetch_poses')
        get_poses = rospy.ServiceProxy('fetch_poses', fetchPoses)
        request = fetchPosesRequest()
        request.prefix = self.waypoint_prefix
        request.frame_id = self.print_frame_id
        request.layer_number = layer_number
        response = get_poses(request)
        return response.poses

    def _transform_trajectory(self, poses):
        #transform to world coordinates system
        rospy.wait_for_service('get_transformed_trajectory')
        transform_poses = rospy.ServiceProxy('get_transformed_trajectory', transformTrajectory)
        request = transformTrajectoryRequest()
        request.poses = poses
        request.transformed_frame_id = self.world_frame_id
        response = transform_poses(request)
        return response.poses_transformed

    def _TOPPRA_interpolation(self, poses):
        #interpolate with TOPPRA
        rospy.wait_for_service('get_TOPPRA_trajectory')
        get_traj = rospy.ServiceProxy('get_TOPPRA_trajectory', TOPPRATrajectory)
        request = TOPPRATrajectoryRequest()
        request.frequency = self.frequency
        request.max_vel = self.max_vel
        request.max_acc = self.max_acc
        request.max_yawrate = self.max_yawrate
        request.max_yawrate_dot = self.max_yawrate_dot
        request.poses = poses
        response = get_traj(request)
        return response.trajectory

    def _offset_drone_trajectory(self, trajectory):
        #get offset drone trajectory
        rospy.wait_for_service('get_drone_trajectory')
        get_traj = rospy.ServiceProxy('get_drone_trajectory', droneTrajectory)
        request = droneTrajectoryRequest()
        request.drone_body_frame_id = self.drone_frame_id
        request.tooltip_frame_id = self.tooltip_frame_id
        request.toolpath_trajectory = trajectory
        response = get_traj(request)
        return response.drone_trajectory