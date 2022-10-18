#! /usr/bin/env python3

from ast import Mult
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
from nav_msgs.msg import Path
from tricopter.srv import *
from viz_functions import *

class trajectoryHandler:
    def __init__(self, frequency, max_vel, max_acc, max_yawrate, max_yawrate_dot):
        self.frequency = frequency
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.max_yawrate = max_yawrate
        self.max_yawrate_dot = max_yawrate_dot
        
        self.world_frame_id = "map"
        self.drone_frame_id = "base_link"
        self.tooltip_frame_id = "tooltip_init"
        self.print_frame_id = "printing_plane"
        self.waypoint_prefix = "waypoints"

        self._point_count = 0    
        self._drone_trajectory = MultiDOFJointTrajectory()
        self._tooltip_trajectory = MultiDOFJointTrajectory()
        self._transition_trajectory = MultiDOFJointTrajectory()

        # publishers for visualisation only
        self._pub_toolpath_viz = rospy.Publisher('/viz/toolpath', Path, queue_size=1)
        self._pub_dronepath_viz = rospy.Publisher('/viz/dronepath', Path, queue_size=1)
        self._pub_transitionpath_viz = rospy.Publisher('/viz/transitionpath', Path, queue_size=1)
        self._pub_print_viz = rospy.Publisher('/viz/print', Path, queue_size=1) 

        # publish print when object is instantiated
        publish_viz_print(self._pub_print_viz)

    def follow_print_trajectory(self):
        drone_pose = PoseStamped()
        drone_velocity = TwistStamped()
        drone_acceleration = TwistStamped()
        
        tooltip_pose = PoseStamped()
        tooltip_velocity = TwistStamped()
        tooltip_acceleration = TwistStamped()

        if self._point_count < len(self._drone_trajectory.points):   
            drone_pose, drone_velocity, drone_acceleration = self._read_trajectory(self._drone_trajectory, self._point_count)
            tooltip_pose, tooltip_velocity, tooltip_acceleration = self._read_trajectory(self._tooltip_trajectory, self._point_count)
            self._point_count += 1
            complete = False
        else: 
            complete = True
            self._point_count = 0
            self._drone_trajectory = MultiDOFJointTrajectory() #reset trajectory so it can't be repeated by accident
            self._tooltip_trajectory = MultiDOFJointTrajectory()
            
        return drone_pose, drone_velocity, drone_acceleration, tooltip_pose, tooltip_velocity, tooltip_acceleration, complete

    def follow_transition_trajectory(self):
        drone_pose = PoseStamped()
        drone_velocity = TwistStamped()
        drone_acceleration = TwistStamped()
        if self._point_count < len(self._transition_trajectory.points):   
            drone_pose, drone_velocity, drone_acceleration = self._read_trajectory(self._transition_trajectory, self._point_count)
            self._point_count += 1
            complete = False
        else: 
            complete = True
            self._point_count = 0
            self._transition_trajectory = MultiDOFJointTrajectory() #reset trajectory so it can't be repeated by accident
        return drone_pose, drone_velocity, drone_acceleration, complete

    def get_print_start_pose(self):
        pose = PoseStamped()
        pose.header = self._drone_trajectory.header
        pose.pose.position = self._drone_trajectory.points[0].transforms[0].translation
        pose.pose.orientation = self._drone_trajectory.points[0].transforms[0].rotation
        return pose

    def generate_transition(self, start_pose, end_pose):
        if start_pose.header.frame_id != end_pose.header.frame_id:
            rospy.logerr("Cannot interpolate between poses in different reference frames.")
        else:
            poses = PoseArray()
            poses.header.stamp = rospy.Time.now()
            poses.header.frame_id = start_pose.header.frame_id
            poses.poses.append(start_pose.pose)
            poses.poses.append(end_pose.pose)
            self._transition_trajectory = self._TOPPRA_interpolation(poses)
            publish_viz_trajectory(self._transition_trajectory, self._pub_transitionpath_viz)
            # return trajectory

    def generate_print_layer(self, layer_number):
        print_waypoints = self._fetch_waypoints_from_yaml(layer_number)
        print_waypoints_transformed = self._transform_trajectory(print_waypoints)
        self._tooltip_trajectory = self._TOPPRA_interpolation(print_waypoints_transformed)
        self._drone_trajectory = self._offset_drone_trajectory(self._tooltip_trajectory)
        publish_viz_trajectory(self._drone_trajectory, self._pub_dronepath_viz)
        publish_viz_trajectory(self._tooltip_trajectory, self._pub_toolpath_viz)
        # return drone_trajectory, tooltip_trajectory

    def _read_trajectory(self, trajectory, point_num):
        pose = PoseStamped()
        velocity = TwistStamped()
        acceleration = TwistStamped() 
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = trajectory.header.frame_id
        velocity.header = pose.header
        acceleration.header = pose.header
        pose.pose.position = trajectory.points[point_num].transforms[0].translation
        pose.pose.orientation = trajectory.points[point_num].transforms[0].rotation
        velocity.twist = trajectory.points[point_num].velocities[0]
        acceleration.twist = trajectory.points[point_num].accelerations[0]
        return pose, velocity, acceleration

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