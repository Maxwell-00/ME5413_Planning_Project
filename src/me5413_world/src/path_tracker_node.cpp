/**
 * @file path_tracker_node.cpp
 * @brief Implementation of the PathTrackerNode class
 *
 * This file contains the implementation of the PathTrackerNode class, which defines
 * a ROS node responsible for controlling a robot to track a given path.
 * It includes methods for subscribing to robot odometry and local path topics, computing control outputs,
 * and publishing velocity commands.
 *
 * @author Shuo SUN & Advanced Robotics Center, National University of Singapore
 * @date 2024
 * @license MIT License
 */

#include "me5413_world/path_tracker_node.hpp"

namespace me5413_world
{

    PathTrackerNode::PathTrackerNode() : nh_("~"), pure_pursuit_controller_(1.0)
    {
        nh_.param("lookahead_distance", lookahead_distance_, 1.0);
        pure_pursuit_controller_.setLookaheadDistance(lookahead_distance_);

        sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
        sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);
    };

    void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
    {
        local_path_ = path->poses;

        // Update lookahead distance in the PurePursuitController
        pure_pursuit_controller_.setLookaheadDistance(lookahead_distance_);

        // Compute control outputs using pure pursuit algorithm
        geometry_msgs::Twist cmd_vel = computeControlOutputs(odom_world_robot_, local_path_);

        // Publish control command
        pub_cmd_vel_.publish(cmd_vel);
    }

    void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
    {
        odom_world_robot_ = *odom;
    }

    geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const std::vector<geometry_msgs::PoseStamped>& path)
    {
        geometry_msgs::Twist cmd_vel;

        if (path.empty())
        {
            ROS_WARN("Local path is empty, cannot compute control outputs");
            return cmd_vel;
        }

        // Get current robot position and orientation
        geometry_msgs::Pose robot_pose = odom_robot.pose.pose;
        tf2::Vector3 robot_position(robot_pose.position.x, robot_pose.position.y, robot_pose.position.z);
        tf2::Quaternion robot_orientation;
        tf2::fromMsg(robot_pose.orientation, robot_orientation);

        // Find the nearest point in the path
        int nearest_idx = 0;
        double min_distance = std::numeric_limits<double>::max();
        for (int i = 0; i < path.size(); ++i)
        {
            double distance = tf2::tf2Distance(robot_position, tf2::Vector3(path[i].pose.position.x, path[i].pose.position.y, path[i].pose.position.z));
            if (distance < min_distance)
            {
                nearest_idx = i;
                min_distance = distance;
            }
        }

        // Find the lookahead point
        int lookahead_idx = nearest_idx;
        double total_distance = 0;
        while (total_distance < lookahead_distance_ && lookahead_idx < path.size() - 1)
        {
            total_distance += tf2::tf2Distance(tf2::Vector3(path[lookahead_idx].pose.position.x, path[lookahead_idx].pose.position.y, path[lookahead_idx].pose.position.z),
                                               tf2::Vector3(path[lookahead_idx + 1].pose.position.x, path[lookahead_idx + 1].pose.position.y, path[lookahead_idx + 1].pose.position.z));
            lookahead_idx++;
        }

        // Compute control outputs using pure pursuit algorithm
        if (lookahead_idx < path.size())
        {
            double goal_x = path[lookahead_idx].pose.position.x;
            double goal_y = path[lookahead_idx].pose.position.y;

            // Transform the goal point to the robot frame
            tf2::Vector3 goal_in_robot_frame = tf2::quatRotate(robot_orientation.inverse(), tf2::Vector3(goal_x - robot_position.x(), goal_y - robot_position.y(), 0));

            // Calculate the steering angle
            double steering_angle = atan2(goal_in_robot_frame.y(), goal_in_robot_frame.x());

            // Apply the steering angle to the control command
            cmd_vel.angular.z = steering_angle;
            cmd_vel.linear.x = 1.0; // Set linear velocity to a constant value for simplicity
        }

        return cmd_vel;
    }

} // namespace me5413_world

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_tracker_node");
    me5413_world::PathTrackerNode path_tracker_node;
    ros::spin(); // spin the ros node.
    return 0;
}
