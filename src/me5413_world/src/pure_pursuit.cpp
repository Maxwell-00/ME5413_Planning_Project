/**
 * @file pure_pursuit.cpp
 * @brief Implementation of Pure Pursuit Controller ROS Node
 *
 * This file contains the implementation of the PurePursuitController class,
 * which defines a ROS node responsible for controlling a robot to track a given path using the Pure Pursuit algorithm.
 *
 * @author Shuo SUN & Advanced Robotics Center, National University of Singapore
 * @date 2024
 * @license MIT License
 */

#include "pure_pursuit.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

namespace {

    constexpr double kPI = 3.14159265358979323846;

}  // namespace

PurePursuitController::PurePursuitController(double lookahead_distance)
        : lookahead_distance_(lookahead_distance) {}

void PurePursuitController::setLookaheadDistance(double lookahead_distance) {
    lookahead_distance_ = lookahead_distance;
}

geometry_msgs::Twist PurePursuitController::computeControlOutputs(
        const nav_msgs::Odometry& odom_robot,
        const std::vector<geometry_msgs::PoseStamped>& path) {
    geometry_msgs::Twist cmd_vel;

    // Get current robot position and orientation
    geometry_msgs::Pose robot_pose = odom_robot.pose.pose;
    tf2::Vector3 robot_position(robot_pose.position.x, robot_pose.position.y,
                                robot_pose.position.z);
    tf2::Quaternion robot_orientation;
    tf2::fromMsg(robot_pose.orientation, robot_orientation);
    double robot_yaw = robot_orientation.getAngle();

    // Find the closest point on the path
    double min_distance = std::numeric_limits<double>::max();
    int closest_index = -1;
    for (size_t i = 0; i < path.size(); ++i) {
        tf2::Vector3 path_point(path[i].pose.position.x, path[i].pose.position.y,
                                path[i].pose.position.z);
        double distance = tf2::tf2Distance(robot_position, path_point);
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = static_cast<int>(i);
        }
    }

    // Find the point on the path that is the lookahead distance away
    int lookahead_index = closest_index;
    while (lookahead_index < static_cast<int>(path.size()) - 1) {
        tf2::Vector3 path_point(path[lookahead_index].pose.position.x,
                                path[lookahead_index].pose.position.y,
                                path[lookahead_index].pose.position.z);
        double distance = tf2::tf2Distance(robot_position, path_point);
        if (distance >= lookahead_distance_) {
            break;
        }
        lookahead_index++;
    }

    // Compute the steering angle
    if (lookahead_index < static_cast<int>(path.size())) {
        tf2::Vector3 lookahead_point(path[lookahead_index].pose.position.x,
                                     path[lookahead_index].pose.position.y,
                                     path[lookahead_index].pose.position.z);
        tf2::Vector3 goal_vector = lookahead_point - robot_position;
        double desired_yaw = std::atan2(goal_vector.getY(), goal_vector.getX());
        double heading_error = desired_yaw - robot_yaw;

        // Ensure heading error is within [-pi, pi] range
        if (heading_error > kPI) {
            heading_error -= 2 * kPI;
        } else if (heading_error < -kPI) {
            heading_error += 2 * kPI;
        }

        cmd_vel.angular.z = heading_error;  // Adjust angular velocity based on
        // heading error
        cmd_vel.linear.x = 1.0;  // Set linear velocity to a constant value for
        // simplicity
    }

    return cmd_vel;
}
