/**
 * @file pure_pursuit.hpp
 * @brief Declaration of Pure Pursuit Controller
 *
 * This file contains the declaration of the PurePursuitController class,
 * which implements a Pure Pursuit controller for path tracking.
 *
 * @author Shuo SUN & Advanced Robotics Center, National University of Singapore
 * @date 2024
 * @license MIT License
 */
#ifndef PURE_PURSUIT_H_
#define PURE_PURSUIT_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

/**
 * @class PurePursuitController
 * @brief Implements a Pure Pursuit controller for path tracking
 */
class PurePursuitController {
public:
    /**
     * @brief Constructor with lookahead distance parameter
     * @param lookahead_distance The lookahead distance for the controller
     */
    PurePursuitController(double lookahead_distance);

    /**
     * @brief Set lookahead distance
     * @param lookahead_distance The lookahead distance to set
     */
    void setLookaheadDistance(double lookahead_distance);

    /**
     * @brief Compute control outputs based on current robot odometry and path
     * @param odom_robot The current robot odometry
     * @param path The path to track
     * @return The computed control outputs
     */
    geometry_msgs::Twist computeControlOutputs(const nav_msgs::Odometry& odom_robot, const std::vector<geometry_msgs::PoseStamped>& path);

private:
    double lookahead_distance_; ///< Lookahead distance for the controller
};

#endif // PURE_PURSUIT_H_
