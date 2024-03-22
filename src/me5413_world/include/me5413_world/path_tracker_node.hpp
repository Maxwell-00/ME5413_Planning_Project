/**
 * @file path_tracker_node.hpp
 * @brief Declarations for PathTrackerNode class
 *
 * This file contains the declarations of the PathTrackerNode class, which implements
 * a ROS node responsible for controlling a robot to track a given path.
 * It includes necessary header files and ROS declarations.
 *
 * @author Shuo SUN & Advanced Robotics Center, National University of Singapore
 * @date 2024
 * @license MIT License
 */

#ifndef PATH_TRACKER_NODE_H_
#define PATH_TRACKER_NODE_H_

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <me5413_world/path_trackerConfig.h>

#include "pure_pursuit.hpp" // Include the pure pursuit controller header

namespace me5413_world
{

    /**
     * @class PathTrackerNode
     * @brief Class responsible for controlling a robot to track a given path
     */
    class PathTrackerNode
    {
    public:
        /**
         * @brief Default constructor
         */
        PathTrackerNode();

        /**
         * @brief Destructor
         */
        virtual ~PathTrackerNode() {};

    private:
        /**
         * @brief Callback function for robot odometry data
         * @param odom The received odometry message
         */
        void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);

        /**
         * @brief Callback function for goal pose data
         * @param goal_pose The received goal pose message
         */
        void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose);

        /**
         * @brief Callback function for local path data
         * @param path The received local path message
         */
        void localPathCallback(const nav_msgs::Path::ConstPtr& path);

        /**
         * @brief Compute control outputs based on the provided odometry data and path
         * @param odom_robot The odometry data of the robot
         * @param path The local path to track
         * @return The computed control outputs (Twist message)
         */
        geometry_msgs::Twist computeControlOutputs(const nav_msgs::Odometry& odom_robot, const std::vector<geometry_msgs::PoseStamped>& path);

        /**
         * @brief Convert a pose to a transform
         * @param pose The pose to convert
         * @return The corresponding transform
         */
        tf2::Transform convertPoseToTransform(const geometry_msgs::Pose& pose);

        // ROS declaration
        ros::NodeHandle nh_;
        ros::Timer timer_;
        ros::Subscriber sub_robot_odom_;
        ros::Subscriber sub_local_path_;
        ros::Publisher pub_cmd_vel_;

        // Robot pose
        std::string world_frame_;
        std::string robot_frame_;
        nav_msgs::Odometry odom_world_robot_;

        // Pure pursuit controller
        PurePursuitController pure_pursuit_controller_;

        // Lookahead distance and local path
        double lookahead_distance_;
        std::vector<geometry_msgs::PoseStamped> local_path_;

        // Dynamic reconfigure callback function
        void dynamicReconfigureCallback(me5413_world::path_trackerConfig &config, uint32_t level)
        {
            lookahead_distance_ = config.lookahead_distance;
            pure_pursuit_controller_.setLookaheadDistance(lookahead_distance_);
        }

        // Dynamic reconfigure server
        dynamic_reconfigure::Server<me5413_world::path_trackerConfig> server_;
        dynamic_reconfigure::Server<me5413_world::path_trackerConfig>::CallbackType f_;
    };

} // namespace me5413_world

#endif // PATH_TRACKER_NODE_H_
