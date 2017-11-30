/*
 * @copyright Copyright (C) 2017, Jessica Howard 
 * @author Jessica Howard
 * @file turtlebot_rrt/include/turtlebot_rrt/turtlebot_rrt.h
 *
 * @brief Algorithm to make the turtlebot move around a gazebo workspace
 * @detail The turtlebot will move forward in a straight line until
 * encountering an obstacle. The turtlebot will then rotate until a clear
 * path is available and again drive forward. Rinse and repeat until you
 * get bored of watching.
 *
 * 
 * 
 * @license 3-Clause BSD License
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/** include standard libraries **/
#include <iostream>
#include <cmath>
#include <set>
#include <string>
#include <vector>

/** include ROS libraries **/
#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

/** for global path planner interface **/
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#ifndef SRC_TURTLEBOT_RRT_H_
#define SRC_TURTLEBOT_RRT_H_

namespace RRT_Planner {

    /**
     * @struct Vertex
     * @brief A struct that holds a vertex's location and parent
     */
    struct Vertex {
        float x;
        float y;
        int parent_index;
    };

    class RRTPlanner : public nav_core::BaseGlobalPlanner {
    public:
        /**
         * @brief Constructor for RRTPlanner
         * @param costmap_ros cost_map ros wrapper
         */
        RRTPlanner() : costmap_ros_(nullptr), initialized_(false) {
            
        }
        /**
         * @brief Constructor for RRTPlanner
         * @param costmap_ros cost_map ros wrapper
         * @param name name to associate to the node
         */
        RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
        costmap_ros_(costmap_ros) {
            initialize(name, costmap_ros);
        }


        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        /**
         * @brief Initialize the ros handle
         * @param name ROS NodeHandle name
         * @param costmap_ros cost_map ros wrapper
         */
        void initialize(std::string name, 
            costmap_2d::Costmap2DROS* costmap_ros);
        
        /**
         * @brief follows the virtual method of the base class
         * @param start start pose
         * @param goal goal pose
         * @param plan generated path
         * @return bool, true
         */
        bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
        
        /**
         * @brief follows the virtual method of the base class
         * @param start start pose
         * @param goal goal pose
         * @param plan generated path
         * @return bool, true
         */
        bool generatePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
        
    private:
        ros::NodeHandle node_handle_;
        costmap_2d::Costmap2DROS* costmap_ros_;
        costmap_2d::Costmap2D* costmap_;
        base_local_planner::WorldModel* world_model_;
        bool initialized_;
        float goal_radius_;
        float step_size_;
        float delta_;
        float x_origin_;
        float y_origin_;
        float resolution_;
        int map_width_;
        int map_height_;
        std::vector<Vertex> vertex_list_;
        std::pair<float, float> get_random_point();
        int get_closest_vertex(std::pair<float, float> random_point);
        bool move_towards_point(int closest_vertex, std::pair<float, float> random_point);
        float get_distance(std::pair<float, float> start_point, std::pair<float, float> end_point);
        bool reached_goal(std::pair<float, float> new_vertex);
    };
}


#endif // SRC_TURTLEBOT_RRT_H_
