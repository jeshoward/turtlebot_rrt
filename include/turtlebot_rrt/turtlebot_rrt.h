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

#ifndef SRC_TURTLEBOT_RRT_H_
#define SRC_TURTLEBOT_RRT_H_

/** include standard libraries **/
#include <iostream>
#include <cmath>
#include <set>
#include <string>
#include <vector>
#include <boost/random.hpp>

/** include ROS libraries **/
#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/** for global path planner interface **/
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

/** Local includes **/
#include "turtlebot_rrt/vertex.h"

namespace turtlebot_rrt {
  
  class RRTPlanner : public nav_core::BaseGlobalPlanner {
    public:
    
    /**
    * @brief Constructor for RRTPlanner
    */
    RRTPlanner();
    
    /**
    * @brief Constructor for RRTPlanner
    * @param costmap_ros cost_map ros wrapper
    * @param name name to associate to the node
    */
    RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    
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

    
    
    private:
    /**
    * @brief ROS node handle
    */
    ros::NodeHandle node_handle_;
    
    /**
    * @brief publishes the RRT structure
    */
    ros::Publisher pub_tree_;
    
    /**
    * @brief obstacles
    */
    std::vector<bool> obstacle_map_;
    
    /**
    * @brief The ROS wrapper for the costmap the controller will use
    */
    costmap_2d::Costmap2DROS* costmap_ros_;
    
    /**
    * @brief the max number of iterations to try and find a path
    */
    int max_iterations_;
    
    /**
    * @brief the current number of iterations
    */
    int current_iterations_;
    
    /**
    * @brief The ROS wrapper for the costmap the controller will use
    */
    costmap_2d::Costmap2D* costmap_;
    
    /**
    * @brief World model associated to the costmap
    */
    base_local_planner::WorldModel* world_model_;
    
    /**
    * @brief Check if the global planner is initialized
    */
    bool initialized_;
    
    /**
    * @brief How close to the goal is close enough
    */
    float goal_radius_;
    
    /**
    * @brief Size of the step the RRT planner takes
    */
    float step_size_;
    
    /**
    * @brief Size of the sub-step used for collision checking
    */
    float delta_;
    
    /**
    * @brief x coordinate of robot origin
    */
    float x_origin_;

    /**
    * @brief y coordinate of robot origin
    */
    float y_origin_;
    
    /**
    * @brief x coordinate of goal
    */
    float x_goal_;
    
    /**
    * @brief y coordinate of goal
    */
    float y_goal_;
    
    /**
    * @brief Width of the 2D map
    */
    int map_width_;
    
    /**
    * @brief Height of the 2D map
    */
    int map_height_;
    
    /**
    * @brief width of 2d map in cells
    */
    unsigned int map_width_cells_;
    
    /**
    * @brief height of 2d map in cells
    */
    unsigned int map_height_cells_;
    
    /**
    * @brief List of vertices
    */
    std::vector<turtlebot_rrt::Vertex> vertex_list_;
    
    /**
    * @brief Gets a random point in the map space
    * @return returns an x,y pair
    */
    std::pair<float, float> GetRandomPoint();
    
    /**
    * @brief Gets the closest vertex to the given point
    * @param A point in the map space
    * @return the index of the closest vertex to the given point
    */
    int GetClosestVertex(std::pair<float, float> random_point);
    
    /**
    * @brief Moves from the closest vertex towards the random point
    * @detail Begins at the closest point and attempts to move step_size_
    * towards the random point. Each step along the way at delta_ intervals
    * is checked for obstacles. If an obstacle is encountered the function
    * returns false. If it makes it from the closest vertex to step_size_
    * towards the random point a new vertex is created and added to
    * vertex_list_ and the function returns true.
    * @return true if a move was made, false if blocked by obstacle
    */
    bool MoveTowardsPoint(int closest_vertex,
                            std::pair<float, float> random_point);
    
    /**
    * @brief Euclidean distance between two points
    * @param start_point starting point
    * @param end_point ending point
    * @return euclidean distance between the points
    */
    float GetDistance(std::pair<float, float> start_point,
                       std::pair<float, float> end_point);
    
    /**
    * @brief Is vertex within goal_radius_ of the goal
    * @param the vertex to be checked
    * @return true if within goal_radius_
    */
    bool ReachedGoal(int new_vertex);
    
    /**
    * @brief returns the best path
    * @param start starting point of robot
    * @param goal goal point
    * @return returns the index of the point that reaches the goal
    */
    int FindPath(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal);
    
    /**
    * @brief Checks if the path is safe between start_point and end_point
    * @param start_point starting point location
    * @param end_point ending point location
    * @return true if path between points does not intersect obstacles
    */
    bool IsSafe(std::pair<float, float> start_point, 
                std::pair<float, float> end_point);
    
    /**
    * @brief builds the plan from vertices and returns in PoseStamped
    * @param goal_index the index of the vertex that has reached the goal
    * @param start the starting location of the robot as passed to makePlan
    * @param goal the goal location of the robot as passed to makePlan
    * @return a vector of geometry_msgs:PoseStamped from the start to the goal
    */
    std::vector<geometry_msgs::PoseStamped>
      BuildPlan(int goal_index,
                 const geometry_msgs::PoseStamped& start, 
                 const geometry_msgs::PoseStamped& goal);
    };
}
#endif // SRC_TURTLEBOT_RRT_H_
