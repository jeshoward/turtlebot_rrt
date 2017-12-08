/*
 * @copyright Copyright (C) 2017, Jessica Howard
 * @author Jessica Howard
 * @file turtlebot_rrt/src/turtlebot_rrt.cc
 *
 * @brief TODO
 * @detail TODO
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

#include "turtlebot_rrt/turtlebot_rrt.h"
#include "turtlebot_rrt/vertex.h"
#include <pluginlib/class_list_macros.h>

namespace turtlebot_rrt {
  void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
      // Initialize map
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros->getCostmap();
      
      x_origin_ = costmap_->getOriginX();
      y_origin_ = costmap_->getOriginY();
      
      map_width_ = costmap_->getSizeInCellsX();
      map_height_ = costmap_->getSizeInCellsY();
      resolution_ = costmap_->getResolution();
      
      world_model_ = new base_local_planner::CostmapModel(*costmap_);
      
      // Initialize node handle
      ros::NodeHandle node_handle("~/" + name);
      
      // TODO do node handle stuff here
      
      // Initialize root node
      turtlebot_rrt::Vertex root(x_origin_, y_origin_, 0, 0);
      vertex_list_.push_back(root);
      
      // TODO Initialize parameters
		node_handle.getParam("delta_", this->delta_);
		ROS_INFO("delta_=%.2f", delta_);
      
      // Display info message
      ROS_INFO("RRT planner initialized successfully.");
      initialized_ = true;
    } else {
      ROS_WARN("RRT planner has already been initialized.");
    }
  }
  
  bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    // Check if we've initialized, if not error
    if (!initialized_) {
      ROS_ERROR("RRT planner has not been initialized, please call "
                "initialize() to use the planner");
      return false;
    }
    // Print some debugging info
    ROS_DEBUG("Start: %.2f, %.2f", start.pose.position.x,
              start.pose.position.y);
    ROS_DEBUG("Goal: %.2f, %.2f", goal.pose.position.x,
              goal.pose.position.y);
    
    plan.clear();
    
    // Make sure that the goal header frame is correct
    // Goals are set within rviz
    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
      ROS_ERROR("This planner will only accept goals in the %s frame,"
                "the goal was sent to the %s frame.",
                costmap_ros_->getGlobalFrameID().c_str(),
                goal.header.frame_id.c_str());
      return false;
    }
    
    RRTPlanner::x_goal_ = goal.pose.position.x;
    RRTPlanner::y_goal_ = goal.pose.position.y;
    
    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;
    
    poseStampedMsgToTF(goal, goal_tf);
    poseStampedMsgToTF(start, start_tf);
    
    //float start_x = start.pose.position.x;
    //float start_y = start.pose.position.y;
    
    //float goal_x = goal.pose.position.x;
    //float goal_y = goal.pose.position.y;
    
    // Get the starting and goal yaws (pitch and roll are useless here)
    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);
    
    // TODO kick off the RRT algorithm
    
    return false;
  }
  
  std::vector<turtlebot_rrt::Vertex> RRTPlanner::find_path(std::pair<float, float> start, std::pair<float, float> goal) {
    std::vector<turtlebot_rrt::Vertex> best_path;
    
    return best_path;
  }
  
  int RRTPlanner::get_closest_vertex(std::pair<float, float> random_point) {
    // Set our closest Vertex index to our root, since we know that it exists
    int closest = 0;
    
    // closest_distance will keep track of the closest distance we find
    float closest_distance = INFINITY;
    
    // current_distance will keep track of the distance of the current
    float current_distance = INFINITY;
    
    // iterate through the vertex list to find the closest
    for (auto it = begin(vertex_list_); it != end(vertex_list_); ++it) {
      current_distance = RRTPlanner::get_distance(it->get_location(), random_point);
      
      // If the current distance is closer than what was previously
      // saved, update
      if (current_distance < closest_distance) {
        closest = it->get_index();
        closest_distance = current_distance;
      }
    }
    return closest;
  }
  
  float RRTPlanner::get_distance(std::pair<float, float> start_point, std::pair<float, float> end_point) {
    // coords for our first point
    int x1 = start_point.first;
    int y1 = start_point.second;
    
    // coords for our second point
    int x2 = end_point.first;
    int y2 = end_point.second;
    
    // euclidean distance
    return sqrt((x1 - x2)*(x1 - x2)+(y1 - y2)*(y1 - y2));
  }
  
  bool RRTPlanner::move_towards_point(int closest_vertex, std::pair<float, float> random_point) {
    // Convert location of our closest point to a pair
    std::pair<float, float> closest_point =
      RRTPlanner::vertex_list_.at(closest_vertex).get_location();
    
    // Find out what direction we need to move
    float theta = atan2(random_point.second - closest_point.second,
                        random_point.first - closest_point.first);
    
    // Calculate the proposed new position
    float new_x = closest_point.first + RRTPlanner::step_size_ * cos(theta);
    float new_y = closest_point.second + RRTPlanner::step_size_ * sin(theta);
    std::pair<float, float> proposed_point(new_x, new_y);
    
    // Check if the path between closest_vertex and the new point
    // is safe
    if (is_safe(proposed_point, closest_point)) {
      // If safe, add new Vertex to the back of vertex_list_
      turtlebot_rrt::Vertex new_vertex(new_x, new_y, vertex_list_.size(), closest_vertex);
      vertex_list_.push_back(new_vertex);
      
      // Return true, that we moved towards the proposed point
      return true;
    }
    // Return false, move not made
    return false;
  }
  
  bool RRTPlanner::is_safe(std::pair<float, float> start_point, std::pair<float, float> end_point) {
    // Check to see that our end point is within map bounds
    if (end_point.first < 0 || end_point.first > RRTPlanner::map_width_ ||
        end_point.second < 0 || end_point.second > RRTPlanner::map_height_)
      return false;
    
    // Check to make sure our end point isn't inside an obstacle
    // TODO
    
    // Check the path at intervals of delta for collision
    float theta = atan2(end_point.second - start_point.second,
                        end_point.first - start_point.first);
    float current_x = start_point.first;
    float current_y = start_point.second;
    
    while (current_x < end_point.first && current_y < end_point.second) {
      current_x += RRTPlanner::delta_ * cos(theta);
      current_y += RRTPlanner::delta_ * sin(theta);
      
      // TODO Check for collision
    }
    return true;
  }
  
  bool RRTPlanner::reached_goal(int new_vertex) {
    // TODO get goal location from cost map or parameters
    std::pair<float, float> goal(RRTPlanner::x_goal_, RRTPlanner::y_goal_);
    std::pair<float, float> current_location(RRTPlanner::vertex_list_.at(new_vertex).get_location());
    
    // Check distance between current point and goal, if distance is less
    // than goal_radius_ return true, otherwise return false
    float distance = RRTPlanner::get_distance(current_location, goal);
    
    if (distance <= RRTPlanner::goal_radius_)
      return true;
    else
      return false;
  }
}

// Register as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(turtlebot_rrt::RRTPlanner, nav_core::BaseGlobalPlanner)
