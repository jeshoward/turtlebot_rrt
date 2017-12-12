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
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the 
 *     names of its contributors may be used to endorse or promote products 
 *     derived from this software without specific prior written permission.
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

// Register as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(turtlebot_rrt::RRTPlanner, nav_core::BaseGlobalPlanner)

namespace turtlebot_rrt {
  
  RRTPlanner::RRTPlanner() 
    : costmap_ros_(nullptr), initialized_(false) { }
  
  RRTPlanner::RRTPlanner(std::string name,
                         costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(costmap_ros) {
    initialize(name, costmap_ros);
  }
  
  void RRTPlanner::initialize(std::string name, 
                              costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
      // Initialize map
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros->getCostmap();

      // Initialize node handle
      ros::NodeHandle node("~/turtlebot_rrt");
      node_handle_ = node;
      world_model_ = new base_local_planner::CostmapModel(*costmap_);
      
      map_width_ = costmap_->getSizeInMetersX();
      map_height_ = costmap_->getSizeInMetersY();
      resolution_ = costmap_->getResolution();
      node_handle_.getParam("/move_base/step_size_", step_size_);
      node_handle_.param("/move_base/delta_", delta_);
      node_handle_.param("/move_base/goal_radius_", goal_radius_);
      node_handle_.param("/move_base/max_iterations_", max_iterations_);
      current_iterations_ = 0;
      
      
      // publish stuff for viewing
      pub_tree_ = node_handle_.advertise<visualization_msgs::Marker>("rrt_tree", 0);
      ROS_INFO("RRT Tree visualization publishing to 'rrt_tree'.");
     
      // Get obstacles in the costmap
      map_width_cells_ = costmap_-> getSizeInCellsX();
      map_height_cells_ = costmap_-> getSizeInCellsY();

      
      for (unsigned int iy = 0; iy < map_height_cells_; iy++) {
        for (unsigned int ix = 0; ix < map_width_cells_; ix++) {
          unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
          if (cost == 0)
            obstacle_map_.push_back(true);
          else
            obstacle_map_.push_back(false);
        }
      }
      
      //debug
      for (unsigned int iy = 0; iy < map_height_cells_; iy++) {
        for (unsigned int ix = 0; ix < map_width_cells_; ix++) {
          if (obstacle_map_.at(iy * map_width_cells_ + ix))
              std::cout << "0";
          else
            std::cout << "1";
        }
        std::cout << "" << std::endl;
      } 
      
      // Display info message
      ROS_INFO("RRT planner initialized successfully.");
      initialized_ = true;
    } else {
      ROS_WARN("RRT planner has already been initialized.");
    }
  }
  
  bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal, 
                            std::vector<geometry_msgs::PoseStamped>& plan) {

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
    current_iterations_ = 0;
    vertex_list_.clear();
    x_origin_ = start.pose.position.x;
    y_origin_ = start.pose.position.y;
    costmap_ = costmap_ros_->getCostmap();
    x_goal_ = goal.pose.position.x;
    y_goal_ = goal.pose.position.y;
    map_width_cells_ = costmap_-> getSizeInCellsX();
    map_height_cells_ = costmap_-> getSizeInCellsY();
    
    for (unsigned int iy = 0; iy < map_height_cells_; iy++) {
      for (unsigned int ix = 0; ix < map_width_cells_; ix++) {
        unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
        if (cost == 0)
          obstacle_map_.push_back(true);
        else
          obstacle_map_.push_back(false);
      }
    }    
    
    // Initialize root node
    turtlebot_rrt::Vertex root(x_origin_, y_origin_, 0, -1);
    vertex_list_.push_back(root);    
    
    // Make sure that the goal header frame is correct
    // Goals are set within rviz
    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
      ROS_ERROR("This planner will only accept goals in the %s frame,"
                "the goal was sent to the %s frame.",
                costmap_ros_->getGlobalFrameID().c_str(),
                goal.header.frame_id.c_str());
      return false;
    }
    // Have the RRTPlanner calculate the full path
    ROS_DEBUG("Going into find_path");
    int goal_index = RRTPlanner::find_path(start, goal);
    
    ROS_DEBUG("Going into build_plan");
    plan = RRTPlanner::build_plan(goal_index, start, goal);
    
    if (plan.size() > 1) {
      ROS_INFO("A path was found.");
      return true;
    } else {
      ROS_WARN("No path was found.");
      return false;
    }
  }
  
  std::pair<float, float> RRTPlanner::get_random_point() {
    std::pair<float, float> random_point;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x(-map_width_, map_width_);
    std::uniform_real_distribution<> y(-map_height_, map_height_);
    
    random_point.first = x(gen);
    random_point.second = y(gen);
    
    return random_point;
  }
  
  int RRTPlanner::find_path(
    const geometry_msgs::PoseStamped& start, 
    const geometry_msgs::PoseStamped& goal) {
    bool done = false;
    int goal_index = -1;
    current_iterations_ = 0;
    while (!done && current_iterations_ < max_iterations_) {
      ROS_DEBUG("Finding the path.");
      
      // get a random point on the map
      std::pair<float, float> random_point = RRTPlanner::get_random_point();
      ROS_DEBUG("Random point: %.2f, %.2f", random_point.first, random_point.second);
      
      // find the closest known vertex to that point
      int closest_vertex = RRTPlanner::get_closest_vertex(random_point);
      
      // try to move from the closest known vertex towards the random point
      if(RRTPlanner::move_towards_point(closest_vertex, random_point)) {
        ROS_DEBUG("Moved, closest vertex: %d", closest_vertex);
        
        // check if we've reached our goal
        int new_vertex = vertex_list_.back().get_index();
        done = reached_goal(new_vertex);
        
        if (done) {
          ROS_INFO("Hey, we reached our goal, index: %d", new_vertex);
          goal_index = new_vertex;
        }
      }
      current_iterations_++;
      if (current_iterations_ == max_iterations_)
        ROS_INFO("Max iterations reached, no plan found.");
    }
    
    return goal_index;
  }
  
  int RRTPlanner::get_closest_vertex(std::pair<float, float> random_point) {
    // Set our closest Vertex index to our root, since we know that it exists
    int closest = -1;
    
    // closest_distance will keep track of the closest distance we find
    float closest_distance = std::numeric_limits<float>::infinity();
    
    // current_distance will keep track of the distance of the current
    float current_distance = std::numeric_limits<float>::infinity();
    
    // iterate through the vertex list to find the closest
    for (turtlebot_rrt::Vertex v : vertex_list_) {
      current_distance = get_distance(v.get_location(), random_point);
      
      // If the current distance is closer than what was previously
      // saved, update
      if (current_distance < closest_distance) {
        ROS_DEBUG("Closest distance: %.5f, vertex: %d.", current_distance, v.get_index());
        closest = v.get_index();
        closest_distance = current_distance;
      }
    }
    return closest;
  }
  
  float RRTPlanner::get_distance(std::pair<float, float> start_point, 
                                 std::pair<float, float> end_point) {
    // coords for our first point
    float x1 = start_point.first;
    float y1 = start_point.second;
    
    // coords for our second point
    float x2 = end_point.first;
    float y2 = end_point.second;
    
    
    
    // euclidean distance
    float distance = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
    ROS_DEBUG("Distance: %.5f", distance);
    return distance;
  }
  
  bool RRTPlanner::move_towards_point(int closest_vertex, 
                                      std::pair<float, float> random_point) {
    ROS_DEBUG("In move_towards_point");
    float x_closest = vertex_list_.at(closest_vertex).get_location().first;
    float y_closest = vertex_list_.at(closest_vertex).get_location().second;
    float x_random = random_point.first;
    float y_random = random_point.second;
    
    float theta = atan2(y_random - y_closest, x_random - x_closest);
    
    float new_x = x_closest + step_size_ * cos(theta);
    float new_y = y_closest + step_size_ * sin(theta);

    std::pair<float, float> proposed_point(new_x, new_y);
    std::pair<float, float> closest_point(x_closest, y_closest);
    
    // Check if the path between closest_vertex and the new point
    // is safe
    if (is_safe(proposed_point, closest_point)) {
      // If safe, add new Vertex to the back of vertex_list_
      turtlebot_rrt::Vertex new_vertex(new_x, new_y, vertex_list_.size(), 
                                       closest_vertex);
      vertex_list_.push_back(new_vertex);

/*      // publish the vertex in rviz
      visualization_msgs::Marker vertex;
      vertex.header.frame_id = "rrt_tree";
      vertex.header.stamp = ros::Time();
      vertex.ns = "turtlebot_rrt";
      vertex.id = new_vertex.get_index();
      vertex.type = 2; // sphere
      vertex.action = 0; //add
      vertex.pose.position.x = new_vertex.get_location().first;
      vertex.pose.position.y = new_vertex.get_location().second;
      vertex.pose.position.z = 1;
      vertex.pose.orientation.x = 0.0;
      vertex.pose.orientation.y = 0.0;
      vertex.pose.orientation.z = 0.0;
      vertex.pose.orientation.w = 1.0;
      vertex.scale.x = 0.1;
      vertex.scale.y = 0.1;
      vertex.scale.z = 0.1;
      vertex.color.a = 1.0;
      vertex.color.r = 0.0;
      vertex.color.g = 0.0;
      vertex.color.b = 1.0f;
      vertex.lifetime = ros::Duration();
      pub_tree_.publish(vertex); 
      ROS_DEBUG("Tree marker published.");
*/      
      // Return true, that we moved towards the proposed point
      return true;
    }
    // Return false, move not made
    return false;
  }
  
  bool RRTPlanner::is_safe(std::pair<float, float> start_point, 
                           std::pair<float, float> end_point) {
    // need to convert fro world coords to map coords to cell coords
    unsigned int map_x, map_y;
    
    // Check to see that our end point is within map bounds
    if (end_point.first < 0 || end_point.first > RRTPlanner::map_width_ ||
        end_point.second < 0 || end_point.second > RRTPlanner::map_height_)
      return false;
    
    // Check to make sure our end point isn't inside an obstacle
    costmap_->worldToMap(end_point.first, end_point.second, map_x, map_y);
    if(!obstacle_map_.at(map_y * map_width_cells_ + map_x))
      return false;
    
    // Check the path at intervals of delta for collision
    float theta = atan2(end_point.second - start_point.second,
                        end_point.first - start_point.first);
    float current_x = start_point.first;
    float current_y = start_point.second;
    
    while (current_x < end_point.first && current_y < end_point.second) {
      current_x += RRTPlanner::delta_ * cos(theta);
      current_y += RRTPlanner::delta_ * sin(theta);
      
      // Check for collision
      costmap_->worldToMap(current_x, current_y, map_x, map_y);
      if(!obstacle_map_.at(map_y * map_width_cells_ + map_x))
        return false;
    } 
    return true;
  }
  
  bool RRTPlanner::reached_goal(int new_vertex) {
    // TODO get goal location from cost map or parameters
    ROS_DEBUG("In reached_goal, vertex index: %d.", new_vertex);
    std::pair<float, float> goal(x_goal_, y_goal_);
    std::pair<float, float> current_location;
    current_location.first = 
      vertex_list_.at(new_vertex).get_location().first;
    current_location.second = 
      vertex_list_.at(new_vertex).get_location().second;
    ROS_DEBUG("cx: %.5f, cy: %.5f, gx: %.5f, gy: %.5f", 
              current_location.first, 
              current_location.second, 
              goal.first, 
              goal.second);
    
    // Check distance between current point and goal, if distance is less
    // than goal_radius_ return true, otherwise return false
    float distance = get_distance(current_location, goal);
    ROS_DEBUG("Distance to goal: %.5f", distance);
    
    if (distance <= goal_radius_)
      return true;
    else
      return false;
  }
  
  std::vector<geometry_msgs::PoseStamped> 
    RRTPlanner::build_plan(int goal_index, 
                           const geometry_msgs::PoseStamped& start, 
                           const geometry_msgs::PoseStamped& goal) {
      ROS_INFO("Building the plan.");
      
      // reset our current iterations
      current_iterations_ = 0;
      
      // The plan we'll be adding to and returning
      std::vector<geometry_msgs::PoseStamped> plan;
      if (goal_index == -1)
        //no plan found
        return plan;
      
      // The list of vertex indices we pass through to get to the goal
      std::deque<int> index_path;
      int current_index = goal_index;
      while(current_index > 0) {
        index_path.push_front(current_index);
        current_index = vertex_list_.at(current_index).get_parent();
      }
      index_path.push_front(0);
      
      
      for(int i : index_path) {
        if(i == 0)
          plan.push_back(start);
        else {
          geometry_msgs::PoseStamped pos;

          pos.pose.position.x = vertex_list_.at(i).get_location().first;
          pos.pose.position.y = vertex_list_.at(i).get_location().second;
          pos.pose.position.z = 0.0;
          
          // TODO figure out angles between points for the orientation piece
          pos.pose.orientation = tf::createQuaternionMsgFromYaw(0);
          plan.push_back(pos);
        }
      }
      plan.push_back(goal);
      for (geometry_msgs::PoseStamped p : plan)
        ROS_INFO("x: %.5f, y: %.5f", p.pose.position.x, p.pose.position.y);
      return plan;
  }
};
