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

#include <pluginlib/class_list_macros.h>
#include "turtlebot_rrt/turtlebot_rrt.h"
#include "turtlebot_rrt/vertex.h"

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
      node_handle_.getParam("/move_base/step_size_", step_size_);
      node_handle_.getParam("/move_base/delta_", delta_);
      node_handle_.getParam("/move_base/goal_radius_", goal_radius_);
      node_handle_.getParam("/move_base/max_iterations_", max_iterations_);
      ROS_INFO("Step size: %.2f, goal radius: %.2f, delta: %.2f, max "
               "iterations: %d", step_size_, goal_radius_, delta_,
               max_iterations_);
      current_iterations_ = 0;

      // Get obstacles in the costmap
      map_width_cells_ = costmap_-> getSizeInCellsX();
      map_height_cells_ = costmap_-> getSizeInCellsY();

      obstacle_map_ = RRTPlanner::buildObstacleMap(map_height_cells_,
                                                   map_width_cells_,
                                                   costmap_);


/*      //debug
      for (unsigned int iy = 0; iy < map_height_cells_; iy++) {
        for (unsigned int ix = 0; ix < map_width_cells_; ix++) {
          if (obstacle_map_.at(iy * map_width_cells_ + ix))
              std::cout << "0";
          else
            std::cout << "1";
        }
        std::cout << "" << std::endl;
      }
*/
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

    // reset path, iterations, vertex tree
    plan.clear();
    current_iterations_ = 0;
    ROS_INFO("Current iterations reset to %d.", current_iterations_);
    vertex_list_.clear();

    // reset origin and goal
    x_origin_ = start.pose.position.x;
    y_origin_ = start.pose.position.y;
    x_goal_ = goal.pose.position.x;
    y_goal_ = goal.pose.position.y;

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
    // Have the RRTPlanner calculate the path
    ROS_DEBUG("Going into FindPath");
    int goal_index = RRTPlanner::FindPath(start, goal);

    ROS_DEBUG("Going into BuildPlan");
    plan = RRTPlanner::BuildPlan(goal_index, start, goal);

    if (plan.size() > 1) {
      ROS_INFO("A path was found.");
      return true;
    } else {
      ROS_WARN("No path was found.");
      return false;
    }
  }

  std::vector<bool> RRTPlanner::buildObstacleMap(unsigned int map_height,
                                              unsigned int map_width,
                                              costmap_2d::Costmap2D* costmap) {
    std::vector<bool> obstacle_map;

    for (unsigned int iy = 0; iy < map_height; iy++) {
      for (unsigned int ix = 0; ix < map_width; ix++) {
        unsigned int cost = static_cast<int>(costmap->getCost(ix, iy));
        if (cost == 0)
          obstacle_map.push_back(true);
        else
          obstacle_map.push_back(false);
      }
    }
    return obstacle_map;
  }

  std::pair<float, float> RRTPlanner::GetRandomPoint(float map_width,
                                                     float map_height) {
    // generate random x and y coords within map bounds
    std::pair<float, float> random_point;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x(-map_width, map_width);
    std::uniform_real_distribution<> y(-map_height, map_height);

    random_point.first = x(gen);
    random_point.second = y(gen);

    return random_point;
  }

  int RRTPlanner::FindPath(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal) {
    bool done = false;
    int goal_index = -1;
    current_iterations_ = 0;
    while (!done && current_iterations_ < max_iterations_) {
      ROS_DEBUG("Finding the path.");

      // get a random point on the map
      std::pair<float, float> random_point =
        RRTPlanner::GetRandomPoint(map_width_, map_height_);
      ROS_DEBUG("Random point: %.2f, %.2f", random_point.first,
                random_point.second);

      // find the closest known vertex to that point
      int closest_vertex = RRTPlanner::GetClosestVertex(random_point,
                                                        vertex_list_);

      // try to move from the closest known vertex towards the random point
      if (RRTPlanner::MoveTowardsPoint(closest_vertex,
                                       random_point,
                                       vertex_list_)) {
        ROS_DEBUG("Moved, closest vertex: %d", closest_vertex);

        current_iterations_++;

        // check if we've reached our goal
        int new_vertex = vertex_list_.back().get_index();
        done = ReachedGoal(new_vertex,
                           vertex_list_,
                           std::pair<float, float>(x_goal_, y_goal_));

        if (done) {
          ROS_INFO("Hey, we reached our goal, index: %d", new_vertex);
          goal_index = new_vertex;
        }
      }

      if (current_iterations_ == max_iterations_)
        ROS_INFO("Max iterations reached, no plan found.");
    }

    return goal_index;
  }

  int RRTPlanner::GetClosestVertex(std::pair<float, float> random_point,
                              std::vector<turtlebot_rrt::Vertex> vertex_list) {
    // Set our closest Vertex index to our root, since we know that it exists
    int closest = -1;

    // closest_distance will keep track of the closest distance we find
    float closest_distance = std::numeric_limits<float>::infinity();

    // current_distance will keep track of the distance of the current
    float current_distance = std::numeric_limits<float>::infinity();

    // iterate through the vertex list to find the closest
    for (turtlebot_rrt::Vertex v : vertex_list) {
      current_distance = GetDistance(v.get_location(), random_point);

      // If the current distance is closer than what was previously
      // saved, update
      if (current_distance < closest_distance) {
        ROS_DEBUG("Closest distance: %.5f, vertex: %d.",
                  current_distance, v.get_index());
        closest = v.get_index();
        closest_distance = current_distance;
      }
    }
    return closest;
  }

  float RRTPlanner::GetDistance(std::pair<float, float> start_point,
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

  bool RRTPlanner::MoveTowardsPoint(int closest_vertex,
                              std::pair<float, float> random_point,
                              std::vector<turtlebot_rrt::Vertex> vertex_list) {
    ROS_DEBUG("In MoveTowardsPoint");
    float x_closest = vertex_list.at(closest_vertex).get_location().first;
    float y_closest = vertex_list.at(closest_vertex).get_location().second;
    float x_random = random_point.first;
    float y_random = random_point.second;

    float theta = atan2(y_random - y_closest, x_random - x_closest);

    float new_x = x_closest + step_size_ * cos(theta);
    float new_y = y_closest + step_size_ * sin(theta);

    std::pair<float, float> proposed_point(new_x, new_y);
    std::pair<float, float> closest_point(x_closest, y_closest);

    // Check if the path between closest_vertex and the new point
    // is safe
    if (IsSafe(proposed_point, closest_point, obstacle_map_)) {
      // If safe, add new Vertex to the back of vertex_list_
      turtlebot_rrt::Vertex new_vertex(new_x, new_y, vertex_list.size(),
                                       closest_vertex);
      ROS_DEBUG("Added new vertex at: %.5f, %.5f, index: %d",
               new_x, new_y, new_vertex.get_index());
      addVertex(new_vertex);

      // Return true, that we moved towards the proposed point
      return true;
    }
    // Return false, move not made
    return false;
  }

  void RRTPlanner::addVertex(turtlebot_rrt::Vertex new_vertex) {
    vertex_list_.push_back(new_vertex);
  }

  bool RRTPlanner::IsSafe(std::pair<float, float> start_point,
                          std::pair<float, float> end_point,
                          std::vector<bool> obstacle_map) {
    // check the path at intervals of delta for collision
    float theta = atan2(end_point.second - start_point.second,
                        end_point.first - start_point.first);
    float current_x = start_point.first;
    float current_y = start_point.second;

    unsigned int map_x, map_y;

    while (GetDistance(std::pair<float, float>(current_x, current_y),
                       end_point) > goal_radius_) {
      // increment towards end point
      current_x += delta_ * cos(theta);
      current_y += delta_ * sin(theta);

      // convert world coords to map coords
      costmap_->worldToMap(current_x, current_y, map_x, map_y);

      // check for collision
      if (!obstacle_map.at(map_y * map_width_cells_ + map_x))
        return false;
    }
    return true;
  }

  bool RRTPlanner::ReachedGoal(int new_vertex,
                      std::vector<turtlebot_rrt::Vertex> vertex_list,
                      std::pair<float, float> goal) {
    ROS_DEBUG("In ReachedGoal, vertex index: %d.", new_vertex);

    std::pair<float, float> current_location;
    current_location.first =
      vertex_list.at(new_vertex).get_location().first;
    current_location.second =
      vertex_list.at(new_vertex).get_location().second;

    ROS_DEBUG("cx: %.5f, cy: %.5f, gx: %.5f, gy: %.5f",
              current_location.first,
              current_location.second,
              goal.first,
              goal.second);

    // Check distance between current point and goal, if distance is less
    // than goal_radius_ return true, otherwise return false
    float distance = GetDistance(current_location, goal);
    ROS_DEBUG("Distance to goal: %.5f", distance);

    if (distance <= goal_radius_)
      return true;
    else
      return false;
  }

  std::vector<geometry_msgs::PoseStamped>
    RRTPlanner::BuildPlan(int goal_index,
                           const geometry_msgs::PoseStamped& start,
                           const geometry_msgs::PoseStamped& goal) {
      ROS_INFO("Building the plan.");

      // reset our current iterations
      current_iterations_ = 0;

      // The plan we'll be adding to and returning
      std::vector<geometry_msgs::PoseStamped> plan;

      // no plan found
      if (goal_index == -1)
        return plan;

      // The list of vertex indices we pass through to get to the goal
      std::deque<int> index_path;
      int current_index = goal_index;
      while (current_index > 0) {
        index_path.push_front(current_index);
        current_index = vertex_list_.at(current_index).get_parent();
      }
      index_path.push_front(0);

      // build the plan back up in PoseStamped messages
      for (int i : index_path) {
        if (i == 0) {
          plan.push_back(start);
        } else {
          geometry_msgs::PoseStamped pos;

          pos.pose.position.x = vertex_list_.at(i).get_location().first;
          pos.pose.position.y = vertex_list_.at(i).get_location().second;
          pos.pose.position.z = 0.0;

          pos.pose.orientation = tf::createQuaternionMsgFromYaw(0);
          plan.push_back(pos);
        }
      }
      plan.push_back(goal);
      for (geometry_msgs::PoseStamped p : plan)
        ROS_INFO("x: %.5f, y: %.5f", p.pose.position.x, p.pose.position.y);
      return plan;
  }
};  // namespace turtlebot_rrt
