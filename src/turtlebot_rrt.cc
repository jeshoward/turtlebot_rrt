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
#include <pluginlib/class_list_macros.h>

namespace RRT_Planner {
    
    void RRTPlanner::initialize(std::string name, 
            costmap_2d::Costmap2DROS* costmap_ros) {
        if(!initialized_) {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros->getCostmap();
            
            ros::NodeHandle node_handle("~/" + name);
            //do node handle stuff here
            
            x_origin_ = costmap_->getOriginX();
            y_origin_ = costmap_->getOriginY();
            
            map_width_ = costmap_->getSizeInCellsX();
            map_height_ = costmap_->getSizeInCellsY();
            resolution_ = costmap_->getResolution();
            
            world_model_ = new base_local_planner::CostmapModel(*costmap_);
            
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
        if(!initialized_) {
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
        if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
            ROS_ERROR("This planner will only accept goals in the %s frame,"
                    "the goal was sent to the %s frame.", 
                    costmap_ros_->getGlobalFrameID().c_str(),
                    goal.header.frame_id.c_str());
            return false;
        }
        return false;
    }
}

// Register as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(RRT_Planner::RRTPlanner, nav_core::BaseGlobalPlanner)