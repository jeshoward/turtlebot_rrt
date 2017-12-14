/*
 * @copyright Copyright (C) 2017, Jessica Howard
 * @author Jessica Howard
 * @file turtlebot_rrt/test/rrtpath_test.cc
 *
 * @brief Unit tests for the RRT path planning algorithm
 *
 * @license 3-Clause BSD License
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor
 *     the names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission.
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <turtlebot_rrt/turtlebot_rrt.h>

namespace turtlebot_rrt {

  TEST(RRTPath, obstacleMap) {
    EXPECT_TRUE(true);
  }
};

turtlebot_rrt::RRTPlanner* rrt = NULL;
costmap_2d::Costmap2DROS* cost_map = NULL;

int main(int argc, char** argv) {
  ros::init(argc, argv, "rrt_test");
  testing::InitGoogleTest(&argc, argv);

  unsigned int cells_size_x = 384;
  unsigned int cells_size_y = 384;
  double resolution = 0.05;
  double origin_x = -10.0;
  double origin_y = -10.0;

  cost_map = new costmap_2d::Costmap2DROS(cells_size_x,
                                          cells_size_y,
                                          resolution,
                                          origin_x,
                                          origin_y);
  rrt = new turtlebot_rrt::RRTPlanner("turtlebot_rrt", cost_map);

  return RUN_ALL_TESTS();
}
