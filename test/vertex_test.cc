/*
 * @copyright Copyright (C) 2017, Jessica Howard
 * @author Jessica Howard
 * @file turtlebot_rrt/test/vertex_test.cc
 *
 * @brief Unit tests for the Vertex class that is required in the RRTPath
 * global path planning plugin
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
#include "turtlebot_rrt/vertex.h"

TEST(VertexTest, SingleVertex) {
  turtlebot_rrt::Vertex vertex(5.0, 7.5, 3, 2);
  
  // test get location
  EXPECT_FLOAT_EQ(vertex.get_location().first, 5.0);
  EXPECT_FLOAT_EQ(vertex.get_location().second, 7.5);
  
  // test get index
  EXPECT_EQ(vertex.get_index(), 3);
  
  // test get_parent
  EXPECT_EQ(vertex.get_parent(), 2);
  
  // use setter methods and retest
  vertex.set_location(3.0, 6.23);
  vertex.set_index(12);
  vertex.set_parent(1);
  
  // test get location
  EXPECT_FLOAT_EQ(vertex.get_location().first, 3.0);
  EXPECT_FLOAT_EQ(vertex.get_location().second, 6.23);
  
  // test get index
  EXPECT_EQ(vertex.get_index(), 12);
  
  // test get_parent
  EXPECT_EQ(vertex.get_parent(), 1);
}
