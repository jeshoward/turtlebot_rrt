/*
 * @copyright Copyright (C) 2017, Jessica Howard 
 * @author Jessica Howard
 * @file turtlebot_rrt/include/turtlebot_rrt/vertex.h
 *
 * @brief Small class to maintain information held in vertices 
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

#ifndef INCLUDE_VERTEX_H_
#define INCLUDE_VERTEX_H_

#include <cmath>
#include <utility>

namespace RRT_Path {

    class Vertex {
    private:
        /**
         * @brief the x coordinate of the vertex
         */
        float x_;
        /**
         * @brief the y coordinate of the vertex
         */
        float y_;

        /**
         * @brief the index of the vertex
         */
        int index_;

        /**
         * @brief the vertex's parent index
         */
        int parent_index_;

    public:
        /**
         * @brief the constructor for a Vertex
         * @param x the x coordinate of the vertex
         * @param y the y coordinate of the vertex
         * @param index the index of the vertex
         * @param parent_index the index of the parent vertex
         */
        Vertex(float x, float y, int index, int parent_index);

        /**
         * @brief returns the x,y location of the vertex
         * @return returns std::pair<x,y>
         */
        std::pair<float, float> get_location();
        
        /**
         * @brief returns the index of the vertex
         * @return index of the vertex
         */
        int get_index();

        /**
         * @brief returns the index of the parent vertex
         * @return returns the index of the parent
         */
        int get_parent();

        /**
         * @brief overload of == operator
         */
        bool operator==(const Vertex& v) {
            return (x_ == v.x_ && y_ == v.y_ && parent_index_ == v.parent_index_);
        }

        /**
         * @brief overload of != operator
         */
        bool operator!=(const Vertex& v) {
            return (x_ != v.x_ || y_ != v.y_ || parent_index_ != v.parent_index_);
        }
    };
}

#endif /* INCLUDE_VERTEX_H_ */