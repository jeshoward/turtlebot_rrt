# Converting Maps from Gazebo .world Files to YAML Format
One of the requirements for this project was to provide a demo in Gazebo. Unfortunately there isn't a clean way (that I have found) to create a world in Gazebo and have it work with the costmaps required in the navigation stack.

Overall, what I did was to create a world in Gazebo. I used the building editor to create a simplified maze and then saved it as a .world file. To do the conversion I used a simulated Turtlebot3 and the packaged SLAM algorithm to drive around the map and map it in Rviz.

![Gazebo Simple Maze world](/images/simple_maze_gazebo.png)
