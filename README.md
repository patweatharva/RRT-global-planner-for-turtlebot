# RRT global planner
This is a new global planner plugin for ROS that uses the Rapidly-Exploring Random Trees (RRT) algorithm to plan a path for the robot. The plugin is implemented in C++ and is designed to be used with the [navigation stack](http://wiki.ros.org/navigation) of ROS.

![alt text](https://github.com/patweatharva/RRT-global-planner-for-turtlebot/blob/main/refrences/InkedScreenshot%202022-12-11%20192943.jpg)

## Note
The plugin was created and tested in the ROS noetic version and might not work in other distributions.


## Features
* The plugin can be easily configured using ROS parameters.
* The tree generated by the RRT algorithm can be visualized for debugging and analysis.
* The plugin provides metrics such as path length, computation time, number of nodes and path quality.

## Intallation 

1. Install the turtlebot simulation, SLAM(optional) and Navigation packages in your machine by following instructions provided in the [E-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)

2. You can also [use the SLAM node](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/) for mapping your map 

3. After this, clone this repository into the **'src'** directory of your catkin workspace.

```
cd catkin_ws/src
git clone https://github.com/patweatharva/new_global_planner.git

```
4. Build the package using `catkin_make` in the terminal

5. Add the plugin to the `global_planner` parameter in the `move_base` launch file of the turtlebot package.

```
<param name="global_planner" value="new_global_planner/NewGlobalPlanner"/>
```
The plugin can be configured using the following parameters:

## Configuration

* `tolerance` (default: 0.05): The tolerance for the goal position.
* `K_in` (default: 4000): The maximum number of iterations of the RRT algorithm.
* `d` (default: 0.2): The distance between the newly generated point and the nearest point in the tree.
* `viz_tree` (default: true): A flag to enable/disable visualization of the tree.

These parameters can be set in the move_base launch file or in a separate configuration file.

## Usage
The plugin can be used by running the move_base node, which is part of the navigation stack. The robot will start moving towards the goal position and the plugin will continuously plan a path for the robot.

## Metrics
The plugin provides the following metrics:

* `pathLength : The length of the final path.
* `computationTime`: The time taken to compute the path.
* `numNodes` : The number of nodes in the final tree.
* `pathQuality` : The quality of the path (measured by a user-defined metric).

These metrics can be accessed using the `/new_global_planner/metrics` topic.

## Support
Please open an issue if you have any questions or encounter any problems with the plugin.

## Contributions
All contributions are welcome! If you would like to contribute to the plugin, please fork this repository and submit a pull request.

## References

* S.M. LaValle and J.J. Kuffner, "Rapidly-exploring random trees: A new tool for path planning," in IEEE International Conference on Robotics and Automation, vol. 3, pp. 995-1001, 2000.

* R. Kuffner and J. LaValle, "RRT-connect: An efficient approach to single-query path planning," in IEEE International Conference on Robotics and Automation, vol. 3, pp. 2152-2157, 2004.

* R. Kuffner and J. LaValle, "RRT-star: An efficient approach to single-query path planning," in IEEE International Conference on Robotics and Automation, vol. 3, pp. 995-1001, 2010.


## Contribution
If you want to contribute to this project, you are most welcome. Just fork the repo and make a pull request.

## License
This project is licensed under the [MIT License](https://github.com/patweatharva/RRT-global-planner-for-turtlebot/blob/main/LICENSE) - see the LICENSE file for details.

## Contact
If you have any questions or feedback, please feel free to contact me at [patweatharva@gmail.com](patweatharva@gmail.com)

