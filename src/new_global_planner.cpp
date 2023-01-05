#include "planner.hpp"
#include <ios>
#include <pluginlib/class_list_macros.h>

#include <fstream>
#include <iostream>
#include <string>

PLUGINLIB_EXPORT_CLASS(global_planner::NewGlobalPlanner,
                       nav_core::BaseGlobalPlanner)

using namespace std;

namespace global_planner {
NewGlobalPlanner::NewGlobalPlanner() {}

double tolerance;
int K_in;
double d;
bool viz_tree;
double pathLength, computationTime, pathQuality;
int numNodes;

NewGlobalPlanner::NewGlobalPlanner(std::string name,
                                   costmap_2d::Costmap2DROS *costmap_ros) {
  initialize(name, costmap_ros);
}

void NewGlobalPlanner::initialize(std::string name,
                                  costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    ROS_INFO("Initializing New_Global_Planner");
    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    footprint = costmap_ros_->getRobotFootprint();
    robot_radius = getRobotRadius(footprint);

    ros::NodeHandle nh("~/" + name);
    nh.param("tolerance", tolerance, 0.05);
    nh.param("K_in", K_in, 4000);
    nh.param("d", d, 0.2);
    nh.param("viz_tree", viz_tree, true);

    plan_pub_ = nh.advertise<nav_msgs::Path>("new_global_plan", 1);

    if (viz_tree) {
      tree_pub_ = nh.advertise<visualization_msgs::Marker>("Rapid_tree", 1);
    }

    initialized_ = true;
  }
}

bool NewGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan) {
  if (!initialized_) {
    ROS_ERROR("New Global planner was not initialized, performing "
              "initialization step");
    initialize(name_, costmap_ros_);
    return false;
  }

  ROS_INFO("Generating new global plan using New_Global_Planner");
  double startTime = ros::Time::now().toSec();
  rapidtree T_out =
      generateRapidtree(start, goal, this->costmap_ros_, this->robot_radius,
                        this->tolerance, this->K_in, this->d);

  if (viz_tree) {
    visualization_msgs::Marker tree_msg;
    init_line(&tree_msg);
    for (auto edge : T_out.edges) {
      pub_line(&tree_msg, &tree_pub_, edge.at(0).x, edge.at(0).y, edge.at(1).x,
               edge.at(1).y);
    }
  }

  if (T_out.success) {

    getGlobalPath(&T_out, &plan, start, goal);
    computeMetrics(T_out, goal, pathLength, computationTime, numNodes,
                   pathQuality, startTime, plan);
    // ROS_INFO("Start node: %.2f, %.2f ", start.pose.position.x,
    //          start.pose.position.y);
    // ROS_INFO("Goal : %.2f, %.2f ", goal.pose.position.x,
    // goal.pose.position.y); ROS_INFO("Path length: %.2f m", pathLength);
    // ROS_INFO("Computation time: %.2f sec", computationTime);
    // ROS_INFO("Number of nodes in tree: %d", numNodes);
    // ROS_INFO("Path quality: %.2f rad", pathQuality);
    ROS_INFO("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %d, %.2f",
             start.pose.position.x, start.pose.position.y, goal.pose.position.x,
             goal.pose.position.y, pathLength, computationTime, numNodes,
             pathQuality);

    const char *path =
        "/home/catkin_ws/src/new_global_planner/src/rrt_performance.txt";
    ofstream output_file(path, ios::app);
    // if (!output_file)
    //   ROS_INFO("NO such file foundddd");

    output_file << "Start node: " << start.pose.position.x << ","
                << start.pose.position.y << std::endl;
    output_file << "Goal node: " << goal.pose.position.x << ","
                << goal.pose.position.y << std::endl;
    output_file << "Path length: " << pathLength << std::endl;
    output_file << "Number of nodes: " << numNodes << std::endl;
    output_file << "Computation time: " << computationTime << "sec"
                << std::endl;
    output_file << "Path Quality: " << pathQuality << std::endl;
    output_file.close();

    publishPlan(plan);
    return true;
  } else {
    ROS_INFO("Couldn't find a valid path, trying again");
    return false;
  }
}

void NewGlobalPlanner::publishPlan(
    const std::vector<geometry_msgs::PoseStamped> &plan) {
  nav_msgs::Path rviz_path;
  rviz_path.poses.resize(plan.size());

  if (plan.empty()) {
    rviz_path.header.frame_id = "map";
    rviz_path.header.stamp = ros::Time::now();
  } else {
    rviz_path.header.frame_id = plan[0].header.frame_id;
    rviz_path.header.stamp = plan[0].header.stamp;
  }

  for (unsigned int i = 0; i < plan.size(); i++) {
    rviz_path.poses[i] = plan[i];
  }

  plan_pub_.publish(rviz_path);
}
}; // namespace global_planner