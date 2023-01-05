#include "ros/time.h"
#include <algorithm>
#include <angles/angles.h>
#include <math.h>
#include <stdlib.h>
#include <vector>

#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

#include <fstream>
#include <iostream>
using namespace std;

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

#define TWO_M_PI 2 * M_PI
#define M_PI_10 M_PI / 10.

namespace global_planner {
class NewGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
  NewGlobalPlanner();
  NewGlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);
  void publishPlan(const std::vector<geometry_msgs::PoseStamped> &plan);
  ~NewGlobalPlanner() {}

private:
  std::string name_;
  costmap_2d::Costmap2DROS *costmap_ros_;
  costmap_2d::Costmap2D *costmap_;
  std::vector<geometry_msgs::Point> footprint;
  double tolerance, d, robot_radius;
  int K_in;
  bool viz_tree, initialized_;
  ros::Publisher plan_pub_, tree_pub_;
};
}; // namespace global_planner

#endif

struct tree_node {
  int parent_id{};
  geometry_msgs::Point vertex{};
};

class rapidtree {
public:
  geometry_msgs::Point x_initial{};
  std::vector<tree_node> tree_nodes{};
  std::vector<std::vector<geometry_msgs::Point>> edges{};
  costmap_2d::Costmap2DROS *X_space;
  bool success{0};

  rapidtree(geometry_msgs::Point x_init,
            costmap_2d::Costmap2DROS *costmap_ros) {
    x_initial = x_init;

    this->X_space = costmap_ros;

    tree_node initial_node;
    initial_node.parent_id = 0;
    initial_node.vertex = x_init;
    add_vertex(initial_node);
  }

  void add_vertex(const tree_node new_node) {
    this->tree_nodes.push_back(new_node);
  }

  void add_edge(geometry_msgs::Point point1, geometry_msgs::Point point2) {
    std::vector<geometry_msgs::Point> edge{};
    edge.push_back(point1);
    edge.push_back(point2);
    this->edges.push_back(edge);
  }

  ~rapidtree(){};
};

double randomDouble(double fMin, double fMax) {
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

double getDistance(const geometry_msgs::Point point1,
                   const geometry_msgs::Point point2) {
  double distance =
      sqrt(pow(point2.y - point1.y, 2) + pow(point2.x - point1.x, 2));
  return distance;
}

bool inFreeSpace(const geometry_msgs::Point point,
                 const costmap_2d::Costmap2DROS *costmap_ros,
                 const double robot_radius_max) {
  bool result{1};
  double theta{0};
  double robot_radius_ii{robot_radius_max};
  double robot_radius_step(0.05);
  costmap_2d::Costmap2D *costmap_;
  geometry_msgs::Point point_to_check;
  unsigned int mx, my;
  std::vector<costmap_2d::MapLocation> map_polygon, polygon_cells;

  costmap_ = costmap_ros->getCostmap();

  while (theta <= TWO_M_PI) {
    costmap_2d::MapLocation map_loc;

    if (!costmap_->worldToMap(point.x + robot_radius_max * cos(theta),
                              point.y + robot_radius_max * sin(theta),
                              map_loc.x, map_loc.y)) {
      return false;
    }

    map_polygon.push_back(map_loc);

    theta += M_PI_10;
  }

  costmap_->convexFillCells(map_polygon, polygon_cells);

  for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
    if (costmap_->getCost(polygon_cells[i].x, polygon_cells[i].y) >=
        costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      result = 0;
      break;
    }
  }

  return result;
}

bool edgeInFreeSpace(const std::vector<geometry_msgs::Point> edge,
                     const costmap_2d::Costmap2DROS *costmap_ros,
                     const double robot_radius) {
  bool result{1};

  double dist = getDistance(edge[0], edge[1]);
  double num_points = dist / robot_radius;
  geometry_msgs::Point edge_pt_ii{};
  for (double ii = 0.; ii <= num_points; ii++) {
    edge_pt_ii.x = edge[0].x + ii * (edge[1].x - edge[0].x) / num_points;
    edge_pt_ii.y = edge[0].y + ii * (edge[1].y - edge[0].y) / num_points;

    if (!inFreeSpace(edge_pt_ii, costmap_ros, robot_radius)) {
      result = 0;
      break;
    }
  }

  return result;
}

geometry_msgs::Point getRandomState(costmap_2d::Costmap2DROS *costmap_ros,
                                    const double robot_radius) {
  geometry_msgs::Point randomState{};
  randomState.z = 0.;
  costmap_2d::Costmap2D *costmap_;
  costmap_ = costmap_ros->getCostmap();

  bool pointIsFree{0};

  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();

  while (!pointIsFree) {
    randomState.x =
        randomDouble(origin_x, origin_x + costmap_->getSizeInMetersX());
    randomState.y =
        randomDouble(origin_y, origin_y + costmap_->getSizeInMetersY());
    pointIsFree = inFreeSpace(randomState, costmap_ros, robot_radius);
  }
  return randomState;
}

tree_node getNearestNeighbor(const geometry_msgs::Point point1,
                             const rapidtree *T) {
  geometry_msgs::Point nearest_neighbor{};
  tree_node nearest_neighbor_node{};
  int parent_id{};
  double nearest_distance{HUGE_VAL};
  double current_distance{HUGE_VAL};

  for (int ii = 0; ii < T->tree_nodes.size(); ii++) {
    if (point1.x != T->tree_nodes.at(ii).vertex.x &&
        point1.y != T->tree_nodes.at(ii).vertex.y) {
      current_distance = getDistance(point1, T->tree_nodes.at(ii).vertex);
      if (current_distance < nearest_distance) {
        nearest_distance = current_distance;
        nearest_neighbor = T->tree_nodes.at(ii).vertex;
        parent_id = ii;
      }
    }
  }

  nearest_neighbor_node.vertex = nearest_neighbor;
  nearest_neighbor_node.vertex.z = 0.;
  nearest_neighbor_node.parent_id = parent_id;

  return nearest_neighbor_node;
}

tree_node extendTree(const tree_node point_near,
                     const geometry_msgs::Point point_rand, const double d) {
  tree_node point_new{};
  point_new.vertex.z = 0.;

  double theta = atan2(point_rand.y - point_near.vertex.y,
                       point_rand.x - point_near.vertex.x);
  point_new.vertex.x = point_near.vertex.x + d * cos(theta);
  point_new.vertex.y = point_near.vertex.y + d * sin(theta);

  point_new.parent_id = point_near.parent_id;

  return point_new;
}

rapidtree generateRapidtree(geometry_msgs::PoseStamped x_init,
                            geometry_msgs::PoseStamped x_final,
                            costmap_2d::Costmap2DROS *costmap_ros,
                            double robot_radius, double tolerance, int K,
                            double d) {
  rapidtree T(x_init.pose.position, costmap_ros);
  geometry_msgs::Point x_rand;
  tree_node x_near, x_new;

  for (int k = 1; k <= K; k++) {
    bool edgeIsFree{0};
    std::vector<geometry_msgs::Point> edge{};

    x_rand = getRandomState(T.X_space, robot_radius);
    x_near = getNearestNeighbor(x_rand, &T);
    x_new = extendTree(x_near, x_rand, d);

    edge.push_back(x_new.vertex);
    edge.push_back(x_near.vertex);

    edgeIsFree = edgeInFreeSpace(edge, T.X_space, robot_radius);
    if (edgeIsFree) {
      T.add_vertex(x_new);
      T.add_edge(x_near.vertex, x_new.vertex);
    } else {
      continue;
    };

    if (getDistance(x_new.vertex, x_final.pose.position) <= tolerance) {
      ROS_INFO("Found solution with %i/%i RRT vertices.", k, K);
      T.success = 1;
      break;
    }
  }

  return T;
}

double getRobotRadius(std::vector<geometry_msgs::Point> footprint) {
  double max_dist{0.}, dist{};
  geometry_msgs::Point origin{};
  origin.x = 0.;
  origin.y = 0.;
  origin.z = 0.;

  for (auto pt : footprint) {
    dist = getDistance(origin, pt);
    if (dist > max_dist) {
      max_dist = dist;
    }
  }
  return max_dist;
}

bool getGlobalPath(const rapidtree *tree,
                   std::vector<geometry_msgs::PoseStamped> *plan,
                   const geometry_msgs::PoseStamped &start,
                   const geometry_msgs::PoseStamped &goal) {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "map";
  int prev_id;
  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg;

  plan->clear();

  int current_id = tree->tree_nodes.size() - 1;
  pose_stamped.pose.orientation = goal.pose.orientation;

  while (current_id != 0) {
    pose_stamped.pose.position = tree->tree_nodes.at(current_id).vertex;
    plan->push_back(pose_stamped);
    prev_id = current_id;
    current_id = tree->tree_nodes.at(current_id).parent_id;

    double dy, dx, yaw;
    dy = tree->tree_nodes.at(prev_id).vertex.y -
         tree->tree_nodes.at(current_id).vertex.y;
    dx = tree->tree_nodes.at(prev_id).vertex.x -
         tree->tree_nodes.at(current_id).vertex.x;
    yaw = atan2(dy, dx);
    quat_tf.setRPY(0, 0, yaw);
    quat_msg = tf2::toMsg(quat_tf);
    pose_stamped.pose.orientation = quat_msg;
  }

  pose_stamped.pose.position = tree->tree_nodes.at(0).vertex;
  pose_stamped.pose.orientation = start.pose.orientation;
  plan->push_back(pose_stamped);
  std::reverse(plan->begin(), plan->end());

  return true;
}

void computeMetrics(rapidtree &rrt, const geometry_msgs::PoseStamped &goal,
                    double &pathLength, double &computationTime, int &numNodes,
                    double &pathQuality, double &startTime,
                    std::vector<geometry_msgs::PoseStamped> &plan) {

  // Compute the length of the path.
  pathLength = 0.0;
  for (size_t i = 0; i < plan.size() - 1; i++) {
    pathLength += hypot(plan[i].pose.position.x - plan[i + 1].pose.position.x,
                        plan[i].pose.position.y - plan[i + 1].pose.position.y);
  }

  // Compute the computation time.
  computationTime = ros::Time::now().toSec() - startTime;

  // Compute the number of nodes in the tree.
  numNodes = rrt.tree_nodes.size();

  // Compute the quality of the path.
  pathQuality = 0.0;
  for (size_t i = 0; i < rrt.tree_nodes.size() - 1; i++) {
    pathQuality += angles::shortest_angular_distance(
        atan2(plan[i].pose.position.y - plan[i + 1].pose.position.y,
              plan[i].pose.position.x - plan[i + 1].pose.position.x),
        atan2(goal.pose.position.y - plan[i].pose.position.y,
              goal.pose.position.x - plan[i].pose.position.x));
  }
  pathQuality /= plan.size();
}

void init_line(visualization_msgs::Marker *line_msg) {
  line_msg->header.frame_id = "map";
  line_msg->id = 0;
  line_msg->ns = "tree";
  line_msg->type = visualization_msgs::Marker::LINE_LIST;
  line_msg->action = visualization_msgs::Marker::ADD;
  line_msg->pose.orientation.w = 1.0;
  line_msg->scale.x = 0.05;
}

void pub_line(visualization_msgs::Marker *line_msg, ros::Publisher *line_pub,
              double x1, double y1, double x2, double y2) {
  line_msg->header.stamp = ros::Time::now();

  geometry_msgs::Point p1, p2;
  std_msgs::ColorRGBA c1, c2;

  p1.x = x1;
  p1.y = y1;
  p1.z = 1.0;

  p2.x = x2;
  p2.y = y2;
  p2.z = 1.0;

  c1.r = 1.0; // 1.0=255
  c1.g = 1.0;
  c1.b = 1.0;
  c1.a = 0.5; // alpha

  c2.r = 1.0; // 1.0=255
  c2.g = 1.0;
  c2.b = 1.0;
  c2.a = 0.5; // alpha

  line_msg->points.push_back(p1);
  line_msg->points.push_back(p2);

  line_msg->colors.push_back(c1);
  line_msg->colors.push_back(c2);

  // Publish line_msg
  line_pub->publish(*line_msg);
}