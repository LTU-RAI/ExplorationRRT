#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64.h"
#include <chrono>
#include <dlfcn.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <list>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rrt/rrt_bindings.h>
#include <stdlib.h>
#include <tf/tf.h>
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <visualization_msgs/Marker.h>

#include "std_msgs/Int64.h"
#include <algorithm>
#include <bits/stdc++.h>
#include <chrono>
#include <climits>
#include <cmath>
#include <dlfcn.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>
#include <iterator>
#include <limits>
#include <list>
#include <math.h>
#include <mav_msgs/Status.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <memory>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <queue>
#include <ros/console.h>
#include <ros/message_traits.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <sensor_msgs/PointCloud2.h>
#include <set>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <tf/message_filter.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <ufo/math/vector3.h>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std::chrono;
using namespace std;
using namespace std::this_thread;
double planning_depth_ = 0;

double sensor_range_;
// helper obb function //

ufo::geometry::OBB makeOBB(ufo::math::Vector3 source, ufo::math::Vector3 goal,
                           float radius) {
  ufo::math::Vector3 direction = goal - source;
  ufo::math::Vector3 center = source + (direction / 2.0);
  double distance = direction.norm();
  direction /= distance;
  double yaw = -atan2(direction[1], direction[0]);
  double pitch = -asin(direction[2]);
  double roll = 0; // TODO: Fix
  ufo::geometry::OBB obb(center,
                         ufo::map::Point3(distance / 2.0, radius, radius),
                         ufo::math::Quaternion(roll, pitch, yaw));

  return obb;
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// // Structs

struct rrtSolverStatus rrt_solve(struct rrtCache *instance, double *u,
                                 const double *params, const double *y0,
                                 const double *c0);

struct rrtCache *rrt_new(void);

// The node struct.
// This struct represents the different kinds of points in the rrt, this
// includes: the nodes in the rrt tree, the goals and the paths.
struct node {
public:
  ufo::math::Vector3 *point;
  node *myParent = nullptr;
  double distanceToParent = -1;
  std::vector<ufo::math::Vector3> unknowns_in_sight_;
  std::list<struct node *> myChilds{};
  std::list<struct node *> myParents{};
  std::list<struct node *> myPath{};
  std::list<ufo::math::Vector3> myHits{};

  void addParents() {

    myParents.clear();
    node *temp_parent = nullptr;

    if (myParent != nullptr) {

      temp_parent = myParent;
    }

    while (temp_parent != nullptr) {

      myParents.push_back(temp_parent);
      temp_parent = temp_parent->myParent;

      if (temp_parent == nullptr) {
        break;
      }
    }
  }

  double distanceToGoal;
  node(float x, float y, float z) { point = new ufo::math::Vector3(x, y, z); }
  node(ufo::math::Vector3 *givenPoint) {
    point = new ufo::math::Vector3(givenPoint->x(), givenPoint->y(),
                                   givenPoint->z());
  }
  ufo::math::Vector3 *getNodePointer() { return point; }
  void addChild(node *newChild) { myChilds.push_back(newChild); }
  void addParent(node *newParent) { myParent = newParent; }
  void changeDistanceToGoal(double newDistance) {
    distanceToGoal = newDistance;
  }
  void changeDistanceToParent() {
    if (myParent != nullptr and distanceToParent != -1) {
      distanceToParent = sqrt(pow(point->x() - myParent->point->x(), 2) +
                              pow(point->y() - myParent->point->y(), 2) +
                              pow(point->z() - myParent->point->z(), 2));
    } else {
      distanceToParent = 0;
    }
  }
  double sumDistance() {
    changeDistanceToParent();
    double myDistance = 0;
    if (myParent != nullptr) {
      myDistance = myParent->sumDistance();
      changeDistanceToParent();
    }
    if (myParent == nullptr) {
      return 0;
    }
    return (myDistance + distanceToParent);
  }

  void getPath(std::list<struct node *> *givenPath) {
    if (myParent != nullptr) {
      myParent->getPath(givenPath);
      if (myParent->myParent != nullptr) {
        if (myParent->point->x() != point->x() or
            myParent->point->y() != point->y() or
            myParent->point->z() != point->z()) {
          givenPath->push_back(new node(myParent->point->x(),
                                        myParent->point->y(),
                                        myParent->point->z()));
        }
      }
    }
    if (myParent == nullptr) {
      return;
    }
    return;
  }

  void
  extractUnknownVoxelsInsideFrustum(ufo::map::OccupancyMapColor const &map) {

    unknowns_in_sight_.clear();
    ufo::geometry::Sphere sphere(*point, 4);

    std::vector<ufo::math::Vector3> unknown_voxels;

    for (auto it = map.beginLeaves(sphere, false, false, true, false,
                                   planning_depth_),
              it_end = map.endLeaves();
         it != it_end; ++it) {
      if (it.isUnknown()) {

        ufo::math::Vector3 free_voxel(it.getX(), it.getY(), it.getZ());
        unknown_voxels.push_back(free_voxel);
      }
    }

    double hFOV = 2 * M_PI;
    double vFOV = M_PI / 4;
    double range = 4;

    // TODO @ can use OMP to parallalize the following 3 loops

    for (ufo::math::Vector3 voxel : unknown_voxels) {

      ufo::math::Vector3 toPoint = voxel - *point;
      double h_angle = std::atan2(toPoint.y(), toPoint.x());
      double v_angle = std::atan2(toPoint.z(), toPoint.norm());

      if (std::abs(h_angle) <= hFOV / 2 and std::abs(v_angle) <= vFOV / 2 and
          toPoint.norm() <= range) {

        ufo::geometry::LineSegment myLine(*point, voxel);
        if (!isInCollision(map, myLine, true, false, false, planning_depth_)) {
          unknowns_in_sight_.push_back(voxel);
        }
      }
    }
  }

  int findInformationGain(float v_local_, float givenHorizontal,
                          float givenVertical, float givenMin, float givenMax,
                          ufo::map::OccupancyMapColor const &map,
                          bool excludePath, bool findAnyInfo) {

    if (myParents.empty()) {
      return 0;
    }

    double dist = 0;
    node *check_node = nullptr;
    check_node = *(myParents.begin());
    bool check_dist_pass = false;

    for (auto i = myParents.begin(); i != myParents.end(); i++) {

      if (i != myParents.begin()) {

        ufo::math::Vector3 v1 = *((*i)->point);
        ufo::math::Vector3 v2 = *(check_node->point);
        dist = (v1 - v2).norm();
        if (dist > 4) {
          check_node = *i;
          check_dist_pass = true;
        } else {
          check_dist_pass = false;
        }
      }

      if (check_dist_pass) {

        ufo::geometry::Sphere sphere(*((*i)->point), sensor_range_);

        std::list<ufo::math::Vector3> unknown_voxels;

        for (auto it = map.beginLeaves(sphere, false, false, true, false,
                                       planning_depth_),
                  it_end = map.endLeaves();
             it != it_end; ++it) {
          if (it.isUnknown()) {

            ufo::math::Vector3 unknown_voxel(it.getX(), it.getY(), it.getZ());
            unknown_voxels.push_back(unknown_voxel);
          }
        }

        double hFOV = 2 * M_PI;
        double vFOV = M_PI / 4;
        double range = sensor_range_;

        for (ufo::math::Vector3 voxel : unknown_voxels) {

          ufo::math::Vector3 toPoint = voxel - *((*i)->point);
          double h_angle = std::atan2(toPoint.y(), toPoint.x());
          double v_angle = std::atan2(toPoint.z(), toPoint.norm());

          if (std::abs(h_angle) <= hFOV / 2 and
              std::abs(v_angle) <= vFOV / 2 and toPoint.norm() <= range) {

            ufo::geometry::Sphere unk_sphere(voxel, 2);
            ufo::geometry::LineSegment myLine(*((*i)->point), voxel);
            if (!isInCollision(map, unk_sphere, true, false, false,
                               planning_depth_) and
                isInCollision(map, unk_sphere, false, true, false,
                              planning_depth_) and
                !isInCollision(map, myLine, true, false, false,
                               planning_depth_)) {
              myHits.push_back(voxel);
            }
          }
        }
      }
    }

    std::list<ufo::math::Vector3> myTotalHits{};
    addHits(&myTotalHits);
    int hits = myTotalHits.size();
    return hits;
  }

  void clearInformationGain() {
    myHits.clear();
    if (myParent != nullptr) {
      myParent->clearInformationGain();
    }
  }
  void addHits(std::list<ufo::math::Vector3> *hitList) {
    bool add = true;
    for (auto it = myHits.begin(), it_end = myHits.end(); it != it_end; ++it) {
      for (auto it2 = hitList->begin(), it_end2 = hitList->end();
           it2 != it_end2; ++it2) {
        if (it->x() == it2->x() and it->y() == it2->y() and
            it->z() == it2->z()) {
          add = false;
          break;
        }
      }
      if (add) {
        hitList->push_back(*it);
      }
      add = true;
    }
    if (myParent != nullptr) {
      myParent->addHits(hitList);
    }
  };

  bool findPathImprovement(struct node *targetNode,
                           ufo::map::OccupancyMapColor const &map,
                           float givenDistance, float givenRadious,
                           auto pathImprovement_start, int givenMax) {
    bool improvementFound;
    if (targetNode == this and myParent == nullptr) {
      return true;
    }
    if (myParent != nullptr) {
      improvementFound = myParent->findPathImprovement(
          targetNode, map, givenDistance, givenRadious, pathImprovement_start,
          givenMax);
      auto pathImprovement_stop = high_resolution_clock::now();
      auto pathImprovement_total =
          duration_cast<microseconds>(pathImprovement_stop -
                                      pathImprovement_start)
              .count();
      if (pathImprovement_total > givenMax) {
        return true;
      }
    } else {
      ufo::geometry::LineSegment myLine(*(targetNode->point), *point);
      if (!isInCollision(map, myLine, true, false, true, planning_depth_)) {
        ufo::math::Vector3 newVector(targetNode->point->x() - point->x(),
                                     targetNode->point->y() - point->y(),
                                     targetNode->point->z() - point->z());
        float distance = newVector.norm();
        float itterations = (distance / givenRadious);
        float part = givenRadious / distance;
        float xStep = (targetNode->point->x() - point->x()) * part;
        float yStep = (targetNode->point->y() - point->y()) * part;
        float zStep = (targetNode->point->z() - point->z()) * part;
        for (int i = 1; i < itterations; i++) {
          auto pathImprovement_stop = high_resolution_clock::now();
          auto pathImprovement_total =
              duration_cast<microseconds>(pathImprovement_stop -
                                          pathImprovement_start)
                  .count();
          if (pathImprovement_total > givenMax) {
            return true;
          }
          ufo::math::Vector3 newVector =
              ufo::math::Vector3(point->x() + i * xStep, point->y() + i * yStep,
                                 point->z() + i * zStep);
          ufo::geometry::Sphere new_sphere(newVector, givenRadious);
          if (isInCollision(map, new_sphere, true, false, true,
                            planning_depth_)) {
            return false;
          }
        }
        targetNode->addParent(this);
        return true;
      } else {
        return false;
      };
    }
    if (!improvementFound) {
      ufo::geometry::LineSegment myLine(*(targetNode->point), *point);
      if (!isInCollision(map, myLine, true, false, true, planning_depth_)) {
        ufo::math::Vector3 newVector(targetNode->point->x() - point->x(),
                                     targetNode->point->y() - point->y(),
                                     targetNode->point->z() - point->z());
        float distance = newVector.norm();
        float itterations = (distance / givenRadious);
        float part = givenRadious / distance;
        float xStep = (targetNode->point->x() - point->x()) * part;
        float yStep = (targetNode->point->y() - point->y()) * part;
        float zStep = (targetNode->point->z() - point->z()) * part;
        for (int i = 1; i < itterations; i++) {
          auto pathImprovement_stop = high_resolution_clock::now();
          auto pathImprovement_total =
              duration_cast<microseconds>(pathImprovement_stop -
                                          pathImprovement_start)
                  .count();
          if (pathImprovement_total > givenMax) {
            return true;
          }
          ufo::math::Vector3 newVector =
              ufo::math::Vector3(point->x() + i * xStep, point->y() + i * yStep,
                                 point->z() + i * zStep);
          ufo::geometry::Sphere new_sphere(newVector, givenRadious);
          if (isInCollision(map, new_sphere, true, false, true,
                            planning_depth_)) {
            return false;
          }
        }
        targetNode->addParent(this);
        improvementFound =
            findPathImprovement(this, map, givenDistance, givenRadious,
                                pathImprovement_start, givenMax);
        return true;
      } else {
        return false;
      }
    } else {
      return true;
    }
  }

  void readyForDeletion() { delete point; }

  bool isInCollision(ufo::map::OccupancyMapColor const &map,
                     ufo::geometry::BoundingVar const &bounding_volume,
                     bool occupied_space = true, bool free_space = false,
                     bool unknown_space = false,
                     ufo::map::DepthType min_depth = planning_depth_) {
    // Iterate through all leaf nodes that intersects the bounding volume
    for (auto it = map.beginLeaves(bounding_volume, occupied_space, free_space,
                                   unknown_space, false, min_depth),
              it_end = map.endLeaves();
         it != it_end; ++it) {
      // Is in collision since a leaf node intersects the bounding volume.
      return true;
    }
    // No leaf node intersects the bounding volume.
    return false;
  }
};

// End of structs
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// // Variables

// Variables

int n_seq_;
double dt_ = 1;

int number_of_nodes_;
int number_of_goals_;
int number_of_iterations_;
int nmpc_horizon_;
int itterations;
int advance_index = 0;
int path_improvement_max_;
float dist_nodes_; // 1.0;
float dist_goals_;
float min_dist_to_goal_;
float v_local_;
float SCALER_X = 0;
float SCALER_Y = 0;
float SCALER_Z = 0;
float position_x = 0;
float position_y = 0;
float position_z = 0;
float velocity_x = 0;
float velocity_y = 0;
float velocity_z = 0;
float lowest_x;
float lowest_y;
float lowest_z;
float highest_x;
float highest_y;
float highest_z;
float GLOBAL_STRATEGY_THRESHOLD;
float GLOBAL_PATH_THRESHOLD;
float initialGoalInfo = 0.0;
bool run_by_nodes_;
bool start_from_waypoint_;
bool map_received = false;
bool RRT_created = false;
bool GOALS_generated = false;
bool position_received = false;
bool fetched_path = false;
bool newPath = false;
bool allowNewPath = true;
bool recoveryUnderway = false;
bool visualizeNewData = true;
double min_sensor_range_;
double sensor_vertical_fov_;
double SENSOR_HORIZONTAL = 0.78;
double k_info_;
double k_dist_;
double k_u_;
double recalc_dist_;
double path_update_dist_;
double nmpc_dt_;
double robot_size_;
double min_info_goal_;
double goal_sensor_range_;
double goal_connect_dist_;

double position_tracking_weight_x_;
double position_tracking_weight_y_;
double position_tracking_weight_z_;
double angle_weight_roll_;
double angle_weight_pitch_;
double input_weight_thrust_;
double input_weight_roll_;
double input_weight_pitch_;
double input_rate_weight_thrust_;
double input_rate_weight_roll_;
double input_rate_weight_pitch_;
double initial_x_;
double initial_y_;
double initial_z_;
double roll = 0;
double pitch = 0;
double yaw = 0;
double totalCost = std::numeric_limits<float>::max();
double totalDistance = -1;
string map_frame_id_;
node *goalNode = nullptr;
node *reserveGoalNode = nullptr;
node *currentTarget;

// Ufomap
ufo::map::OccupancyMapColor myMap(0.1);

// Lists
std::list<struct node *> RRT_TREE{};
std::list<struct node *> CHOSEN_PATH{};
std::list<double> CHOSEN_PATH_VREF{};
std::list<double>::iterator vref_itterator;
std::list<struct node *> ALL_PATH{};
std::list<struct node *> myGoals{};
std::list<struct node *> myReserveGoals{};
std::list<ufo::math::Vector3> hits{};
std::list<node *> VISITED_POINTS{};
std::list<node *>::iterator path_itterator;

ros::Publisher m_trajectory_Publisher;
ros::Publisher m_command_Path_Publisher;
nav_msgs::Path command_path;
trajectory_msgs::MultiDOFJointTrajectory trajectory_array_;

// End of variables
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Functions

Eigen::Quaternion<float> Euler2Quaternion(Eigen::Vector3d v) {
  float roll = v.x();
  float pitch = v.y();
  float yaw = v.z();

  Eigen::Quaternion<float> q;

  q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

  return q;
}

// Fills in the space between nodes with new nodes.
// This allows for simpler path building.
void linSpace(node *givenNode, float givenDistance) {

  ufo::math::Vector3 newVector(
      givenNode->myParent->point->x() - givenNode->point->x(),
      givenNode->myParent->point->y() - givenNode->point->y(),
      givenNode->myParent->point->z() - givenNode->point->z());
  float distance = newVector.norm();
  float itterations = (distance / givenDistance);
  float part = givenDistance / distance;
  float xStep =
      (givenNode->myParent->point->x() - givenNode->point->x()) * part;
  float yStep =
      (givenNode->myParent->point->y() - givenNode->point->y()) * part;
  float zStep =
      (givenNode->myParent->point->z() - givenNode->point->z()) * part;
  node *parent = givenNode->myParent;
  node *nextNode = givenNode->myParent;
  for (int i = 1; i < itterations; i++) {
    node *newPoint = new node(givenNode->myParent->point->x() - i * xStep,
                              givenNode->myParent->point->y() - i * yStep,
                              givenNode->myParent->point->z() - i * zStep);
    newPoint->addParent(parent);
    parent = newPoint;
    RRT_TREE.push_back(newPoint);
  }
  givenNode->addParent(parent);
  if (nextNode->myParent != nullptr) {
    linSpace(nextNode, givenDistance);
  }
}

void segmentPath(const nav_msgs::Path &path, nav_msgs::Path &path_seg) {
  path_seg.poses.clear();
  double v_max_ = 1;
  double yaw_rate_max_ = 0.05;
  if (path.poses.size() == 0)
    return;
  if (path.poses.size() == 1)
    path_seg.poses.push_back(path.poses[0]);

  for (int i = 0; i < (path.poses.size() - 1); ++i) {
    Eigen::Vector3d start(path.poses[i].pose.position.x,
                          path.poses[i].pose.position.y,
                          path.poses[i].pose.position.z);
    Eigen::Vector3d end(path.poses[i + 1].pose.position.x,
                        path.poses[i + 1].pose.position.y,
                        path.poses[i + 1].pose.position.z);
    Eigen::Vector3d distance = end - start;
    double yaw_start = tf::getYaw(path.poses[i].pose.orientation);
    double yaw_end = tf::getYaw(path.poses[i + 1].pose.orientation);
    double yaw_direction = yaw_end - yaw_start;
    if (yaw_direction > M_PI) {
      yaw_direction -= 2.0 * M_PI;
    }
    if (yaw_direction < -M_PI) {
      yaw_direction += 2.0 * M_PI;
    }

    double dist_norm = distance.norm();
    double disc = std::min(dt_ * v_max_ / dist_norm,
                           dt_ * yaw_rate_max_ / abs(yaw_direction));

    bool int_flag = true;

    if (int_flag) {
      for (double it = 0.0; it <= 1.0; it += disc) {
        tf::Vector3 origin((1.0 - it) * start[0] + it * end[0],
                           (1.0 - it) * start[1] + it * end[1],
                           (1.0 - it) * start[2] + it * end[2]);
        double yaw = yaw_start + yaw_direction * it;
        if (yaw > M_PI)
          yaw -= 2.0 * M_PI;
        if (yaw < -M_PI)
          yaw += 2.0 * M_PI;
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        tf::Pose poseTF(quat, origin);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(poseTF, pose);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position = pose.position;
        pose_stamped.pose.orientation = pose.orientation;
        path_seg.poses.push_back(pose_stamped);
      }
    }
  }
}

void generateTrajectory() {

  nav_msgs::Path my_path;
  my_path.poses.clear();

  geometry_msgs::PoseStamped new_pose;

  for (auto i = std::next(CHOSEN_PATH.begin(), 0); i != CHOSEN_PATH.end();
       i++) {

    new_pose.pose.position.x = (*i)->point->x();
    new_pose.pose.position.y = (*i)->point->y();
    new_pose.pose.position.z = (*i)->point->z();

    new_pose.pose.orientation.x = 0;
    new_pose.pose.orientation.y = 0;
    new_pose.pose.orientation.z = 0;
    new_pose.pose.orientation.w = 1;

    my_path.poses.push_back(new_pose);
  }

  nav_msgs::Path seg_path;
  seg_path.poses.clear();

  nav_msgs::Path new_path;
  new_path.poses.clear();

  segmentPath(my_path, seg_path);

  for (int i = 0; i < seg_path.poses.size() - 1; i++) {

    geometry_msgs::PoseStamped p;
    float angle = atan2(seg_path.poses[i + 1].pose.position.y -
                            seg_path.poses[i].pose.position.y,
                        seg_path.poses[i + 1].pose.position.x -
                            seg_path.poses[i].pose.position.x);
    Eigen::Vector3d v(0, 0, angle);
    Eigen::Quaternion<float> q = Euler2Quaternion(v);

    p.pose.position.x = seg_path.poses[i].pose.position.x;
    p.pose.position.y = seg_path.poses[i].pose.position.y;
    p.pose.position.z = seg_path.poses[i].pose.position.z;
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    new_path.poses.push_back(p);
  }
  n_seq_++;
  mav_msgs::EigenTrajectoryPoint trajectory_point_;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg_;
  std::vector<geometry_msgs::Pose> executing_path_;

  trajectory_array_.header.seq = n_seq_;
  trajectory_array_.header.stamp = ros::Time::now();
  trajectory_array_.header.frame_id = "world";
  trajectory_array_.points.clear();
  double time_sum = 0;

  geometry_msgs::PoseStamped pose_ref;
  command_path.poses.clear();
  pose_ref.header.frame_id = "world";
  command_path.header.frame_id = "world";

  for (int i = 0; i < new_path.poses.size(); i++) {

    double yaw = tf::getYaw(new_path.poses[i].pose.orientation);
    Eigen::Vector3d p(new_path.poses[i].pose.position.x,
                      new_path.poses[i].pose.position.y,
                      new_path.poses[i].pose.position.z);
    trajectory_point_.position_W.x() = p.x();
    trajectory_point_.position_W.y() = p.y();
    trajectory_point_.position_W.z() = p.z();
    trajectory_point_.setFromYaw(yaw);
    pose_ref = new_path.poses[i];

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point_,
                                                       &trajectory_point_msg_);
    time_sum += dt_;

    // uncomment bwlow for yaw tracking trajectory.

    /*
      trajectory_point_msg_.time_from_start = ros::Duration(time_sum);
      trajectory_array_.points.push_back(trajectory_point_msg_);
    */

    command_path.poses.push_back(pose_ref);
  }
}

void publishTrajectory() {

  std::pair<Eigen::Vector3d, double>
      traj_ref_; // position and corresponding velocity reference.
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> traj_ref_vec_;
  traj_ref_vec_.clear();
  std::vector<Eigen::Vector3d> position_vec;
  std::vector<Eigen::Vector3d> vel_vec;
  position_vec.clear();
  vel_vec.clear();
  Eigen::Vector3d position_ref;
  Eigen::Vector3d vel_ref;

  for (auto path_it = CHOSEN_PATH.begin(); path_it != CHOSEN_PATH.end();
       ++path_it) {

    position_ref[0] = (*path_it)->point->x();
    position_ref[1] = (*path_it)->point->y();
    position_ref[2] = (*path_it)->point->z();
    position_vec.push_back(position_ref);
  }

  for (auto vel_it = CHOSEN_PATH_VREF.begin();
       vel_it != std::prev(CHOSEN_PATH_VREF.end(), 2);) {

    vel_ref[0] = *vel_it;
    vel_ref[1] = *std::next(vel_it, 1);
    vel_ref[2] = *std::next(vel_it, 2);
    vel_vec.push_back(vel_ref);
    vel_it++;
  }

  for (int i = 0; i < position_vec.size(); ++i) {
    std::pair<Eigen::Vector3d, Eigen::Vector3d> traj_pair =
        std::make_pair(position_vec[i], vel_vec[i]);
    traj_ref_vec_.push_back(traj_pair);
  }
  double t_sum = 0;
  geometry_msgs::Transform transform;
  geometry_msgs::Twist velocity;

  n_seq_++;
  trajectory_array_.header.seq = n_seq_;
  trajectory_array_.header.stamp = ros::Time::now();
  trajectory_array_.header.frame_id = "world";
  trajectory_array_.points.clear();

  for (int i = 0; i < traj_ref_vec_.size(); ++i) {

    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;
    transform.translation.x = traj_ref_vec_[i].first[0];
    transform.translation.y = traj_ref_vec_[i].first[1];
    transform.translation.z = traj_ref_vec_[i].first[2];

    // velocity.linear.x = traj_ref_vec_[i].second[0];
    // velocity.linear.y = traj_ref_vec_[i].second[1];
    // velocity.linear.z = traj_ref_vec_[i].second[2];

    // Velocities set to zero for debugging trajectory response.

    velocity.linear.x = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;
    trajectory_point.transforms.push_back(transform);
    trajectory_point.velocities.push_back(velocity);

    t_sum += dt_;
    trajectory_point.time_from_start = ros::Duration(t_sum);
    trajectory_array_.points.push_back(trajectory_point);
  }

  // Publish final trajectory. Command path is only for visualization.

  m_trajectory_Publisher.publish(trajectory_array_);
  m_command_Path_Publisher.publish(command_path);
}

// Evaluates the current point in the current path.
// This includes deciding when to change the current target to the next node in
// the path and when to calculate a new path.
void evaluateCurrentPoint(ros::Publisher *chosen_path_pub) {

  // generateTrajectory ();

  // ROS_INFO_STREAM ("Goal -- Number of parents : " <<
  // goalNode->myParents.size() << "\n");

  if ((sqrt(pow(position_x - goalNode->point->x(), 2) +
            pow(position_y - goalNode->point->y(), 2) +
            pow(position_z - goalNode->point->z(), 2)) < path_update_dist_)) {
    itterations = 0;
    fetched_path = false;
    RRT_created = false;
    GOALS_generated = false;
    position_received = false;
    allowNewPath = true;
  }

  // @NOTE : uncomment the code block below to publish REFERENCE_OUT_ on
  // nav_msgs::Odometry to use nmpc reference instead of trajectory tracking.

  /*
  if ((sqrt(pow(position_x - currentTarget->point->x(), 2) +
            pow(position_y - currentTarget->point->y(), 2) +
            pow(position_z - currentTarget->point->z(), 2)) < recalc_dist_) and
      path_itterator != --CHOSEN_PATH.end()) {
    advance_index++;
    path_itterator = CHOSEN_PATH.begin();
    std::advance(path_itterator, advance_index);
    currentTarget = *path_itterator;
    std::advance(vref_itterator, 3);
    if (path_itterator == CHOSEN_PATH.end()) {
      path_itterator--;
      currentTarget = *path_itterator;
    }
  }
  if (path_itterator != CHOSEN_PATH.end()) {
    nav_msgs::Odometry nextPoint;
    nextPoint.pose.pose.position.x = (currentTarget)->point->x();
    nextPoint.pose.pose.position.y = (currentTarget)->point->y();
    nextPoint.pose.pose.position.z = (currentTarget)->point->z();
    if (!recoveryUnderway) {
      nextPoint.twist.twist.linear.x = *vref_itterator;
      vref_itterator++;
      nextPoint.twist.twist.linear.y = *vref_itterator;
      vref_itterator++;
      nextPoint.twist.twist.linear.z = *vref_itterator;
      std::advance(vref_itterator, -2);
    } else {
      nextPoint.twist.twist.linear.x = 0;
      nextPoint.twist.twist.linear.y = 0;
      nextPoint.twist.twist.linear.z = 0;
    }
    nextPoint.pose.pose.orientation.x = 0;
    nextPoint.pose.pose.orientation.y = 0;
    nextPoint.pose.pose.orientation.z = 0;
    nextPoint.pose.pose.orientation.w = 0;
    nextPoint.header.stamp = ros::Time::now();
    nextPoint.header.frame_id = map_frame_id_;
    chosen_path_pub->publish(nextPoint);
  }

  */
}

// Builds and publishes the visualization messages.
void visualize(ros::Publisher *points_pub, ros::Publisher *output_path_pub,
               ros::Publisher *all_path_pub, ros::Publisher *goal_pub,
               ros::Publisher *hits_pub, ros::Publisher *taken_path_pub,
               ros::Publisher *map_pub, ros::Publisher *position_pub,
               ros::Publisher *unknowns_pub) {
  visualization_msgs::Marker RRT_points, RRT_line_list, CHOSEN_PATH_points,
      CHOSEN_PATH_line_list, PATH_points, PATH_line_list, GOAL_points,
      HITS_points, TAKEN_PATH_points, TAKEN_PATH_line_list, POSITION_point,
      unknowns;
  // Visualize each itteration

  node *currentNode = new node(position_x, position_y, position_z);
  currentNode->extractUnknownVoxelsInsideFrustum(myMap);

  if (!currentNode->unknowns_in_sight_.empty()) {

    unknowns.header.frame_id = map_frame_id_;
    unknowns.ns = "points";
    unknowns.action = visualization_msgs::Marker::ADD;
    unknowns.pose.orientation.w = 1.0;
    unknowns.id = 0;
    unknowns.type = visualization_msgs::Marker::CUBE_LIST;
    unknowns.scale.x = 0.2;
    unknowns.scale.y = 0.2;
    unknowns.scale.z = 0.2;
    unknowns.color.r = 1.0;
    unknowns.color.g = 1.0;
    unknowns.color.b = 1.0;
    unknowns.color.a = 0.6;
    std::vector<ufo::math::Vector3>::iterator it_comeon_visualizer22;
    for (it_comeon_visualizer22 = currentNode->unknowns_in_sight_.begin();
         it_comeon_visualizer22 != currentNode->unknowns_in_sight_.end();
         it_comeon_visualizer22++) {
      geometry_msgs::Point p;
      p.x = (*it_comeon_visualizer22).x();
      p.y = (*it_comeon_visualizer22).y();
      p.z = (*it_comeon_visualizer22).z();
      unknowns.points.push_back(p);
    }
    unknowns_pub->publish(unknowns);
  }

  // if (position_received) {
  //   POSITION_point.header.frame_id = map_frame_id_;
  //   POSITION_point.ns = "points";
  //   POSITION_point.action = visualization_msgs::Marker::ADD;
  //   POSITION_point.pose.orientation.w = 1.0;
  //   POSITION_point.id = 0;
  //   POSITION_point.type = visualization_msgs::Marker::SPHERE;
  //   POSITION_point.scale.x = 2*robot_size_;
  //   POSITION_point.scale.y = 2*robot_size_;
  //   POSITION_point.scale.z = 2*robot_size_;
  //   POSITION_point.color.g = 1.0f;
  //   POSITION_point.color.a = 0.06;
  //   geometry_msgs::Point p;
  //   p.x = position_x;
  //   p.y = position_y;
  //   p.z = position_z;
  //   POSITION_point.points.push_back(p);
  //   position_pub->publish(POSITION_point);
  // }

  // Visualize only once
  if (visualizeNewData) {
    if (RRT_created) {
      RRT_points.header.frame_id = RRT_line_list.header.frame_id =
          map_frame_id_;
      RRT_points.ns = "points";
      RRT_points.action = visualization_msgs::Marker::ADD;
      RRT_points.pose.orientation.w = 1.0;
      RRT_points.id = 0;
      RRT_line_list.id = 1;
      RRT_points.type = visualization_msgs::Marker::POINTS;
      RRT_line_list.type = visualization_msgs::Marker::LINE_LIST;
      RRT_points.scale.x = 0.2;
      RRT_points.scale.y = 0.2;
      RRT_line_list.scale.x = 0.1;
      RRT_points.color.g = 1.0f;
      RRT_points.color.a = 1.0;
      RRT_line_list.color.b = 1.0;
      RRT_line_list.color.a = 1.0;
      RRT_points.lifetime = ros::Duration(4.0);
      RRT_line_list.lifetime = ros::Duration(4.0);
      std::list<node *>::iterator it_comeon_visualizer;
      for (it_comeon_visualizer = RRT_TREE.begin();
           it_comeon_visualizer != RRT_TREE.end(); it_comeon_visualizer++) {
        geometry_msgs::Point p;
        p.x = (*it_comeon_visualizer)->point->x();
        p.y = (*it_comeon_visualizer)->point->y();
        p.z = (*it_comeon_visualizer)->point->z();
        RRT_points.points.push_back(p);
        if ((*it_comeon_visualizer)->myParent != nullptr) {
          RRT_line_list.points.push_back(p);
          p.x = (*it_comeon_visualizer)->myParent->point->x();
          p.y = (*it_comeon_visualizer)->myParent->point->y();
          p.z = (*it_comeon_visualizer)->myParent->point->z();
          RRT_line_list.points.push_back(p);
        }
      }
      points_pub->publish(RRT_points);
      points_pub->publish(RRT_line_list);
    }
    if (!CHOSEN_PATH.empty()) {
      nav_msgs::Path chosen_path;
      chosen_path.header.frame_id = map_frame_id_;
      chosen_path.poses.clear();
      for (auto i = CHOSEN_PATH.begin(); i != CHOSEN_PATH.end(); i++) {
        geometry_msgs::PoseStamped chosen_pose;
        chosen_pose.header.frame_id = map_frame_id_;
        chosen_pose.pose.position.x = (*i)->point->x();
        chosen_pose.pose.position.y = (*i)->point->y();
        chosen_pose.pose.position.z = (*i)->point->z();
        chosen_path.poses.push_back(chosen_pose);
      }
      output_path_pub->publish(chosen_path);
    }
    if (RRT_created) {

      PATH_points.header.frame_id = PATH_line_list.header.frame_id =
          map_frame_id_;
      PATH_points.ns = "points";
      PATH_points.action = visualization_msgs::Marker::ADD;
      PATH_points.pose.orientation.w = 1.0;
      PATH_points.id = 0;
      PATH_line_list.id = 1;
      PATH_points.type = visualization_msgs::Marker::POINTS;
      PATH_line_list.type = visualization_msgs::Marker::LINE_LIST;
      PATH_points.scale.x = 0.2;
      PATH_points.scale.y = 0.2;
      PATH_line_list.scale.x = 0.1;
      PATH_points.color.g = 1.0f;
      PATH_points.color.a = 1.0;
      PATH_line_list.color.g = 1.0;
      PATH_line_list.color.a = 1.0;
      PATH_line_list.lifetime = ros::Duration(6.0);
      PATH_points.lifetime = ros::Duration(6.0);
      std::list<node *>::iterator it_comeon_visualizer5;
      ALL_PATH.clear();
      for (it_comeon_visualizer5 = myGoals.begin();
           it_comeon_visualizer5 != myGoals.end(); it_comeon_visualizer5++) {
        (*it_comeon_visualizer5)->getPath(&ALL_PATH);
        ALL_PATH.push_back((*it_comeon_visualizer5));
      }
      std::list<node *>::iterator it_comeon_visualizer6;
      for (it_comeon_visualizer6 = ALL_PATH.begin();
           it_comeon_visualizer6 != std::prev(ALL_PATH.end(), 1);
           it_comeon_visualizer6++) {
        geometry_msgs::Point p;
        geometry_msgs::Point p1;
        p.x = (*it_comeon_visualizer6)->point->x();
        p.y = (*it_comeon_visualizer6)->point->y();
        p.z = (*it_comeon_visualizer6)->point->z();
        auto temp_it = std::next(it_comeon_visualizer6, 1);
        p1.x = (*temp_it)->point->x();
        p1.y = (*temp_it)->point->y();
        p1.z = (*temp_it)->point->z();

        PATH_points.points.push_back(p);
        if (temp_it != std::prev(ALL_PATH.end(), 0)) {
          PATH_line_list.points.push_back(p);
          PATH_line_list.points.push_back(p1);
        }
      }
      // all_path_pub->publish(PATH_points);
      all_path_pub->publish(PATH_line_list);
    }
    if (GOALS_generated) {
      GOAL_points.header.frame_id = map_frame_id_;
      GOAL_points.ns = "points";
      GOAL_points.action = visualization_msgs::Marker::ADD;
      GOAL_points.pose.orientation.w = 1.0;
      GOAL_points.id = 0;
      GOAL_points.type = visualization_msgs::Marker::SPHERE_LIST;
      GOAL_points.scale.x = 0.2;
      GOAL_points.scale.y = 0.2;
      GOAL_points.scale.z = 0.2;
      GOAL_points.color.g = 0.8;
      GOAL_points.color.r = 0.8;
      GOAL_points.color.a = 0.8;
      std::list<node *>::iterator it_comeon_visualizer3;
      for (it_comeon_visualizer3 = myGoals.begin();
           it_comeon_visualizer3 != myGoals.end(); it_comeon_visualizer3++) {
        geometry_msgs::Point p;
        p.x = (*it_comeon_visualizer3)->point->x();
        p.y = (*it_comeon_visualizer3)->point->y();
        p.z = (*it_comeon_visualizer3)->point->z();
        GOAL_points.points.push_back(p);
      }
      goal_pub->publish(GOAL_points);
    }
    if (goalNode != nullptr) {

      // hits.clear();
      HITS_points.header.frame_id = map_frame_id_;
      HITS_points.ns = "points";
      HITS_points.action = visualization_msgs::Marker::ADD;
      HITS_points.pose.orientation.w = 1.0;
      HITS_points.id = 0;
      HITS_points.type = visualization_msgs::Marker::POINTS;
      HITS_points.scale.x = 0.2;
      HITS_points.scale.y = 0.2;
      HITS_points.color.r = 1.0f;
      HITS_points.color.a = 1.0;
      HITS_points.lifetime = ros::Duration(8.0);
      std::list<ufo::math::Vector3>::iterator it_comeon_visualizer4;

      for (it_comeon_visualizer4 = goalNode->myHits.begin();
           it_comeon_visualizer4 != goalNode->myHits.end();
           it_comeon_visualizer4++) {
        geometry_msgs::Point p;
        p.x = it_comeon_visualizer4->x();
        p.y = it_comeon_visualizer4->y();
        p.z = it_comeon_visualizer4->z();
        HITS_points.points.push_back(p);
      }
      hits_pub->publish(HITS_points);
    }
    if (goalNode != nullptr) {
      TAKEN_PATH_points.header.frame_id = TAKEN_PATH_line_list.header.frame_id =
          map_frame_id_;
      TAKEN_PATH_points.ns = "points";
      TAKEN_PATH_line_list.ns = "lines";
      TAKEN_PATH_points.action = TAKEN_PATH_line_list.action =
          visualization_msgs::Marker::ADD;
      TAKEN_PATH_points.pose.orientation.w = 1.0;
      TAKEN_PATH_line_list.pose.orientation.w = 1.0;
      TAKEN_PATH_points.id = 0;
      TAKEN_PATH_line_list.id = 0;
      TAKEN_PATH_points.type = visualization_msgs::Marker::SPHERE_LIST;
      TAKEN_PATH_line_list.type = visualization_msgs::Marker::LINE_LIST;
      TAKEN_PATH_points.scale.x = 0.1;
      TAKEN_PATH_line_list.scale.x = 0.1;
      TAKEN_PATH_points.scale.y = 0.1;
      TAKEN_PATH_line_list.scale.y = 0.1;
      TAKEN_PATH_points.color.b = 0;
      TAKEN_PATH_points.color.g = 0;
      TAKEN_PATH_points.color.r = 0.7;
      TAKEN_PATH_points.color.a = 0.8;
      TAKEN_PATH_line_list.color.b = 0;
      TAKEN_PATH_line_list.color.g = 0;
      TAKEN_PATH_line_list.color.r = 0.7;
      TAKEN_PATH_line_list.color.a = 0.8;
      std::list<node *>::iterator taken_path_visualizer;

      // for (taken_path_visualizer = goalNode->myParents.begin();
      //      taken_path_visualizer != goalNode->myParents.end();
      for (taken_path_visualizer = VISITED_POINTS.begin();
           taken_path_visualizer != VISITED_POINTS.end();
           taken_path_visualizer++) {
        geometry_msgs::Point p;
        p.x = (*taken_path_visualizer)->point->x();
        p.y = (*taken_path_visualizer)->point->y();
        p.z = (*taken_path_visualizer)->point->z();
        TAKEN_PATH_points.points.push_back(p);

        if ((*taken_path_visualizer)->myParent != nullptr) {
          TAKEN_PATH_line_list.points.push_back(p);
          p.x = (*taken_path_visualizer)->myParent->point->x();
          p.y = (*taken_path_visualizer)->myParent->point->y();
          p.z = (*taken_path_visualizer)->myParent->point->z();
          TAKEN_PATH_line_list.points.push_back(p);
        }
      }
      taken_path_pub->publish(TAKEN_PATH_points);
      taken_path_pub->publish(TAKEN_PATH_line_list);
    }
    ufomap_msgs::UFOMapStamped::Ptr msg(new ufomap_msgs::UFOMapStamped);
    bool compress = false;
    ufo::map::DepthType pub_depth = 0;
    if (ufomap_msgs::ufoToMsg(myMap, msg->map, compress, pub_depth)) {
      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = map_frame_id_;
      map_pub->publish(msg);
    } else {
      std::cout << "Map conversion failed!" << std::endl;
    }
    visualizeNewData = false;
  }
}

// Evaluates and, if necessary, adds a new node to the path taken.
// The path taken is used during visualization and in the global strategy.
void updatePathTaken() {
  if (VISITED_POINTS.empty() and position_received) {
    node *myNode = new node(position_x, position_y, position_z);
    VISITED_POINTS.push_back(myNode);
  } else {
    if (position_received) {
      std::list<node *>::iterator taken_path_visualizer;
      taken_path_visualizer = --VISITED_POINTS.end();
      if (sqrt(pow((*taken_path_visualizer)->point->x() - position_x, 2) +
               pow((*taken_path_visualizer)->point->y() - position_y, 2) +
               pow((*taken_path_visualizer)->point->z() - position_z, 2)) >=
          0.2) {
        node *myNode = new node(position_x, position_y, position_z);
        (*taken_path_visualizer)->addParent(myNode);
        VISITED_POINTS.push_back(myNode);
      }
    }
  }
}

// Tunes the generation.
// This sets the bounding box which represents the local space.
// The bounding box will be reduced in size as much as possible without losing
// free space.
void tuneGeneration(ufo::map::OccupancyMapColor const &map, bool occupied_space,
                    bool free_space, bool unknown_space, float given_x,
                    float given_y, float given_z,
                    ufo::map::DepthType min_depth = planning_depth_) {
  highest_x = -9999;
  highest_y = -9999;
  highest_z = -9999;
  lowest_x = std::numeric_limits<float>::max();
  lowest_y = std::numeric_limits<float>::max();
  lowest_z = std::numeric_limits<float>::max();
  ufo::math::Vector3 minPoint(given_x - 1 * (v_local_ / 2),
                              given_y - 1 * (v_local_ / 2),
                              given_z - 1 * (v_local_ / 2));
  ufo::math::Vector3 maxPoint(given_x + 1 * (v_local_ / 2),
                              given_y + 1 * (v_local_ / 2),
                              given_z + 1 * (v_local_ / 2));
  ufo::geometry::AABB aabb(minPoint, maxPoint);
  for (auto it = map.beginLeaves(aabb, occupied_space, free_space,
                                 unknown_space, false, min_depth),
            it_end = map.endLeaves();
       it != it_end; ++it) {
    if (it.getX() > highest_x) {
      highest_x = it.getX();
    }
    if (it.getX() < lowest_x) {
      lowest_x = it.getX();
    }
    if (it.getY() > highest_y) {
      highest_y = it.getY();
    }
    if (it.getY() < lowest_y) {
      lowest_y = it.getY();
    }
    if (it.getZ() > highest_z) {
      highest_z = it.getZ();
    }
    if (it.getZ() < lowest_z) {
      lowest_z = it.getZ();
    }
  }
  SCALER_X = highest_x - lowest_x;
  SCALER_Y = highest_y - lowest_y;
  SCALER_Z = highest_z - lowest_z;
}

bool isInCollision(ufo::map::OccupancyMapColor const &map,
                   ufo::geometry::BoundingVar const &bounding_volume,
                   bool occupied_space = false, bool free_space = false,
                   bool unknown_space = false,
                   ufo::map::DepthType min_depth = planning_depth_) {
  // Iterate through all leaf nodes that intersects the bounding volume
  for (auto it = map.beginLeaves(bounding_volume, occupied_space, free_space,
                                 unknown_space, false, min_depth),
            it_end = map.endLeaves();
       it != it_end; ++it) {
    // Is in collision since a leaf node intersects the bounding volume.
    return true;
  }
  // No leaf node intersects the bounding volume.
  return false;
}

// Evaluates the RRT tree for the purpose of reducing the distance in a path.
// The node with the shortest path to the root node, which the goal in question
// can see, will be the parent of the currently evaluated goal node. The path
// between the goal node and its' parent has to pass a series of sphere checks,
// which aims to guarantee a distance of robot_size_ between the path and the
// occupied and unknown space.
void findShortestPath() {

  for (std::list<node *>::iterator it_goals = myGoals.begin();
       it_goals != myGoals.end(); it_goals++) {
    struct node *chosenNode = nullptr;
    double distance = std::numeric_limits<double>::max();
    for (std::list<node *>::iterator it_RRT = RRT_TREE.begin();
         it_RRT != RRT_TREE.end(); it_RRT++) {
      double distanceNodeToGoal =
          sqrt(pow((*it_RRT)->point->x() - (*it_goals)->point->x(), 2) +
               pow((*it_RRT)->point->y() - (*it_goals)->point->y(), 2) +
               pow((*it_RRT)->point->z() - (*it_goals)->point->z(), 2));
      double distanceToNode = (*it_RRT)->sumDistance();
      if (distanceNodeToGoal < goal_connect_dist_) {

        double totalDistance = distanceNodeToGoal + distanceToNode;
        if (totalDistance < distance) {

          ufo::geometry::OBB obb =
              makeOBB(*((*it_RRT)->point), *((*it_goals)->point), robot_size_);

          if (!isInCollision(myMap, obb, true, false, true, planning_depth_)) {
            bool add = true;

            if (isInCollision(myMap, obb, true, false, true, planning_depth_)) {
              add = false;
              break;
            }
            if (add) {

              distance = totalDistance;
              chosenNode = *it_RRT;
            }
          }
        }
      }
    }
    if (chosenNode != nullptr) {
      (*it_goals)->addParent(chosenNode);
      chosenNode->addChild(*it_goals);
    }
  }
}

// Generates goals.
// The goals generated guarantees at least one piece of new information within
// sensor_range_, as well as being in free space and at least robot_size_
// distance away from both unknown and occupied space.
void generateGoals(ufo::map::OccupancyMapColor const &map,
                   bool evaluateOldGoals) {
  if (!myGoals.empty() and evaluateOldGoals) {
    double newCost = std::numeric_limits<float>::max();
    double totalCost = std::numeric_limits<float>::max();
    for (std::list<node *>::iterator it_goal = myGoals.begin();
         it_goal != myGoals.end(); it_goal++) {
      if ((*it_goal)->myParent != nullptr) {
        (*it_goal)->clearInformationGain();
        double informationGain =
            k_info_ *
            ((*it_goal)->findInformationGain(
                v_local_, SENSOR_HORIZONTAL, sensor_vertical_fov_,
                min_sensor_range_, sensor_range_, myMap, true, false));
        newCost = -informationGain;
        if (informationGain > initialGoalInfo) {
          initialGoalInfo = informationGain;
        }
        int stickyCounter = 0;
        for (std::list<ufo::math::Vector3>::iterator it_floor =
                 (*it_goal)->myHits.begin();
             it_floor != (*it_goal)->myHits.end(); it_floor++) {
          if (it_floor->z() < (*it_goal)->point->z()) {
            stickyCounter++;
          }
        }
        bool infoRequirement =
            ((*it_goal)->myHits.size() > GLOBAL_PATH_THRESHOLD);
        bool stickyFloor = ((stickyCounter < 0.8 * (*it_goal)->myHits.size()));
        if ((newCost < totalCost) and
            ((*it_goal)->findInformationGain(
                 v_local_, SENSOR_HORIZONTAL, sensor_vertical_fov_,
                 min_sensor_range_, sensor_range_, myMap, false, false) > 0) and
            (stickyFloor and infoRequirement) and
            (*it_goal)->myParent != nullptr) {
          totalCost = newCost;
          reserveGoalNode = *it_goal;
        }
      }
    }
    if (reserveGoalNode != nullptr) {
      myReserveGoals.push_back(reserveGoalNode);
      for (std::list<node *>::iterator it_parent_finder =
               --VISITED_POINTS.end();
           it_parent_finder != VISITED_POINTS.begin(); it_parent_finder--) {
        ufo::geometry::LineSegment myLine((*(*it_parent_finder)->point),
                                          (*reserveGoalNode->point));
        if (!isInCollision(map, myLine, true, false, true, planning_depth_)) {
          reserveGoalNode->addParent((*it_parent_finder));
          break;
        }
      }
    }
  }
  if (goalNode != nullptr) {
    if (sqrt(pow(position_x - goalNode->point->x(), 2) +
             pow(position_y - goalNode->point->y(), 2) +
             pow(position_z - goalNode->point->z(), 2)) > path_update_dist_) {
      std::list<node *>::iterator it_goal2;
      int help_counter = 0;
      for (it_goal2 = myGoals.begin(); it_goal2 != myGoals.end(); it_goal2++) {
        help_counter++;
        if (*it_goal2 != goalNode and *it_goal2 != reserveGoalNode) {
          (*it_goal2)->readyForDeletion();
          delete (*it_goal2);
        }
      }
      myGoals.clear();
      myGoals.push_back(goalNode);
    } else {
      std::list<node *>::iterator it_goal2;
      for (it_goal2 = myGoals.begin(); it_goal2 != myGoals.end(); it_goal2++) {
        if (*it_goal2 != reserveGoalNode) {
          (*it_goal2)->readyForDeletion();
          delete (*it_goal2);
        }
      }
      goalNode = nullptr;
      reserveGoalNode = nullptr;
      myGoals.clear();
      allowNewPath = true;
    }
  }
  ufo::math::Vector3 goal;
  srand(time(0));
  itterations = 0;
  while ((myGoals.size() < number_of_goals_) and (itterations < 20000)) {
    float x = lowest_x + abs(1024 * rand() / (RAND_MAX + 1.0)) * SCALER_X;
    float y = lowest_y + abs(1024 * rand() / (RAND_MAX + 1.0)) * SCALER_Y;
    float z = lowest_z + abs(1024 * rand() / (RAND_MAX + 1.0)) * SCALER_Z;
    node goal = node(x, y, z);
    ufo::geometry::Sphere goal_sphere(*(goal.point), robot_size_);
    if (sqrt(pow(position_x - x, 2) + pow(position_y - y, 2) +
             pow(position_z - z, 2)) > min_dist_to_goal_) {
      if (!isInCollision(myMap, goal_sphere, true, false, true,
                         planning_depth_) and
          isInCollision(myMap, goal_sphere, false, true, false,
                        planning_depth_)) {
        bool add = true;
        for (std::list<node *>::iterator it_goal = myGoals.begin();
             it_goal != myGoals.end(); it_goal++) {
          if (sqrt(pow((*it_goal)->point->x() - x, 2) +
                   pow((*it_goal)->point->y() - y, 2) +
                   pow((*it_goal)->point->z() - z, 2)) < dist_goals_) {
            add = false;
            break;
          }
        }
        if (add) {

          int foundInfo;

          node *newGoal = new node(x, y, z);
          myGoals.push_back(newGoal);
        }
      };
      itterations++;
    }
  };
  if (myGoals.size() == number_of_goals_) {
    std::cout << "Goals generated successfully\n" << std::endl;
    GOALS_generated = true;
  } else if (myGoals.size() == 0) {
    std::cout << "No goals found, trying again soon" << std::endl;
    sleep_for(microseconds(100000));
  } else {
    std::cout << "Only " << myGoals.size() << " goals found" << std::endl;
    GOALS_generated = true;
  }
};

// Evaluates and sets the current path
// The evaluation takes the distance cost, information gain and the distance
// cost in mind, choosing the path with the overall smallest cost. The
// evaluation depends heavily on the values of k_dist_,
// k_info_ and k_u_
void setPath() {
  bool setDistance = false;
  if (goalNode != nullptr) {
    goalNode->clearInformationGain();

    if ((sqrt(pow(position_x - goalNode->point->x(), 2) +
              pow(position_y - goalNode->point->y(), 2) +
              pow(position_z - goalNode->point->z(), 2)) < path_update_dist_)) {
      allowNewPath = true;
      totalCost = std::numeric_limits<float>::max();
    } else {
      totalCost =
          goalNode->sumDistance() * k_dist_ -
          k_info_ * (goalNode->findInformationGain(
                        v_local_, SENSOR_HORIZONTAL, sensor_vertical_fov_,
                        min_sensor_range_, sensor_range_, myMap, true, false));
    }
  } else {
    totalCost = std::numeric_limits<float>::max();
  }
  double newCost = std::numeric_limits<float>::max();
  if (allowNewPath) {
    std::list<double> PATH_CONTAINER{};
    initialGoalInfo = 0;

    for (std::list<node *>::iterator it_goal = myGoals.begin();
         it_goal != myGoals.end(); it_goal++) {

      if (*it_goal != nullptr) {
        (*it_goal)->addParents();
      }

      if ((*it_goal)->myParent != nullptr) {

        auto pathImprovement_start = high_resolution_clock::now();

        (*it_goal)->findPathImprovement(*it_goal, myMap, dist_nodes_,
                                        robot_size_, pathImprovement_start,
                                        path_improvement_max_);

        linSpace(*it_goal, dist_nodes_);
        auto pathImprovement_start_2 = high_resolution_clock::now();
        (*it_goal)->findPathImprovement(*it_goal, myMap, dist_nodes_,
                                        robot_size_, pathImprovement_start_2,
                                        path_improvement_max_);

        linSpace(*it_goal, dist_nodes_);
        double informationGain =
            k_info_ *
            ((*it_goal)->findInformationGain(
                v_local_, SENSOR_HORIZONTAL, sensor_vertical_fov_,
                min_sensor_range_, sensor_range_, myMap, false, false));

        double distanceCost = (*it_goal)->sumDistance() * k_dist_;

        typedef rrtCache *(*arbitrary)();
        typedef rrtSolverStatus (*arbitrary2)(void *, double *, double *,
                                              double, double *);
        typedef void (*rrt_clearer)(rrtCache *);
        int i = 0;
        double p[159] = {0};
        std::list<double> xref = {};
        std::list<double> x0 = {position_x, position_y, position_z, velocity_x,
                                velocity_y, velocity_z, roll,       pitch};

        // Current position
        p[0] = position_x;
        p[1] = position_y;
        p[2] = position_z;
        p[3] = velocity_x;
        p[4] = velocity_y;
        p[5] = velocity_z;
        p[6] = roll;
        p[7] = pitch;

        // Trajectory
        std::list<struct node *> EVALUATE_PATH{};
        EVALUATE_PATH.clear();
        (*it_goal)->getPath(&EVALUATE_PATH);
        EVALUATE_PATH.push_back(new node((*it_goal)->point->x(),
                                         (*it_goal)->point->y(),
                                         (*it_goal)->point->z()));
        xref.push_back((*it_goal)->point->x());
        xref.push_back((*it_goal)->point->y());
        xref.push_back((*it_goal)->point->z());
        std::list<node *>::iterator it_evaluator = EVALUATE_PATH.begin();
        for (i = 1; i < nmpc_horizon_; ++i) {
          p[8 + 3 * i] = (*it_evaluator)->point->x();
          xref.push_back((*it_evaluator)->point->x());
          p[8 + 3 * i + 1] = (*it_evaluator)->point->y();
          xref.push_back((*it_evaluator)->point->y());
          p[8 + 3 * i + 2] = (*it_evaluator)->point->z();
          xref.push_back((*it_evaluator)->point->z());
          if (it_evaluator != --EVALUATE_PATH.end()) {
            it_evaluator++;
          }
        }
        p[158] = nmpc_dt_;

        double u[150] = {0};

        for (i = 0; i < nmpc_horizon_; ++i) {
          u[3 * i] = 0;
          u[3 * i + 1] = 0;
          u[3 * i + 2] = 0;
        }

        double init_penalty = 1;
        void *handle = dlopen((ros::package::getPath("errt") +
                               "/MAV/rrt/target/release/librrt.so")
                                  .c_str(),
                              RTLD_LAZY);
        if (!handle) {
          fprintf(stderr, "%s\n", dlerror());
          exit(EXIT_FAILURE);
        }
        arbitrary rrt_new;
        *(void **)(&rrt_new) = dlsym(handle, "rrt_new");
        rrtCache *cache = rrt_new();
        arbitrary2 rrt_solve;
        *(void **)(&rrt_solve) = dlsym(handle, "rrt_solve");
        rrt_clearer rrt_free;
        *(void **)(&rrt_free) = dlsym(handle, "rrt_free");

        ROS_INFO_STREAM(" Calculating .. " << init_penalty);

        rrtSolverStatus status = rrt_solve(cache, u, p, 0, &init_penalty);

        std::list<double> uold = {9.81, 0.0, 0.0};
        std::list<double> uref = {9.81, 0.0, 0.0};

        std::list<double> x_hist;
        std::list<double> p_hist;
        double cost;
        std::tuple<std::list<double>, double, std::list<double>> trajectory(
            std::list<double> x, double *u, double N, double dt,
            std::list<double> nmpc_ref);
        std::tie(x_hist, cost, p_hist) =
            trajectory(x0, u, nmpc_horizon_, nmpc_dt_, xref);
        xref.clear();
        rrt_free(cache);
        double actuationCost = k_u_ * cost;
        newCost = distanceCost - informationGain + actuationCost;
        ////////////////////////////////////////////////////////////////

        ROS_DEBUG_STREAM("\n Information gain : " << informationGain << "\n");
        ROS_DEBUG_STREAM(" Distance cost : " << distanceCost << "\n");
        ROS_DEBUG_STREAM(" Actuation cost : " << actuationCost << "\n");

        ROS_DEBUG_STREAM("\n ---------------- \n");

        std::list<double> new_p_hist;
        for (auto i = p_hist.begin(); i != p_hist.end(); i++) {
          if (std::distance(p_hist.begin(), i) / 3 <=
              EVALUATE_PATH.size() + 1) {
            new_p_hist.push_back(*i);
          }
        }

        if (informationGain > initialGoalInfo) {
          initialGoalInfo = informationGain;
        }
        int stickyCounter = 0;
        for (std::list<ufo::math::Vector3>::iterator it_floor =
                 (*it_goal)->myHits.begin();
             it_floor != (*it_goal)->myHits.end(); it_floor++) {
          if (it_floor->z() < (*it_goal)->point->z()) {
            stickyCounter++;
          }
        }
        bool infoRequirement =
            ((*it_goal)->myHits.size() > GLOBAL_STRATEGY_THRESHOLD);
        bool stickyFloor = ((stickyCounter < 0.8 * (*it_goal)->myHits.size()));
        if ((newCost < totalCost) and
            ((*it_goal)->findInformationGain(
                 v_local_, SENSOR_HORIZONTAL, sensor_vertical_fov_,
                 min_sensor_range_, sensor_range_, myMap, false, false) > 0) and
            allowNewPath and (stickyFloor and infoRequirement) and
            (*it_goal)->myParent != nullptr) {
          totalCost = newCost;
          goalNode = *it_goal;
          newPath = true;
          PATH_CONTAINER.clear();
          for (std::list<double>::iterator path_itterator_helper =
                   new_p_hist.begin();
               path_itterator_helper != new_p_hist.end();) {
            double x = *path_itterator_helper;
            PATH_CONTAINER.push_back(x);
            path_itterator_helper++;
            double y = *path_itterator_helper;
            PATH_CONTAINER.push_back(y);
            path_itterator_helper++;
            double z = *path_itterator_helper;
            PATH_CONTAINER.push_back(z);
            path_itterator_helper++;
          }
          if (EVALUATE_PATH.size() > nmpc_horizon_) {
            std::list<node *>::iterator path_itterator_helper2 =
                EVALUATE_PATH.begin();
            std::advance(path_itterator_helper2, nmpc_horizon_);
            while (path_itterator_helper2 != EVALUATE_PATH.end()) {
              PATH_CONTAINER.push_back((*path_itterator_helper2)->point->x());
              PATH_CONTAINER.push_back((*path_itterator_helper2)->point->y());
              PATH_CONTAINER.push_back((*path_itterator_helper2)->point->z());
              path_itterator_helper2++;
            }
          }
          CHOSEN_PATH_VREF.clear();
          for (std::list<double>::iterator reference_itterator_helper =
                   x_hist.begin();
               reference_itterator_helper != x_hist.end();
               reference_itterator_helper++) {
            CHOSEN_PATH_VREF.push_back(*reference_itterator_helper);
          }
          vref_itterator = CHOSEN_PATH_VREF.begin();
        }
      }
    }
    if (not recoveryUnderway) {
      std::list<node *>::iterator it_clear_helper;
      for (it_clear_helper = CHOSEN_PATH.begin();
           it_clear_helper != --CHOSEN_PATH.end(); it_clear_helper++) {
        (*it_clear_helper)->readyForDeletion();
        delete (*it_clear_helper);
      }
    } else {
      std::list<node *>::iterator it_clear_helper = --CHOSEN_PATH.end();
      (*it_clear_helper)->readyForDeletion();
      delete (*it_clear_helper);
      recoveryUnderway = false;
    }
    CHOSEN_PATH.clear();
    if (goalNode != nullptr and newPath) {
      setDistance = true;
      std::list<double>::iterator it_path_helper = PATH_CONTAINER.begin();
      for (int i = 0; i < (PATH_CONTAINER.size() / 3); i++) {
        double x = *it_path_helper;
        it_path_helper++;
        double y = *it_path_helper;
        it_path_helper++;
        double z = *it_path_helper;
        it_path_helper++;
        CHOSEN_PATH.push_back(new node(x, y, z));
      }
      PATH_CONTAINER.clear();
      CHOSEN_PATH.push_back(new node(goalNode->point->x(), goalNode->point->y(),
                                     goalNode->point->z()));
      fetched_path = true;
      visualizeNewData = true;
      allowNewPath = false;
      newPath = false;
      path_itterator = CHOSEN_PATH.begin();
      currentTarget = *path_itterator;
      advance_index = 0;
    }
  }
  if (setDistance) {
    totalDistance = goalNode->sumDistance();
  };
}

// Generates the RRT
// Generates the nodes in the RRT-tree and connects them to a already existing
// parent. The root of the node exists at the current position of the plant.
void generateRRT(float given_x, float given_y, float given_z) {
  // high_resolution_clock::time_point start_total =
  // high_resolution_clock::now();

  for (std::list<node *>::iterator it_clear_helper = RRT_TREE.begin();
       it_clear_helper != --RRT_TREE.end(); it_clear_helper++) {
    (*it_clear_helper)->readyForDeletion();
    delete (*it_clear_helper);
  }
  for (std::list<node *>::iterator it_clear_helper = myGoals.begin();
       it_clear_helper != --myGoals.end(); it_clear_helper++) {
    (*it_clear_helper)->addParent(nullptr);
  }
  RRT_TREE.clear();
  std::cout << "Building RRT-tree" << std::endl;
  node *origin = new node(given_x, given_y, given_z);
  origin->addParent(nullptr);
  RRT_TREE.push_back(origin);
  srand(time(0));
  itterations = 0;
  while (((RRT_TREE.size() <= number_of_nodes_ and run_by_nodes_) or
          (itterations <= number_of_iterations_ and !run_by_nodes_)) and
         itterations < 100000) {
    // Generate a random point
    float x = lowest_x + abs(1024 * rand() / (RAND_MAX + 1.0)) * SCALER_X;
    float y = lowest_y + abs(1024 * rand() / (RAND_MAX + 1.0)) * SCALER_Y;
    float z = lowest_z + abs(1024 * rand() / (RAND_MAX + 1.0)) * SCALER_Z;
    ufo::math::Vector3 random_point(x, y, z);
    ufo::geometry::Sphere point_sphere(random_point, robot_size_);
    if (!isInCollision(myMap, point_sphere, true, false, true,
                       planning_depth_) and
        isInCollision(myMap, point_sphere, false, true, false,
                      planning_depth_)) {
      float distance = std::numeric_limits<float>::max();
      node *parent;
      std::list<node *>::iterator it_node;
      for (it_node = RRT_TREE.begin(); it_node != RRT_TREE.end(); it_node++) {
        ufo::math::Vector3 direction = random_point - *((*it_node)->point);
        double new_distance = abs(direction.norm());
        if (new_distance < distance) {
          distance = new_distance;
          parent = *it_node;
        }
      };
      ufo::math::Vector3 start_point(parent->point->x(), parent->point->y(),
                                     parent->point->z());
      ufo::geometry::OBB obb = makeOBB(start_point, random_point, robot_size_);
      if (!isInCollision(myMap, obb, true, false, true, planning_depth_)) {
        node *new_node = new node(x, y, z);
        new_node->addParent(parent);
        parent->addChild(new_node);
        RRT_TREE.push_back(new_node);
      }
    };
    itterations++;
  };
  std::cout << "RRT-tree built successfully" << std::endl;

  if (run_by_nodes_) {
    std::cout << "Verifying tree" << std::endl;
    int total_childs = 0;
    int total_parents = 0;
    std::list<node *>::iterator it_comeon;
    for (it_comeon = RRT_TREE.begin(); it_comeon != RRT_TREE.end();
         it_comeon++) {
      total_childs = total_childs + (*it_comeon)->myChilds.size();
      if ((*it_comeon)->myParent != nullptr) {
        total_parents++;
      }
    };
    if (total_childs != 0) {
      RRT_created = true;
    }
    if (total_childs == number_of_nodes_) {
      std::cout << "All children accounted for" << std::endl;
    } else {
      std::cout << "Expected " << number_of_nodes_ << " children, but "
                << total_childs << " was found." << std::endl;
    };
    if (total_parents == number_of_nodes_) {
      std::cout << "All parents accounted for" << std::endl;
    } else {
      std::cout << "Expected " << number_of_nodes_ << " parents, but "
                << total_parents << " was found." << std::endl;
    };
  } else {
    std::cout << "Running by itterations, so the amount of nodes are unknown "
                 "and hence can't be verified"
              << std::endl;
  }
};

// Trajectory
std::tuple<std::list<double>, double, std::list<double>>
trajectory(std::list<double> x, double *u, double N, double dt,
           std::list<double> nmpc_ref) {
  // Calculate the dynamic costs based on selected weights
  double ns = 8;
  std::list<double> p_traj{};
  std::list<double> v_traj{};
  double cost = 0;
  // Weight matrices
  std::list<double> Qx = {
      position_tracking_weight_x_,
      position_tracking_weight_y_,
      position_tracking_weight_z_,
      0,
      0,
      0,
      angle_weight_roll_,
      angle_weight_pitch_}; // Position tracking weights x, y, z, 0, 0, 0, angle
  std::list<double> Ru = {
      input_weight_thrust_, input_weight_roll_,
      input_weight_pitch_}; // input weights (Thrust, roll, pitch)
  std::list<double> Rd = {
      input_rate_weight_thrust_, input_rate_weight_roll_,
      input_rate_weight_pitch_}; // input rate weights (thrust, roll, pitch)
  std::list<double> u_old = {9.81, 0, 0};
  std::list<double> u_ref = {9.81, 0.0, 0.0};
  std::list<double>::iterator x_ref_itterator = nmpc_ref.begin();
  for (int i = 0; i < N; i++) {
    std::list<double>::iterator Qx_itterator = Qx.begin();
    std::list<double>::iterator x_itterator = x.begin();

    // Setting up itterators
    std::list<double>::iterator Ru_itterator = Ru.begin();
    std::list<double>::iterator Rd_itterator = Rd.begin();
    std::list<double>::iterator u_old_itterator = u_old.begin();

    for (int j = 0; j < 3; j++) {
      cost = cost + *Ru_itterator * pow(u[3 * i + j] - *u_old_itterator, 2);
      cost = cost + *Rd_itterator * pow(u[3 * i + j] - *u_old_itterator, 2);
      Ru_itterator++;
      u_old_itterator++;
      Rd_itterator++;
    }
    u_old.clear();
    u_old = {u[3 * i], u[3 * i + 1], u[3 * i + 2]};

    x_itterator = x.begin();
    std::list<double>::iterator x2_itterator = x.begin();
    std::advance(x2_itterator, 3);
    for (int j = 0; j < 3; j++) {
      *x_itterator = *x_itterator + dt * *x2_itterator;
      x_itterator++;
      x2_itterator++;
    }
    std::list<double>::iterator x3_itterator = x.begin();
    std::advance(x3_itterator, 7); // x[7]
    *x_itterator = *x_itterator +
                   dt * (sin(*x3_itterator) * cos(*x2_itterator) * u[3 * i] -
                         1 * (*x_itterator));
    x_itterator++; // x[4]
    *x_itterator = *x_itterator +
                   dt * (-sin(*x2_itterator) * u[3 * i] - 1 * (*x_itterator));
    x_itterator++; // x[5]
    *x_itterator = *x_itterator +
                   dt * (cos(*x3_itterator) * cos(*x2_itterator) * u[3 * i] -
                         1 * *x_itterator - 9.81);
    x_itterator++; // x[6]
    *x_itterator =
        *x_itterator + dt * ((1.0 / 0.5) * (u[3 * i + 1] - *x_itterator));
    x_itterator++; // x[7]
    *x_itterator =
        *x_itterator + dt * ((1.0 / 0.5) * (u[3 * i + 2] - *x_itterator));
    std::advance(x_itterator, -7);

    for (int j = 0; j < 8; j++) {
      if (j < 3) {
        cost = cost +
               (*Qx_itterator) * pow((*x_itterator) - (*x_ref_itterator), 2);
        x_ref_itterator++;
      } else {
        cost = cost + (*Qx_itterator) * pow((*x_itterator), 2);
      }
      Qx_itterator++;
      x_itterator++;
    }

    x_itterator = x.begin();
    p_traj.push_back(*x_itterator);
    x_itterator++;
    p_traj.push_back(*x_itterator);
    x_itterator++;
    p_traj.push_back(*x_itterator);
    x_itterator++;
    v_traj.push_back(*x_itterator);
    x_itterator++;
    v_traj.push_back(*x_itterator);
    x_itterator++;
    v_traj.push_back(*x_itterator);
  }
  return std::make_tuple(v_traj, cost, p_traj);
}

// End of functions
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// // Callback functions

void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &msg) {
  if (ufomap_msgs::msgToUfo(msg->map, myMap)) {
    map_received = true;
  } else {
    std::cout << "Conversion failed" << std::endl;
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  position_received = true;
  position_x = msg->pose.pose.position.x;
  position_y = msg->pose.pose.position.y;
  position_z = msg->pose.pose.position.z;
  velocity_x = msg->twist.twist.linear.x;
  velocity_y = msg->twist.twist.linear.y;
  velocity_z = msg->twist.twist.linear.z;
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

// End of callback functions
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// // Main

typedef rrtCache *(*arbitrary)();
typedef rrtSolverStatus (*arbitrary2)(void *, double *, double *, double,
                                      double *);
typedef void (*rrt_clearer)(rrtCache *);

int main(int argc, char *argv[]) {
  // Subscribers and publishers
  ros::init(argc, argv, "errt");
  ros::NodeHandle nh;

  ros::Publisher unknowns_pub =
      nh.advertise<visualization_msgs::Marker>("UNKNOWN_NODES", 1);
  ros::Publisher points_pub =
      nh.advertise<visualization_msgs::Marker>("tree_expansion", 1);
  // ros::Publisher chosen_path_visualization_pub =
  // nh.advertise<visualization_msgs::Marker>("selected_branch",
  // 1);
  ros::Publisher output_path_pub =
      nh.advertise<nav_msgs::Path>("selected_trajectory", 1);
  ros::Publisher chosen_path_pub =
      nh.advertise<nav_msgs::Odometry>("reference_out_", 1);

  // change it to path msg
  ros::Publisher all_path_pub =
      nh.advertise<visualization_msgs::Marker>("candidate_branches", 1);
  ros::Publisher goal_pub =
      nh.advertise<visualization_msgs::Marker>("candidate_goals", 1);
  ros::Publisher map_pub =
      nh.advertise<ufomap_msgs::UFOMapStamped>("Internal_ufo_map", 11);
  ros::Subscriber map_sub = nh.subscribe("ufomap_in_", 1, mapCallback);
  ros::Subscriber sub = nh.subscribe("odometry_in_", 1, odomCallback);
  ros::Publisher hits_pub =
      nh.advertise<visualization_msgs::Marker>("predicted_info_gain", 1);
  ros::Publisher position_pub =
      nh.advertise<visualization_msgs::Marker>("robot_bounding_box", 1);

  // change to path msg
  ros::Publisher taken_path_pub =
      nh.advertise<visualization_msgs::Marker>("exploration_path", 1);
  ros::Publisher execution_time_pub =
      nh.advertise<std_msgs::Float64MultiArray>("errt_execution_time", 1);

  m_command_Path_Publisher =
      nh.advertise<nav_msgs::Path>("command_path", 1, true);

  m_trajectory_Publisher =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("path_out_", 1,
                                                             true);
  ros::Rate rate(10);

  // Initial point
  // This manually sets the first point which the drone will travel to.
  // For each point in the path CHOSEN_PATH, one needs to add the VREF vx, vy,
  // vz to CHOSEN_PATH_VREF.
  ros::param::get(ros::this_node::getName() + "/start_from_waypoint",
                  start_from_waypoint_);

  ros::param::get(ros::this_node::getName() + "/initial_x_", initial_x_);
  ros::param::get(ros::this_node::getName() + "/initial_y_", initial_y_);
  ros::param::get(ros::this_node::getName() + "/initial_z_", initial_z_);

  ros::param::get(ros::this_node::getName() + "/map_frame_id", map_frame_id_);

  ros::param::get(ros::this_node::getName() + "/run_by_nodes", run_by_nodes_);
  ros::param::get(ros::this_node::getName() + "/number_of_nodes",
                  number_of_nodes_);
  ros::param::get(ros::this_node::getName() + "/number_of_goals",
                  number_of_goals_);
  ros::param::get(ros::this_node::getName() + "/number_of_iterations",
                  number_of_iterations_);

  ros::param::get(ros::this_node::getName() + "/min_info_goal", min_info_goal_);
  ros::param::get(ros::this_node::getName() + "/goal_sensor_range_",
                  goal_sensor_range_);
  ros::param::get(ros::this_node::getName() + "/robot_size", robot_size_);
  ros::param::get(ros::this_node::getName() + "/goal_connect_dist",
                  goal_connect_dist_);

  ros::param::get(ros::this_node::getName() + "/dist_nodes", dist_nodes_);
  ros::param::get(ros::this_node::getName() + "/dist_goals", dist_goals_);
  ros::param::get(ros::this_node::getName() + "/min_dist_to_goal",
                  min_dist_to_goal_);
  ros::param::get(ros::this_node::getName() + "/planning_depth",
                  planning_depth_);

  ros::param::get(ros::this_node::getName() + "/v_local", v_local_);

  ros::param::get(ros::this_node::getName() + "/k_dist", k_dist_);
  ros::param::get(ros::this_node::getName() + "/k_info", k_info_);
  ros::param::get(ros::this_node::getName() + "/k_u", k_u_);
  ros::param::get(ros::this_node::getName() + "/path_update_dist",
                  path_update_dist_);
  ros::param::get(ros::this_node::getName() + "/recalc_dist", recalc_dist_);
  ros::param::get(ros::this_node::getName() + "/path_improvement_max",
                  path_improvement_max_);

  ros::param::get(ros::this_node::getName() + "/sensor_range", sensor_range_);
  ros::param::get(ros::this_node::getName() + "/min_sensor_range",
                  min_sensor_range_);
  ros::param::get(ros::this_node::getName() + "/senros_vertical_fov",
                  sensor_vertical_fov_);

  ros::param::get(ros::this_node::getName() + "/nmpc_horizon", nmpc_horizon_);
  ros::param::get(ros::this_node::getName() + "/nmpc_dt", nmpc_dt_);
  ros::param::get(ros::this_node::getName() + "/position_tracking_weight_x",
                  position_tracking_weight_x_);
  ros::param::get(ros::this_node::getName() + "/position_tracking_weight_y",
                  position_tracking_weight_y_);
  ros::param::get(ros::this_node::getName() + "/position_tracking_weight_z",
                  position_tracking_weight_z_);
  ros::param::get(ros::this_node::getName() + "/angle_weight_roll",
                  angle_weight_roll_);
  ros::param::get(ros::this_node::getName() + "/angle_weight_pitch",
                  angle_weight_pitch_);
  ros::param::get(ros::this_node::getName() + "/input_weight_thrust",
                  input_weight_thrust_);
  ros::param::get(ros::this_node::getName() + "/input_weight_roll",
                  input_weight_roll_);
  ros::param::get(ros::this_node::getName() + "/input_weight_pitch",
                  input_weight_pitch_);
  ros::param::get(ros::this_node::getName() + "/input_rate_weight_thrust",
                  input_rate_weight_thrust_);
  ros::param::get(ros::this_node::getName() + "/input_rate_weight_roll",
                  input_rate_weight_roll_);
  ros::param::get(ros::this_node::getName() + "/input_rate_weight_pitch",
                  input_rate_weight_pitch_);

  if (start_from_waypoint_) {
    // start_from_waypoint_ = false;
    ros::param::get(ros::this_node::getName() + "/initial_x", initial_x_);
    ros::param::get(ros::this_node::getName() + "/initial_y", initial_y_);
    ros::param::get(ros::this_node::getName() + "/initial_z", initial_z_);
    std::cout << "Init reference : " << initial_x_ << ", " << initial_y_ << ", "
              << initial_z_ << std::endl;
    CHOSEN_PATH.push_back(new node(initial_x_, initial_y_, initial_z_));
    fetched_path = true;
    RRT_created = true;
    GOALS_generated = true;
    position_received = true;
    allowNewPath = false;
    currentTarget = *CHOSEN_PATH.begin();
    goalNode = *CHOSEN_PATH.begin();
    CHOSEN_PATH_VREF.push_back(0);
    CHOSEN_PATH_VREF.push_back(0);
    CHOSEN_PATH_VREF.push_back(0);
    vref_itterator = CHOSEN_PATH_VREF.begin();

    generateTrajectory();
    publishTrajectory();
  }
  // Main
  // When the ufomap and current position have been received through their
  // respective callback functions, the rrt executes by: 1) Tuning the
  // generation and generating goals. 2) Generating the RRT tree and attatching
  // generated goals. 3) Set the path. 4) Evaluates the current point. 5)
  // Trigger global strategy if necessary.
  while (ros::ok()) {

    high_resolution_clock::time_point start_total =
        high_resolution_clock::now();

    if (map_received and not GOALS_generated and position_received) {

      if (CHOSEN_PATH.empty()) {

        tuneGeneration(myMap, false, true, false, position_x, position_y,
                       position_z, planning_depth_);
      } else {

        tuneGeneration(myMap, false, true, false,
                       (*(--CHOSEN_PATH.end()))->point->x(),
                       (*(--CHOSEN_PATH.end()))->point->y(),
                       (*(--CHOSEN_PATH.end()))->point->z(), planning_depth_);
      }

      high_resolution_clock::time_point start_total =
          high_resolution_clock::now();

      generateGoals(myMap, true);
    }

    if (map_received and not RRT_created and GOALS_generated) {

      if (CHOSEN_PATH.empty()) {

        generateRRT(position_x, position_y, position_z);
      } else {

        high_resolution_clock::time_point start_total =
            high_resolution_clock::now();

        generateRRT((*(--CHOSEN_PATH.end()))->point->x(),
                    (*(--CHOSEN_PATH.end()))->point->y(),
                    (*(--CHOSEN_PATH.end()))->point->z());
      }

      if (RRT_created) {

        findShortestPath();
      }

      itterations = 0; // DO NOT TOUCH!
    }

    if (map_received and RRT_created) {

      if (!fetched_path) {

        high_resolution_clock::time_point start_total =
            high_resolution_clock::now();

        setPath();

        generateTrajectory();

        publishTrajectory();
      }

      if (fetched_path and goalNode != nullptr) {

        itterations++;

        evaluateCurrentPoint(&chosen_path_pub);
      }

      if ((goalNode == nullptr and GOALS_generated)) {
        if (initialGoalInfo < GLOBAL_STRATEGY_THRESHOLD and
            not recoveryUnderway) {
          tuneGeneration(myMap, false, true, false, position_x, position_y,
                         position_z, planning_depth_);
          for (int i = 0; i < 3; i++) {
            generateGoals(myMap, false);
            generateRRT(position_x, position_y, position_z);
            allowNewPath = true;
            setPath();
          }
        }
      }
    }

    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total =
        duration_cast<std::chrono::milliseconds>(stop_total - start_total);

    std_msgs::Float64MultiArray planning_time;

    planning_time.data = {duration_total.count(), ros::Time::now().toSec()};
    if (!duration_total.count() == 0) {

      execution_time_pub.publish(planning_time);
      cout << "\nExecution time: " << duration_total.count() << " ms for "
           << myGoals.size() << " path/s." << endl;
    }

    updatePathTaken();
    visualize(&points_pub, &output_path_pub, &all_path_pub, &goal_pub,
              &hits_pub, &taken_path_pub, &map_pub, &position_pub,
              &unknowns_pub);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// //

// ***** fix path length after NMPC calculation ** //

// ***** computation time plot ** //
