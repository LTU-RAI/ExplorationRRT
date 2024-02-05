#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <ros/ros.h>
#include <list>
#include <iostream>
#include <chrono>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <rrt/rrt_bindings.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <tf/tf.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include "std_msgs/Int64.h"
#include "std_msgs/Float64MultiArray.h"


using namespace std::chrono;
using namespace std;
using namespace std::this_thread;
double PLANNING_DEPTH = 0;
double INFO_GAIN_CHECK_DEPTH;
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// Structs

struct rrtSolverStatus rrt_solve(struct rrtCache *instance,
                                 double *u,
                                 const double *params,
                                 const double *y0,
                                 const double *c0);
                                 
struct rrtCache *rrt_new(void);

// The node struct.
// This struct represents the different kinds of points in the rrt, this includes:
// the nodes in the rrt tree, the goals and the paths.
struct node{
   public:
      ufo::math::Vector3* point;
      node* myParent = nullptr;
      double distanceToParent = -1;
      std::list<struct node*> myChilds{};
      std::list<struct node*> myPath{};
      std::list<ufo::math::Vector3> myHits{};
      double distanceToGoal;
      node(float x, float y, float z){
         point = new ufo::math::Vector3(x, y, z);
      }
      node(ufo::math::Vector3* givenPoint){
        point = new ufo::math::Vector3(givenPoint->x(), givenPoint->y(), givenPoint->z());
      }
      ufo::math::Vector3* getNodePointer(){
         return point;
      }
      void addChild(node* newChild){
        myChilds.push_back(newChild);
      }
      void addParent(node* newParent){
        myParent = newParent;
      }
      void changeDistanceToGoal(double newDistance){
        distanceToGoal = newDistance;
      }
      void changeDistanceToParent(){
        if(myParent != nullptr and distanceToParent != -1){
          distanceToParent = sqrt(pow(point->x() - myParent->point->x(), 2) + pow(point->y() - myParent->point->y(), 2) + pow(point->z() - myParent->point->z(), 2));
        }
        else{
          distanceToParent = 0;
        }
      }
      double sumDistance(){
        changeDistanceToParent();
        double myDistance = 0;
        if(myParent != nullptr){
          myDistance = myParent->sumDistance();
          changeDistanceToParent();
        }
        if(myParent == nullptr){
          return 0;
        }
        return (myDistance + distanceToParent);
      }
      
      void getPath(std::list<struct node*>* givenPath){
        if(myParent != nullptr){
          myParent->getPath(givenPath);
          if(myParent->myParent != nullptr){
            if(myParent->point->x() != point->x() or myParent->point->y() != point->y() or myParent->point->z() != point->z()){ 
              givenPath->push_back(new node(myParent->point->x(), myParent->point->y(), myParent->point->z()));
            }
          }
        }
        if(myParent == nullptr){
          return;
        }
        return;
      }


      int findInformationGain(float SCALER_AABB, float givenHorizontal, float givenVertical, float givenMin, float givenMax, ufo::map::OccupancyMapColor const& map, bool excludePath, bool findAnyInfo){
        if(myHits.empty() or findAnyInfo){
          // Setting up targets, X / Y targets in either positive (P) or negative (N) direction
          ufo::math::Vector3 targetXP(point->x() + 1, point->y(), point->z());
          ufo::math::Vector3 targetXN(point->x() - 1, point->y(), point->z());
          ufo::math::Vector3 targetYP(point->x(), point->y() + 1, point->z());
          ufo::math::Vector3 targetYN(point->x(), point->y() - 1, point->z());
          ufo::math::Vector3 upwards(point->x(), point->y(), point->z() + 1);
        
          //Angles
          double vertical_angle = givenVertical;
          double horizontal_angle = givenHorizontal;
        
          //Distances
          double near_distance = givenMin;
          double far_distance = givenMax;
        
          //Frustums
          ufo::geometry::Frustum frustXP(*point, targetXP, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
          ufo::geometry::Frustum frustXN(*point, targetXN, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
          ufo::geometry::Frustum frustYP(*point, targetYP, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
          ufo::geometry::Frustum frustYN(*point, targetYN, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
          int checks = 0;
          for (auto it = map.beginLeaves(frustXP, false, false, true, false, INFO_GAIN_CHECK_DEPTH), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, INFO_GAIN_CHECK_DEPTH)){
              if(findAnyInfo){
                return 1;
              }
              myHits.push_back(end_point);
            }
          }
          for (auto it = map.beginLeaves(frustXN, false, false, true, false, INFO_GAIN_CHECK_DEPTH), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, INFO_GAIN_CHECK_DEPTH)){
              if(findAnyInfo){
                return 1;
              }
              myHits.push_back(end_point);
            }
          }
          for (auto it = map.beginLeaves(frustYP, false, false, true, false, INFO_GAIN_CHECK_DEPTH), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, INFO_GAIN_CHECK_DEPTH)){
              if(findAnyInfo){
                return 1;
              }
              myHits.push_back(end_point);
            }
          }
          for (auto it = map.beginLeaves(frustYN, false, false, true, false, INFO_GAIN_CHECK_DEPTH), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, INFO_GAIN_CHECK_DEPTH)){
              if(findAnyInfo){
                return 1;
              }
              myHits.push_back(end_point);
            }
          }

          // info gain check if more than sum distance. 


          if(myParent != nullptr and not excludePath){
            myParent->findInformationGain(SCALER_AABB, givenVertical, givenHorizontal, givenMin, givenMax, map, excludePath, findAnyInfo);
          }
        }else{
          std::list<ufo::math::Vector3>::iterator it_hits;	
          for(it_hits = myHits.begin(); it_hits != myHits.end();){
            ufo::geometry::LineSegment myLine2(*point, *it_hits);
            if(isInCollision(map, myLine2, true, false, false, INFO_GAIN_CHECK_DEPTH)){
              it_hits = myHits.erase(it_hits);
            }else{
              if(isInCollision(map, *it_hits, false, true, false, INFO_GAIN_CHECK_DEPTH)){
                it_hits = myHits.erase(it_hits);
              }else{
                it_hits++;
              };
            };
          };
        }
        std::list<ufo::math::Vector3> myTotalHits{};
        addHits(&myTotalHits);
        int hits = myTotalHits.size();
        return hits;
      }
      
      void clearInformationGain(){
        myHits.clear();
        if(myParent != nullptr){
          myParent->clearInformationGain();
        }
      }
      
      void addHits(std::list<ufo::math::Vector3>* hitList){
        bool add = true;
        for(auto it = myHits.begin(), it_end = myHits.end(); it != it_end; ++it){
          for(auto it2 = hitList->begin(), it_end2 = hitList->end(); it2 != it_end2; ++it2){
            if(it->x() == it2->x() and it->y() == it2->y() and it->z() == it2->z()){
              add = false;
              break;
            }
          }
          if(add){
            hitList->push_back(*it);
          }
          add = true;
        }
        if(myParent != nullptr){
          myParent->addHits(hitList);
        }
      };
      
      // "world" -> odom_shafter
      // "Vänd" på funktionen, försök koppla "målet" till "nuvarande nod"
      bool findPathImprovement(struct node* targetNode, ufo::map::OccupancyMapColor const& map, float givenDistance, float givenRadious, auto pathImprovement_start, int givenMax){
        bool improvementFound;
        if(targetNode == this and myParent == nullptr){
          return true;
        }
        if(myParent != nullptr){
          improvementFound = myParent->findPathImprovement(targetNode, map, givenDistance, givenRadious, pathImprovement_start, givenMax);
          auto pathImprovement_stop = high_resolution_clock::now();
          auto pathImprovement_total = duration_cast<microseconds>(pathImprovement_stop - pathImprovement_start).count();
          if(pathImprovement_total > givenMax){
            return true;
          }
        }else{
          ufo::geometry::LineSegment myLine(*(targetNode->point), *point);
          if(!isInCollision(map, myLine, true, false, true, PLANNING_DEPTH)){
            ufo::math::Vector3 newVector(targetNode->point->x() - point->x(), targetNode->point->y() - point->y(), targetNode->point->z() - point->z());
            float distance = newVector.norm();
            // float distance = sqrt(pow(targetNode->point->x() - point->x(), 2) + pow(targetNode->point->y() - point->y(), 2) + pow(targetNode->point->z() - point->z(), 2));
            float itterations = (distance / givenRadious);
            float part = givenRadious / distance;
            float xStep = (targetNode->point->x() - point->x()) * part;
            float yStep = (targetNode->point->y() - point->y()) * part;
            float zStep = (targetNode->point->z() - point->z()) * part;
            for(int i = 1; i < itterations; i++){
              auto pathImprovement_stop = high_resolution_clock::now();
              auto pathImprovement_total = duration_cast<microseconds>(pathImprovement_stop - pathImprovement_start).count();
              if(pathImprovement_total > givenMax){
                return true;
              }
              ufo::math::Vector3 newVector = ufo::math::Vector3(point->x() + i * xStep, point->y() + i * yStep, point->z() + i * zStep);
              ufo::geometry::Sphere new_sphere(newVector, givenRadious);
              if(isInCollision(map, new_sphere, true, false, true, PLANNING_DEPTH)){
                return false;
              }
            }
            targetNode->addParent(this);
            return true;
          }else{
            return false;
          };
        }
        if(!improvementFound){
          ufo::geometry::LineSegment myLine(*(targetNode->point), *point);
          if(!isInCollision(map, myLine, true, false, true, PLANNING_DEPTH)){
            ufo::math::Vector3 newVector(targetNode->point->x() - point->x(), targetNode->point->y() - point->y(), targetNode->point->z() - point->z());
            float distance = newVector.norm();
            float itterations = (distance / givenRadious);
            float part = givenRadious / distance;
            float xStep = (targetNode->point->x() - point->x()) * part;
            float yStep = (targetNode->point->y() - point->y()) * part;
            float zStep = (targetNode->point->z() - point->z()) * part;
            for(int i = 1; i < itterations; i++){
              auto pathImprovement_stop = high_resolution_clock::now();
              auto pathImprovement_total = duration_cast<microseconds>(pathImprovement_stop - pathImprovement_start).count();
              if(pathImprovement_total > givenMax){
                return true;
              }
              ufo::math::Vector3 newVector = ufo::math::Vector3(point->x() + i * xStep, point->y() + i * yStep, point->z() + i * zStep);
              ufo::geometry::Sphere new_sphere(newVector, givenRadious);
              if(isInCollision(map, new_sphere, true, false, true, PLANNING_DEPTH)){
                return false;
              }
            }
            targetNode->addParent(this);
            improvementFound = findPathImprovement(this, map, givenDistance, givenRadious, pathImprovement_start, givenMax);
            return true;
          }else{
            return false;
          }
        }else{
          return true;
        }
      }
      
      void readyForDeletion(){
        delete point;
      }
      
      bool isInCollision(ufo::map::OccupancyMapColor const& map, 
                   ufo::geometry::BoundingVar const& bounding_volume, 
                   bool occupied_space = true, bool free_space = false,
                   bool unknown_space = false, ufo::map::DepthType min_depth = PLANNING_DEPTH){
        // Iterate through all leaf nodes that intersects the bounding volume
        for (auto it = map.beginLeaves(bounding_volume, occupied_space, 
                                 free_space, unknown_space, false, min_depth), 
        it_end = map.endLeaves(); it != it_end; ++it) {
          // Is in collision since a leaf node intersects the bounding volume.
          return true;
        }
        // No leaf node intersects the bounding volume.
        return false;
      }
};

// End of structs
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// Variables

// Variables
int NUMBER_OF_NODES;
int NUMBER_OF_GOALS;
int NUMBER_OF_ITTERATIONS;
int NMPC_POINTS;
int itterations;
int advance_index = 0;
int PATH_IMPROVEMENT_MAX;
float DISTANCE_BETWEEN_NODES; // 1.0;
float DISTANCE_BETWEEN_GOALS;
float MINIMUM_DISTANCE_TO_GOAL;
float SCALER_AABB;
float SCALER_X = SCALER_AABB;
float SCALER_Y = SCALER_AABB;
float SCALER_Z = SCALER_AABB;
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
bool RUN_BY_NODES;
bool INITIAL_POINT;
bool map_received = false;
bool RRT_created = false;
bool GOALS_generated = false;
bool position_received = false;
bool fetched_path = false;
bool newPath = false;
bool allowNewPath = true;
bool recoveryUnderway = false;
bool visualizeNewData = true;
double SENSOR_RANGE;
double SENSOR_MIN;
double SENSOR_VERTICAL;
double SENSOR_HORIZONTAL;
double SCALER_INFORMATION_GAIN;
double SCALER_DISTANCE;
double SCALER_ACTUATION;
double NEXT_POINT_DISTANCE;
double NEXT_PATH_DISTANCE;
double NMPC_DT;
double RADIOUS;
double RESOLUTION;
double MIN_INFO_GOAL;
double GOAL_SENSOR_RANGE;
double OBB_RADIUS;
double GOAL_CHECK_DIST;

double POSITION_TRACKING_WEIGHT_X;
double POSITION_TRACKING_WEIGHT_Y;
double POSITION_TRACKING_WEIGHT_Z;
double ANGLE_WEIGHT_ROLL;
double ANGLE_WEIGHT_PITCH;
double INPUT_WEIGHT_THRUST;
double INPUT_WEIGHT_ROLL;
double INPUT_WEIGHT_PITCH;
double INPUT_RATE_WEIGHT_THRUST;
double INPUT_RATE_WEIGHT_ROLL;
double INPUT_RATE_WEIGHT_PITCH;
double INITIAL_X;
double INITIAL_Y;
double INITIAL_Z;
double roll = 0;
double pitch = 0;
double yaw = 0;
double totalCost = std::numeric_limits<float>::max();
double totalDistance = -1;
string MAP_FRAME_ID;
node* goalNode = nullptr;
node* reserveGoalNode = nullptr;
node* currentTarget;

// Ufomap
ufo::map::OccupancyMapColor myMap(RESOLUTION);

// Lists
std::list<struct node*> RRT_TREE{};
std::list<struct node*> CHOSEN_PATH{};
std::list<double> CHOSEN_PATH_VREF{};
std::list<double>::iterator vref_itterator;
std::list<struct node*> ALL_PATH{};
std::list<struct node*> myGoals{};
std::list<struct node*> myReserveGoals{};
std::list<ufo::math::Vector3> hits{};
std::list<node*> VISITED_POINTS{};
std::list<node*>::iterator path_itterator;

// End of variables
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// Functions

ufo::geometry::OBB makeOBB(ufo::math::Vector3 source, ufo::math::Vector3 goal, float radius)
{
  ufo::math::Vector3 direction = goal - source;
  ufo::math::Vector3 center = source + (direction / 2.0);
  double distance = direction.norm();
  direction /= distance;
  double yaw = -atan2(direction[1], direction[0]);
  double pitch = -asin(direction[2]);
  double roll = 0; // TODO: Fix
  ufo::geometry::OBB obb(center, ufo::map::Point3(distance / 2.0, radius, radius),
                         ufo::math::Quaternion(roll, pitch, yaw));

  return obb;
}

// Fills in the space between nodes with new nodes.
// This allows for simpler path building.
void linSpace(node* givenNode, float givenDistance){
  ufo::math::Vector3 newVector(givenNode->myParent->point->x() - givenNode->point->x(), givenNode->myParent->point->y() - givenNode->point->y(), givenNode->myParent->point->z() - givenNode->point->z());
  float distance = newVector.norm();
  float itterations = (distance / givenDistance);
  float part = givenDistance / distance;
  float xStep = (givenNode->myParent->point->x() - givenNode->point->x()) * part;
  float yStep = (givenNode->myParent->point->y() - givenNode->point->y()) * part;
  float zStep = (givenNode->myParent->point->z() - givenNode->point->z()) * part;
  node* parent = givenNode->myParent;
  node* nextNode = givenNode->myParent;
  for(int i = 1; i < itterations; i++){
    node* newPoint = new node(givenNode->myParent->point->x() - i * xStep, givenNode->myParent->point->y() - i * yStep, givenNode->myParent->point->z() - i * zStep);
    newPoint->addParent(parent);
    parent = newPoint;
    RRT_TREE.push_back(newPoint);
  }
  givenNode->addParent(parent);
  if(nextNode->myParent != nullptr){
    linSpace(nextNode, givenDistance);
  }
}

// Evaluates the current point in the current path.
// This includes deciding when to change the current target to the next node in the path and when to calculate a new path.
void evaluateCurrentPoint(ros::Publisher* chosen_path_pub){
  if((sqrt(pow(position_x - goalNode->point->x(), 2) + pow(position_y - goalNode->point->y(), 2) + pow(position_z - goalNode->point->z(), 2)) < NEXT_PATH_DISTANCE)){
    itterations = 0;
    fetched_path = false;
    RRT_created = false;
    GOALS_generated = false;
    position_received = false;
    allowNewPath = true;
  }
  if((sqrt(pow(position_x - currentTarget->point->x(), 2) + pow(position_y - currentTarget->point->y(), 2) + pow(position_z - currentTarget->point->z(), 2)) < NEXT_POINT_DISTANCE) and path_itterator != --CHOSEN_PATH.end()){
    advance_index++;
    path_itterator = CHOSEN_PATH.begin();
    std::advance(path_itterator, advance_index);
    currentTarget = *path_itterator;
    std::advance(vref_itterator, 3);
    if(path_itterator == CHOSEN_PATH.end()){
      path_itterator--;
      currentTarget = *path_itterator;
    }
  }
  if(path_itterator != CHOSEN_PATH.end()){
    nav_msgs::Odometry nextPoint;
    nextPoint.pose.pose.position.x = (currentTarget)->point->x();
    nextPoint.pose.pose.position.y = (currentTarget)->point->y();
    nextPoint.pose.pose.position.z = (currentTarget)->point->z();
    if(!recoveryUnderway){
      nextPoint.twist.twist.linear.x = *vref_itterator;
      vref_itterator++;
      nextPoint.twist.twist.linear.y = *vref_itterator;
      vref_itterator++;
      nextPoint.twist.twist.linear.z = *vref_itterator;
      std::advance(vref_itterator, -2);
    }else{
      nextPoint.twist.twist.linear.x = 0;
      nextPoint.twist.twist.linear.y = 0;
      nextPoint.twist.twist.linear.z = 0;
    }
    nextPoint.pose.pose.orientation.x = 0;
    nextPoint.pose.pose.orientation.y = 0;
    nextPoint.pose.pose.orientation.z = 0;
    nextPoint.pose.pose.orientation.w = 0;
    nextPoint.header.stamp = ros::Time::now();
    nextPoint.header.frame_id = MAP_FRAME_ID;
    chosen_path_pub->publish(nextPoint);
  }
}

// Builds and publishes the visualization messages.
void visualize(ros::Publisher* points_pub, ros::Publisher* output_path_pub, ros::Publisher* chosen_path_visualization_pub, ros::Publisher* all_path_pub, ros::Publisher* goal_pub, ros::Publisher* hits_pub, ros::Publisher* taken_path_pub, ros::Publisher* map_pub, ros::Publisher* position_pub){
  visualization_msgs::Marker RRT_points, RRT_line_list, CHOSEN_PATH_points, CHOSEN_PATH_line_list, PATH_points, PATH_line_list, GOAL_points, HITS_points, TAKEN_PATH_points, TAKEN_PATH_line_list, POSITION_point;
  // Visualize each itteration
  if(position_received){
    POSITION_point.header.frame_id = MAP_FRAME_ID;
    POSITION_point.ns = "points";
    POSITION_point.action = visualization_msgs::Marker::ADD;
    POSITION_point.pose.orientation.w = 1.0;
    POSITION_point.id = 0;
    POSITION_point.type = visualization_msgs::Marker::POINTS;
    POSITION_point.scale.x = 0.2;
    POSITION_point.scale.y = 0.2;
    POSITION_point.color.g = 1.0f;
    POSITION_point.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = position_x;
    p.y = position_y;
    p.z = position_z;
    POSITION_point.points.push_back(p);
    position_pub->publish(POSITION_point);
  }
  // Visualize only once
  if(visualizeNewData){
    if(RRT_created){
      RRT_points.header.frame_id = RRT_line_list.header.frame_id = MAP_FRAME_ID;
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
      std::list<node*>::iterator it_comeon_visualizer;
      for(it_comeon_visualizer = RRT_TREE.begin(); it_comeon_visualizer != RRT_TREE.end(); it_comeon_visualizer++){
        geometry_msgs::Point p;
        p.x = (*it_comeon_visualizer)->point->x();
        p.y = (*it_comeon_visualizer)->point->y();
        p.z = (*it_comeon_visualizer)->point->z();
        RRT_points.points.push_back(p);
        if((*it_comeon_visualizer)->myParent != nullptr){
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
    if(!CHOSEN_PATH.empty()){
      nav_msgs::Path chosen_path;
      chosen_path.header.frame_id = MAP_FRAME_ID;
      chosen_path.poses.clear();
      for (auto i = CHOSEN_PATH.begin(); i != CHOSEN_PATH.end(); i++) {
        geometry_msgs::PoseStamped chosen_pose;
        chosen_pose.header.frame_id = MAP_FRAME_ID;
        chosen_pose.pose.position.x = (*i)->point->x();
        chosen_pose.pose.position.y = (*i)->point->y();
        chosen_pose.pose.position.z = (*i)->point->z();
        chosen_path.poses.push_back(chosen_pose);
      }
      output_path_pub->publish(chosen_path);
      CHOSEN_PATH_points.header.frame_id = CHOSEN_PATH_line_list.header.frame_id = MAP_FRAME_ID;
      CHOSEN_PATH_points.ns = "points";
      CHOSEN_PATH_points.action = visualization_msgs::Marker::ADD;
      CHOSEN_PATH_points.pose.orientation.w = 1.0;
      CHOSEN_PATH_points.id = 0;
      CHOSEN_PATH_line_list.id = 1;
      CHOSEN_PATH_points.type = visualization_msgs::Marker::SPHERE_LIST;
      CHOSEN_PATH_line_list.type = visualization_msgs::Marker::LINE_LIST;
      CHOSEN_PATH_points.scale.x = 0.2;
      CHOSEN_PATH_points.scale.y = 0.2;
      CHOSEN_PATH_points.scale.y = 0.2;
      CHOSEN_PATH_line_list.scale.x = 0.1;
      CHOSEN_PATH_points.color.g = 0.8;
      CHOSEN_PATH_points.color.a = 1.0;
      CHOSEN_PATH_line_list.color.b = 1.0;
      CHOSEN_PATH_line_list.color.a = 1.0;
      std::list<node*>::iterator it_comeon_visualizer2;
      for(it_comeon_visualizer2 = CHOSEN_PATH.begin(); it_comeon_visualizer2 != CHOSEN_PATH.end(); it_comeon_visualizer2++){
        geometry_msgs::Point p;
        p.x = (*it_comeon_visualizer2)->point->x();
        p.y = (*it_comeon_visualizer2)->point->y();
        p.z = (*it_comeon_visualizer2)->point->z();
        CHOSEN_PATH_points.points.push_back(p);
        if((*it_comeon_visualizer2)->myParent != nullptr){
          if(*it_comeon_visualizer2 != *CHOSEN_PATH.begin()){
            CHOSEN_PATH_line_list.points.push_back(p);
            p.x = (*it_comeon_visualizer2)->myParent->point->x();
            p.y = (*it_comeon_visualizer2)->myParent->point->y();
            p.z = (*it_comeon_visualizer2)->myParent->point->z();
            CHOSEN_PATH_line_list.points.push_back(p);
          }
        }
      }
      chosen_path_visualization_pub->publish(CHOSEN_PATH_points);
    }
    if(RRT_created){
      PATH_points.header.frame_id = PATH_line_list.header.frame_id = MAP_FRAME_ID;
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
      PATH_line_list.color.b = 1.0;
      PATH_line_list.color.a = 1.0;
      std::list<node*>::iterator it_comeon_visualizer5;
      ALL_PATH.clear();
      for(it_comeon_visualizer5 = myGoals.begin(); it_comeon_visualizer5 != myGoals.end(); it_comeon_visualizer5++){
        (*it_comeon_visualizer5)->getPath(&ALL_PATH);
        ALL_PATH.push_back((*it_comeon_visualizer5));
      }
      std::list<node*>::iterator it_comeon_visualizer6;	
      for(it_comeon_visualizer6 = ALL_PATH.begin(); it_comeon_visualizer6 != ALL_PATH.end(); it_comeon_visualizer6++){
        geometry_msgs::Point p;
        p.x = (*it_comeon_visualizer6)->point->x();
        p.y = (*it_comeon_visualizer6)->point->y();
        p.z = (*it_comeon_visualizer6)->point->z();
        PATH_points.points.push_back(p);
      }
      all_path_pub->publish(PATH_points);
      all_path_pub->publish(PATH_line_list);
    }
    if(GOALS_generated){
      GOAL_points.header.frame_id = MAP_FRAME_ID;
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
      std::list<node*>::iterator it_comeon_visualizer3;	
      for(it_comeon_visualizer3 = myGoals.begin(); it_comeon_visualizer3 != myGoals.end(); it_comeon_visualizer3++){
        geometry_msgs::Point p;
        p.x = (*it_comeon_visualizer3)->point->x();
        p.y = (*it_comeon_visualizer3)->point->y();
        p.z = (*it_comeon_visualizer3)->point->z();
        GOAL_points.points.push_back(p);
      }
      goal_pub->publish(GOAL_points);
    }
    if(goalNode != nullptr){
      hits.clear();
      goalNode->addHits(&hits);
      HITS_points.header.frame_id = MAP_FRAME_ID;
      HITS_points.ns = "points";
      HITS_points.action = visualization_msgs::Marker::ADD;
      HITS_points.pose.orientation.w = 1.0;
      HITS_points.id = 0;
      HITS_points.type = visualization_msgs::Marker::POINTS;
      HITS_points.scale.x = 0.2;
      HITS_points.scale.y = 0.2;
      HITS_points.color.r = 1.0f;
      HITS_points.color.a = 1.0;
      std::list<ufo::math::Vector3>::iterator it_comeon_visualizer4;	
      for(it_comeon_visualizer4 = hits.begin(); it_comeon_visualizer4 != hits.end(); it_comeon_visualizer4++){
        geometry_msgs::Point p;
        p.x = it_comeon_visualizer4->x();
        p.y = it_comeon_visualizer4->y();
        p.z = it_comeon_visualizer4->z();
        HITS_points.points.push_back(p);
      }
      hits_pub->publish(HITS_points);
    }
    if(goalNode != nullptr){
      TAKEN_PATH_points.header.frame_id = MAP_FRAME_ID;
      TAKEN_PATH_points.ns = "points";
      TAKEN_PATH_points.action = visualization_msgs::Marker::ADD;
      TAKEN_PATH_points.pose.orientation.w = 1.0;
      TAKEN_PATH_points.id = 0;
      TAKEN_PATH_points.type = visualization_msgs::Marker::POINTS;
      TAKEN_PATH_points.scale.x = 0.2;
      TAKEN_PATH_points.scale.y = 0.2;
      // TAKEN_PATH_points.scale.z = 0.2;
      TAKEN_PATH_points.color.b = 1;
      TAKEN_PATH_points.color.g = 1;
      TAKEN_PATH_points.color.r = 1;
      TAKEN_PATH_points.color.a = 0.8;
      std::list<node*>::iterator taken_path_visualizer;
      for(taken_path_visualizer = VISITED_POINTS.begin(); taken_path_visualizer != VISITED_POINTS.end(); taken_path_visualizer++){
        geometry_msgs::Point p;
        p.x = (*taken_path_visualizer)->point->x();
        p.y = (*taken_path_visualizer)->point->y();
        p.z = (*taken_path_visualizer)->point->z();
        TAKEN_PATH_points.points.push_back(p);
      }
      taken_path_pub->publish(TAKEN_PATH_points);
    }
    ufomap_msgs::UFOMapStamped::Ptr msg(new ufomap_msgs::UFOMapStamped);
    bool compress = false;
    ufo::map::DepthType pub_depth = 0;
    if(ufomap_msgs::ufoToMsg(myMap, msg->map, compress, pub_depth)) {
      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = MAP_FRAME_ID;
      map_pub->publish(msg);					        
    }else{
      std::cout << "Map conversion failed!" << std::endl;
    }
    visualizeNewData = false;
  }
}





// Evaluates and, if necessary, adds a new node to the path taken.
// The path taken is used during visualization and in the global strategy.
void updatePathTaken(){
  if(VISITED_POINTS.empty() and position_received){
    node* myNode = new node(position_x, position_y, position_z);
    VISITED_POINTS.push_back(myNode);
  }else{
    if(position_received){
      std::list<node*>::iterator taken_path_visualizer;
      taken_path_visualizer = --VISITED_POINTS.end();
      if(sqrt(pow((*taken_path_visualizer)->point->x() - position_x, 2) + pow((*taken_path_visualizer)->point->y() - position_y, 2) + pow((*taken_path_visualizer)->point->z() - position_z, 2)) >= 0.2){
        node* myNode = new node(position_x, position_y, position_z);
        (*taken_path_visualizer)->addParent(myNode);
        VISITED_POINTS.push_back(myNode);
      }
    }
  }
}

// Tunes the generation.
// This sets the bounding box which represents the local space.
// The bounding box will be reduced in size as much as possible without losing free space.
void tuneGeneration(ufo::map::OccupancyMapColor const& map, bool occupied_space, bool free_space, bool unknown_space, float given_x, float given_y, float given_z, ufo::map::DepthType min_depth = PLANNING_DEPTH){
  highest_x = -9999;
  highest_y = -9999;
  highest_z = -9999;
  lowest_x = std::numeric_limits<float>::max();
  lowest_y = std::numeric_limits<float>::max();
  lowest_z = std::numeric_limits<float>::max();
  ufo::math::Vector3 minPoint(given_x - 1 * SCALER_AABB, given_y - 1 * SCALER_AABB, given_z - 1 * SCALER_AABB);
  ufo::math::Vector3 maxPoint(given_x + 1 * SCALER_AABB, given_y + 1 * SCALER_AABB, given_z + 1 * SCALER_AABB);
  ufo::geometry::AABB aabb(minPoint, maxPoint);
  for (auto it = map.beginLeaves(aabb, occupied_space, free_space, unknown_space, false, min_depth), it_end = map.endLeaves(); it != it_end; ++it) {
    if(it.getX() > highest_x){
      highest_x = it.getX();
    }if(it.getX() < lowest_x){
      lowest_x = it.getX();
    }
    if(it.getY() > highest_y){
      highest_y = it.getY();
    }if(it.getY() < lowest_y){
      lowest_y = it.getY();
    }
    if(it.getZ() > highest_z){
      highest_z = it.getZ();
    }if(it.getZ() < lowest_z){
      lowest_z = it.getZ();
    }
  }
  SCALER_X = highest_x - lowest_x;
  SCALER_Y = highest_y - lowest_y;
  SCALER_Z = highest_z - lowest_z;
}

bool isInCollision(ufo::map::OccupancyMapColor const& map, 
                   ufo::geometry::BoundingVar const& bounding_volume, 
                   bool occupied_space = false, bool free_space = false,
                   bool unknown_space = false, ufo::map::DepthType min_depth = PLANNING_DEPTH)
{
  // Iterate through all leaf nodes that intersects the bounding volume
  for (auto it = map.beginLeaves(bounding_volume, occupied_space, 
                                 free_space, unknown_space, false, min_depth), 
        it_end = map.endLeaves(); it != it_end; ++it) {
    // Is in collision since a leaf node intersects the bounding volume.
    return true;
  }
  // No leaf node intersects the bounding volume.
  return false;
}

// The global path planning strategy.
// If the local path planning fails, the global planner tries to retrace to the most recently not picked goal.
void globalStrategy(){
  for(std::list<node*>::iterator retrace_path_itterator = --myReserveGoals.end(); retrace_path_itterator != myReserveGoals.begin(); retrace_path_itterator--){
    if(!CHOSEN_PATH.empty()){
      for(std::list<node*>::iterator it_clear_helper = CHOSEN_PATH.begin(); it_clear_helper != --CHOSEN_PATH.end(); it_clear_helper++){
        (*it_clear_helper)->readyForDeletion();
        delete(*it_clear_helper);
      }
      CHOSEN_PATH.clear();
      CHOSEN_PATH_VREF.clear();
    }
    (*retrace_path_itterator)->clearInformationGain();
    double informationGain = (*retrace_path_itterator)->findInformationGain(SCALER_AABB, SENSOR_HORIZONTAL, SENSOR_VERTICAL, SENSOR_MIN, SENSOR_RANGE, myMap, true, false);
    if(informationGain > GLOBAL_PATH_THRESHOLD){
      auto pathImprovement_start = high_resolution_clock::now();
      (*retrace_path_itterator)->findPathImprovement(*retrace_path_itterator, myMap, DISTANCE_BETWEEN_NODES, RADIOUS, pathImprovement_start, PATH_IMPROVEMENT_MAX);
      std::list<struct node*> PATH_CONTAINER{};
      (*retrace_path_itterator)->getPath(&PATH_CONTAINER);
      PATH_CONTAINER.push_back(new node((*retrace_path_itterator)->point->x(), (*retrace_path_itterator)->point->y(), (*retrace_path_itterator)->point->z()));
      CHOSEN_PATH.push_back(*(PATH_CONTAINER.begin()));
      node* currentPoint = *(PATH_CONTAINER.begin());
      node* savedPoint = nullptr;
      int myIndex = 0;
      int mySavedIndex = 0;
      while(currentPoint != *(--PATH_CONTAINER.end())){
        std::list<node*>::iterator it_path_helper1 = PATH_CONTAINER.begin();
        std::advance(it_path_helper1, mySavedIndex);
        while(it_path_helper1 != PATH_CONTAINER.end()){
          ufo::geometry::LineSegment myLine(*(currentPoint->point), (*(*it_path_helper1)->point));
          if(!isInCollision(myMap, myLine, true, false, true, PLANNING_DEPTH)){
            savedPoint = *(it_path_helper1);
            mySavedIndex = myIndex;
          }
          myIndex++;
          it_path_helper1++;
        }
        currentPoint = savedPoint;
        CHOSEN_PATH.push_back(new node(savedPoint->point->x(), savedPoint->point->y(), savedPoint->point->z()));
      }
      CHOSEN_PATH.push_back(new node((*retrace_path_itterator)->point->x(), (*retrace_path_itterator)->point->y(), (*retrace_path_itterator)->point->z()));
      goalNode = *retrace_path_itterator;
      path_itterator = CHOSEN_PATH.begin();
      currentTarget = *path_itterator;
      advance_index = 0;
      allowNewPath = false;
      recoveryUnderway = true;
      std::list<node*>::iterator erase_it = myReserveGoals.end();
      retrace_path_itterator++;
      myReserveGoals.erase(retrace_path_itterator, erase_it);
      break;
    }
  cout << " \n----- GLOBAL ----- " << endl;
  
  }

  
}

// Evaluates the RRT tree for the purpose of reducing the distance in a path.
// The node with the shortest path to the root node, which the goal in question can see, will be the parent of the currently evaluated goal node.
// The path between the goal node and its' parent has to pass a series of sphere checks, which aims to guarantee a distance of RADIOUS between the path and the occupied and unknown space.
void findShortestPath(){

    high_resolution_clock::time_point start_total = high_resolution_clock::now();

  for(std::list<node*>::iterator it_goals = myGoals.begin(); it_goals != myGoals.end(); it_goals++){
    struct node* chosenNode = nullptr;
    double distance = std::numeric_limits<double>::max();
    for(std::list<node*>::iterator it_RRT = RRT_TREE.begin(); it_RRT != RRT_TREE.end(); it_RRT++){
      double distanceNodeToGoal = sqrt(pow((*it_RRT)->point->x() - (*it_goals)->point->x(), 2) + pow((*it_RRT)->point->y() - (*it_goals)->point->y(), 2) + pow((*it_RRT)->point->z() - (*it_goals)->point->z(), 2));
      double distanceToNode = (*it_RRT)->sumDistance();
      if (distanceNodeToGoal < GOAL_CHECK_DIST) {

      double totalDistance = distanceNodeToGoal + distanceToNode;
      if(totalDistance < distance){
        
        ufo::geometry::OBB obb = makeOBB(*((*it_RRT)->point), *((*it_goals)->point), OBB_RADIUS);
        
        // ufo::geometry::LineSegment myLine(*((*it_goals)->point), *((*it_RRT)->point));

        // if(!isInCollision(myMap, myLine, true, false, true, PLANNING_DEPTH)){

          if(!isInCollision(myMap, obb, true, false, true, PLANNING_DEPTH)){
            bool add = true;
          // ufo::math::Vector3 newVector((*it_goals)->point->x() - (*it_RRT)->point->x(), (*it_goals)->point->y() - (*it_RRT)->point->y(), (*it_goals)->point->z() - (*it_RRT)->point->z());
          // float distance_calc = newVector.norm();
          // float itterations = (distance_calc / DISTANCE_BETWEEN_NODES);
          // float part = DISTANCE_BETWEEN_NODES / distance_calc;
          // float xStep = ((*it_goals)->point->x() - (*it_RRT)->point->x()) * part;
          // float yStep = ((*it_goals)->point->y() - (*it_RRT)->point->y()) * part;
          // float zStep = ((*it_goals)->point->z() - (*it_RRT)->point->z()) * part;
          // bool add = true;
          // for(int i = 1; i < itterations; i++){
          //   ufo::math::Vector3 newVector = ufo::math::Vector3((*it_RRT)->point->x() + i * xStep, (*it_RRT)->point->y() + i * yStep, (*it_RRT)->point->z() + i * zStep);
          //   ufo::geometry::Sphere new_sphere(newVector, RADIOUS);
          //   if(isInCollision(myMap, new_sphere, true, false, true, PLANNING_DEPTH)){
          //     add = false;
          //     break;
          //   }
          // }

            if(isInCollision(myMap, obb, true, false, true, PLANNING_DEPTH)){
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
    if(chosenNode != nullptr){
      (*it_goals)->addParent(chosenNode);
      chosenNode->addChild(*it_goals);
    }
  }


      high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    cout << "\n FIND SHORTEST Execution time: " << duration_total.count() << " micro seconds " << endl;
}

// Generates goals.
// The goals generated guarantees at least one piece of new information within SENSOR_RANGE,
// as well as being in free space and at least RADIOUS distance away from both unknown and occupied space.
void generateGoals(ufo::map::OccupancyMapColor const& map, bool evaluateOldGoals){
  if(!myGoals.empty() and evaluateOldGoals){
    double newCost = std::numeric_limits<float>::max();
    double totalCost = std::numeric_limits<float>::max();
    for(std::list<node*>::iterator it_goal = myGoals.begin(); it_goal != myGoals.end(); it_goal++){
      if((*it_goal)->myParent != nullptr){
        (*it_goal)->clearInformationGain();
        double informationGain = SCALER_INFORMATION_GAIN * ((*it_goal)->findInformationGain(SCALER_AABB, SENSOR_HORIZONTAL, SENSOR_VERTICAL, SENSOR_MIN, SENSOR_RANGE, myMap, true, false));
        newCost = -informationGain;
        if(informationGain > initialGoalInfo){
          initialGoalInfo = informationGain;
        }
        int stickyCounter = 0;
        for(std::list<ufo::math::Vector3>::iterator it_floor = (*it_goal)->myHits.begin(); it_floor != (*it_goal)->myHits.end(); it_floor++){
          if(it_floor->z() < (*it_goal)->point->z()){
            stickyCounter++;
          }
        }
        bool infoRequirement = ((*it_goal)->myHits.size() > GLOBAL_PATH_THRESHOLD);
        bool stickyFloor = ((stickyCounter < 0.8 * (*it_goal)->myHits.size()));
        if((newCost < totalCost) and ((*it_goal)->findInformationGain(SCALER_AABB, SENSOR_HORIZONTAL, SENSOR_VERTICAL, SENSOR_MIN, SENSOR_RANGE, myMap, false, false) > 0) and (stickyFloor and infoRequirement) and (*it_goal)->myParent != nullptr){
          totalCost = newCost;
          reserveGoalNode = *it_goal;
        }
      }
    }
    if(reserveGoalNode != nullptr){
      myReserveGoals.push_back(reserveGoalNode);
      for(std::list<node*>::iterator it_parent_finder = --VISITED_POINTS.end(); it_parent_finder != VISITED_POINTS.begin(); it_parent_finder--){
        ufo::geometry::LineSegment myLine((*(*it_parent_finder)->point), (*reserveGoalNode->point));
        if(!isInCollision(map, myLine, true, false, true, PLANNING_DEPTH)){
          reserveGoalNode->addParent((*it_parent_finder));
          break;
        }
      }
    }
  }
  if(goalNode != nullptr){
    if(sqrt(pow(position_x - goalNode->point->x(), 2) + pow(position_y - goalNode->point->y(), 2) + pow(position_z - goalNode->point->z(), 2)) > NEXT_PATH_DISTANCE){
      std::list<node*>::iterator it_goal2;
      int help_counter = 0;
      for(it_goal2 = myGoals.begin(); it_goal2 != myGoals.end(); it_goal2++){
        help_counter++;
        if(*it_goal2 != goalNode and *it_goal2 != reserveGoalNode){
          (*it_goal2)->readyForDeletion();
          delete(*it_goal2);
        }
      }
      myGoals.clear();
      myGoals.push_back(goalNode);
    }else{
      std::list<node*>::iterator it_goal2;
      for(it_goal2 = myGoals.begin(); it_goal2 != myGoals.end(); it_goal2++){
        if(*it_goal2 != reserveGoalNode){
          (*it_goal2)->readyForDeletion();
          delete(*it_goal2);
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
  while((myGoals.size() < NUMBER_OF_GOALS) and (itterations < 10000)){
    float x = lowest_x + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_X;
    float y = lowest_y + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_Y;
    float z = lowest_z + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_Z;
    node goal = node(x, y, z);
    ufo::geometry::Sphere goal_sphere(*(goal.point), RADIOUS);
    if(sqrt(pow(position_x - x, 2) + pow(position_y - y, 2) + pow(position_z - z, 2)) > MINIMUM_DISTANCE_TO_GOAL){
      if(!isInCollision(myMap, goal_sphere, true, false, true, PLANNING_DEPTH) and isInCollision(myMap, goal_sphere, false, true, false, PLANNING_DEPTH)){
        bool add = true;
        for(std::list<node*>::iterator it_goal = myGoals.begin(); it_goal != myGoals.end(); it_goal++){
          if(sqrt(pow((*it_goal)->point->x() - x, 2) + pow((*it_goal)->point->y() - y, 2) + pow((*it_goal)->point->z() - z, 2)) < DISTANCE_BETWEEN_GOALS){
            add = false;
            break;
          }
        }
        if(add){
          
          int foundInfo;

          if (MIN_INFO_GOAL == 1) {
          foundInfo = goal.findInformationGain(SCALER_AABB, SENSOR_HORIZONTAL, SENSOR_VERTICAL, SENSOR_MIN, GOAL_SENSOR_RANGE, myMap, false, true);
          } else {
          foundInfo = goal.findInformationGain(SCALER_AABB, SENSOR_HORIZONTAL, SENSOR_VERTICAL, SENSOR_MIN, GOAL_SENSOR_RANGE, myMap, false, false);
          }

          // if(foundInfo == MIN_INFO_GOAL){
            if(foundInfo >= MIN_INFO_GOAL){
            node* newGoal = new node(x, y, z);
            myGoals.push_back(newGoal);
          }
        }
      };
      itterations++;
    }
  };
  if(myGoals.size() == NUMBER_OF_GOALS){
    std::cout << "Goals generated successfully\n" << std::endl;
    GOALS_generated = true;
  }else if(myGoals.size() == 0){
    std::cout << "No goals found, trying again soon" << std::endl;
    sleep_for(microseconds(100000));
  }else{
    std::cout << "Only " << myGoals.size() << " goals found" << std::endl;
    GOALS_generated = true;
  }
};

// Evaluates and sets the current path
// The evaluation takes the distance cost, information gain and the distance cost in mind, choosing the path with the overall smallest cost.
// The evaluation depends heavily on the values of SCALER_DISTANCE, SCALER_INFORMATION_GAIN and SCALER_ACTUATION
void setPath(){
  bool setDistance = false;
  if(goalNode != nullptr){
    goalNode->clearInformationGain();


    if((sqrt(pow(position_x - goalNode->point->x(), 2) + pow(position_y - goalNode->point->y(), 2) + pow(position_z - goalNode->point->z(), 2)) < NEXT_PATH_DISTANCE)){
      allowNewPath = true;
      totalCost = std::numeric_limits<float>::max();
    }else{
      totalCost = goalNode->sumDistance() * SCALER_DISTANCE - SCALER_INFORMATION_GAIN * (goalNode->findInformationGain(SCALER_AABB, SENSOR_HORIZONTAL, SENSOR_VERTICAL, SENSOR_MIN, SENSOR_RANGE, myMap, true, false));
    }
  }else{
    totalCost = std::numeric_limits<float>::max();
  }
  double newCost = std::numeric_limits<float>::max();
  if(allowNewPath){
    std::list<double> PATH_CONTAINER{};
    initialGoalInfo = 0;
    for(std::list<node*>::iterator it_goal = myGoals.begin(); it_goal != myGoals.end(); it_goal++){

      if((*it_goal)->myParent != nullptr ) {

        // TODO @aakapatel 
        // Find informatationgain only for the goals not directly in line of sight of the drone 
        //
        // ufo::math::Vector3 current_point (position_x, position_y, position_z);
        // ufo::math::Vector3 goal_point ((*it_goal)->point->x(), (*it_goal)->point->y(), (*it_goal)->point->z());
        // // ufo::geometry::OBB obb = makeOBB(start_point, random_point, OBB_RADIUS);
        // ufo::geometry::LineSegment myLine(current_point, goal_point);
        // if(!isInCollision(myMap, myLine, true, false, true, PLANNING_DEPTH)) {
        //   
        //   ROS_WARN_STREAM ("check 0");
        //   it_goal++;
        // }

        //linSpace(*it_goal, SENSOR_RANGE / 2);
        auto pathImprovement_start = high_resolution_clock::now();

        (*it_goal)->findPathImprovement(*it_goal, myMap, DISTANCE_BETWEEN_NODES, RADIOUS, pathImprovement_start, PATH_IMPROVEMENT_MAX);
        double informationGain = SCALER_INFORMATION_GAIN * ((*it_goal)->findInformationGain(SCALER_AABB, SENSOR_HORIZONTAL, SENSOR_VERTICAL, SENSOR_MIN, SENSOR_RANGE, myMap, false, false));

        linSpace(*it_goal, DISTANCE_BETWEEN_NODES);
        auto pathImprovement_start_2 = high_resolution_clock::now();
        (*it_goal)->findPathImprovement(*it_goal, myMap, DISTANCE_BETWEEN_NODES, RADIOUS, pathImprovement_start_2, PATH_IMPROVEMENT_MAX);
        double distanceCost = (*it_goal)->sumDistance() * SCALER_DISTANCE;
        // double informationGain = SCALER_INFORMATION_GAIN * log((*it_goal)->findInformationGain(SCALER_AABB, SENSOR_HORIZONTAL, SENSOR_VERTICAL, SENSOR_MIN, SENSOR_RANGE, myMap, true, false));
        // double informationGain = SCALER_INFORMATION_GAIN * ((*it_goal)->findInformationGain(SCALER_AABB, SENSOR_HORIZONTAL, SENSOR_VERTICAL, SENSOR_MIN, SENSOR_RANGE, myMap, false, false));
        linSpace(*it_goal, DISTANCE_BETWEEN_NODES);
	
        typedef rrtCache* (*arbitrary)();
        typedef rrtSolverStatus (*arbitrary2)(void*, double*, double*, double, double*);
        typedef void (*rrt_clearer)(rrtCache*);
        int i = 0;
        double p[159] = {0};
        std::list<double> xref = {};
        std::list<double> x0 = {position_x, position_y, position_z, velocity_x, velocity_y, velocity_z, roll, pitch};
        
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
        std::list<struct node*> EVALUATE_PATH{};
        EVALUATE_PATH.clear();
        (*it_goal)->getPath(&EVALUATE_PATH);
        EVALUATE_PATH.push_back(new node((*it_goal)->point->x(), (*it_goal)->point->y(), (*it_goal)->point->z()));
        xref.push_back((*it_goal)->point->x());
        xref.push_back((*it_goal)->point->y());
        xref.push_back((*it_goal)->point->z()); 
        std::list<node*>::iterator it_evaluator = EVALUATE_PATH.begin();
        for (i = 1; i < NMPC_POINTS; ++i){
          p[8+3*i] = (*it_evaluator)->point->x();
          xref.push_back((*it_evaluator)->point->x());
          p[8+3*i+1] = (*it_evaluator)->point->y();
          xref.push_back((*it_evaluator)->point->y());
          p[8+3*i+2] = (*it_evaluator)->point->z();
          xref.push_back((*it_evaluator)->point->z());
          if(it_evaluator != --EVALUATE_PATH.end()){
            it_evaluator++;
          }
        }
        p[158] = NMPC_DT;
        
        double u[150] = {0};

        for (i = 0; i < NMPC_POINTS; ++i) {
          u[3*i] = 0;
          u[3*i + 1] = 0;
          u[3*i + 2] = 0;
        }
        
        double init_penalty = 0;
        void *handle = dlopen((ros::package::getPath("errt")  + "/MAV/rrt/target/release/librrt.so").c_str(), RTLD_LAZY);
        if (!handle) {
          fprintf(stderr, "%s\n", dlerror());
          exit(EXIT_FAILURE);
        }
        arbitrary rrt_new;
        *(void **) (&rrt_new) = dlsym(handle, "rrt_new");
        rrtCache* cache = rrt_new();
        arbitrary2 rrt_solve;
        *(void **) (&rrt_solve) = dlsym(handle, "rrt_solve");
        rrt_clearer rrt_free;
        *(void **) (&rrt_free) = dlsym(handle, "rrt_free");

        ROS_INFO_STREAM ( "Calculating .. " << init_penalty);
        
        rrtSolverStatus status = rrt_solve(cache, u, p, 0, &init_penalty);
  
        std::list<double> uold = {9.81,0.0,0.0};
        std::list<double> uref = {9.81,0.0,0.0};
        
        std::list<double> x_hist;
        std::list<double> p_hist;
        double cost;
        std::tuple<std::list<double>, double, std::list<double>> trajectory(std::list<double> x, double* u, double N, double dt, std::list<double> nmpc_ref);
        std::tie(x_hist, cost, p_hist) = trajectory(x0, u, NMPC_POINTS, NMPC_DT, xref);
        xref.clear();
        rrt_free(cache);
        double actuationCost = SCALER_ACTUATION * cost;
        newCost = distanceCost - informationGain + actuationCost;
        ////////////////////////////////////////////////////////////////
        std::list<double> new_p_hist;
        // new_p_hist.clear();
        // if (EVALUATE_PATH.size() < NMPC_POINTS) {
          for (auto i = p_hist.begin(); i != p_hist.end(); i++) {
            if (std::distance(p_hist.begin(), i) / 3  <= EVALUATE_PATH.size() + 1) {
              new_p_hist.push_back(*i);
            }
          }
        // }

        if(informationGain > initialGoalInfo){
          initialGoalInfo = informationGain;
        }
        int stickyCounter = 0;
        for(std::list<ufo::math::Vector3>::iterator it_floor = (*it_goal)->myHits.begin(); it_floor != (*it_goal)->myHits.end(); it_floor++){
          if(it_floor->z() < (*it_goal)->point->z()){
            stickyCounter++;
          }
        }
        bool infoRequirement = ((*it_goal)->myHits.size() > GLOBAL_STRATEGY_THRESHOLD);
        bool stickyFloor = ((stickyCounter < 0.8 * (*it_goal)->myHits.size()));
        if((newCost < totalCost) and ((*it_goal)->findInformationGain(SCALER_AABB, SENSOR_HORIZONTAL, SENSOR_VERTICAL, SENSOR_MIN, SENSOR_RANGE, myMap, false, false) > 0) and allowNewPath and (stickyFloor and infoRequirement) and (*it_goal)->myParent != nullptr){
          totalCost = newCost;
          goalNode = *it_goal;
          newPath = true;
          PATH_CONTAINER.clear();
          // PATH_CONTAINER.push_back(position_x);
          // PATH_CONTAINER.push_back(position_y);
          // PATH_CONTAINER.push_back(position_z);
          for(std::list<double>::iterator path_itterator_helper = new_p_hist.begin(); path_itterator_helper != new_p_hist.end();){
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
          if(EVALUATE_PATH.size() > NMPC_POINTS){
            std::list<node*>::iterator path_itterator_helper2 = EVALUATE_PATH.begin();
            std::advance(path_itterator_helper2, NMPC_POINTS);
            while(path_itterator_helper2 != EVALUATE_PATH.end()){
              PATH_CONTAINER.push_back((*path_itterator_helper2)->point->x());
              PATH_CONTAINER.push_back((*path_itterator_helper2)->point->y());
              PATH_CONTAINER.push_back((*path_itterator_helper2)->point->z());
              path_itterator_helper2++;
            }
          }
          CHOSEN_PATH_VREF.clear();
          for(std::list<double>::iterator reference_itterator_helper = x_hist.begin(); reference_itterator_helper != x_hist.end(); reference_itterator_helper++){
            CHOSEN_PATH_VREF.push_back(*reference_itterator_helper);
          }
          vref_itterator = CHOSEN_PATH_VREF.begin();
        }
      


      }
    }
    if(not recoveryUnderway){
      std::list<node*>::iterator it_clear_helper;
      for(it_clear_helper = CHOSEN_PATH.begin(); it_clear_helper != --CHOSEN_PATH.end(); it_clear_helper++){
        (*it_clear_helper)->readyForDeletion();
        delete(*it_clear_helper);
      }
    }else{
      std::list<node*>::iterator it_clear_helper = --CHOSEN_PATH.end();
      (*it_clear_helper)->readyForDeletion();
      delete(*it_clear_helper);
      recoveryUnderway = false;
    }
    CHOSEN_PATH.clear();
    if(goalNode != nullptr and newPath){
      setDistance = true;
      std::list<double>::iterator it_path_helper = PATH_CONTAINER.begin();
      for(int i = 0; i < (PATH_CONTAINER.size() / 3); i++){
        double x = *it_path_helper;
        it_path_helper++;
        double y = *it_path_helper;
        it_path_helper++;
        double z = *it_path_helper;
        it_path_helper++;
        CHOSEN_PATH.push_back(new node(x, y, z));
      }
      PATH_CONTAINER.clear();
      CHOSEN_PATH.push_back(new node(goalNode->point->x(), goalNode->point->y(), goalNode->point->z()));
      fetched_path = true;
      visualizeNewData = true;
      allowNewPath = false;
      newPath = false;
      path_itterator = CHOSEN_PATH.begin();
      currentTarget = *path_itterator;
      advance_index = 0;
     }
  }
  if(setDistance){
    totalDistance = goalNode->sumDistance();
  };
}

// Generates the RRT
// Generates the nodes in the RRT-tree and connects them to a already existing parent.
// The root of the node exists at the current position of the plant.
void generateRRT(float given_x, float given_y, float given_z){
  // high_resolution_clock::time_point start_total = high_resolution_clock::now();


  for(std::list<node*>::iterator it_clear_helper = RRT_TREE.begin(); it_clear_helper != --RRT_TREE.end(); it_clear_helper++){
    (*it_clear_helper)->readyForDeletion();
    delete(*it_clear_helper);
  }
  for(std::list<node*>::iterator it_clear_helper = myGoals.begin(); it_clear_helper != --myGoals.end(); it_clear_helper++){
    (*it_clear_helper)->addParent(nullptr);
  }
  RRT_TREE.clear();
  std::cout << "Building RRT-tree" << std::endl;
  node* origin = new node(given_x, given_y, given_z);
  origin->addParent(nullptr);
  RRT_TREE.push_back(origin);
  srand(time(0));
  itterations = 0;
  while(((RRT_TREE.size() <= NUMBER_OF_NODES and RUN_BY_NODES) or (itterations <= NUMBER_OF_ITTERATIONS and !RUN_BY_NODES)) and itterations < 100000){
    // Generate a random point
    float x = lowest_x + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_X;
    float y = lowest_y + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_Y;
    float z = lowest_z + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_Z;
    ufo::math::Vector3 random_point(x, y, z);
    ufo::geometry::Sphere point_sphere(random_point, RADIOUS);
    if(!isInCollision(myMap, point_sphere, true, false, true, PLANNING_DEPTH) and isInCollision(myMap, point_sphere, false, true, false, PLANNING_DEPTH)){
      float distance = std::numeric_limits<float>::max();
      node* parent;
      std::list<node*>::iterator it_node;
      for(it_node = RRT_TREE.begin(); it_node != RRT_TREE.end(); it_node++){
        ufo::math::Vector3 direction = random_point - *((*it_node)->point);
        double new_distance = abs(direction.norm());
        if(new_distance < distance){
          distance = new_distance;
          parent = *it_node;
        }
      };
      ufo::math::Vector3 start_point(parent->point->x(), parent->point->y(), parent->point->z());
      ufo::geometry::OBB obb = makeOBB(start_point, random_point, OBB_RADIUS);
      // ufo::geometry::LineSegment myLine(random_point, start_point);
      // if(!isInCollision(myMap, myLine, true, false, true, PLANNING_DEPTH)) {
        if(!isInCollision(myMap, obb, true, false, true, PLANNING_DEPTH)) {
          node* new_node = new node(x, y, z);
          new_node->addParent(parent);
          parent->addChild(new_node);
          RRT_TREE.push_back(new_node);
      }
    };
    itterations++;
  };
  std::cout << "RRT-tree built successfully" << std::endl;
  
  if(RUN_BY_NODES){
    std::cout << "Verifying tree" << std::endl;
    int total_childs = 0;
    int total_parents = 0;
    std::list<node*>::iterator it_comeon;
    for(it_comeon = RRT_TREE.begin(); it_comeon != RRT_TREE.end(); it_comeon++){
      total_childs = total_childs + (*it_comeon)->myChilds.size();
      if((*it_comeon)->myParent != nullptr){
        total_parents++;
      }
    };
    if(total_childs != 0){
      RRT_created = true;
    }
    if(total_childs == NUMBER_OF_NODES){
      std::cout << "All children accounted for" << std::endl;
    }else{
      std::cout << "Expected " << NUMBER_OF_NODES << " children, but " << total_childs << " was found." << std::endl;
    };
    if(total_parents == NUMBER_OF_NODES){
      std::cout << "All parents accounted for" << std::endl;
    }else{
      std::cout << "Expected " << NUMBER_OF_NODES << " parents, but " << total_parents << " was found." << std::endl;
    };
  }else{
    std::cout << "Running by itterations, so the amount of nodes are unknown and hence can't be verified" << std::endl;
  }

    // high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    // auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    // cout << "\nExecution time: " << duration_total.count() << " micro seconds" << endl;
    
};

// Trajectory
// Developed by Björn Lindqvist
std::tuple<std::list<double>, double, std::list<double>> trajectory(std::list<double> x, double* u, double N, double dt, std::list<double> nmpc_ref){
    // Calculate the dynamic costs based on selected weights  
    double ns = 8;
    std::list<double> p_traj{};
    std::list<double> v_traj{};
    double cost = 0;
    // Weight matrices
    std::list<double> Qx = {POSITION_TRACKING_WEIGHT_X,POSITION_TRACKING_WEIGHT_Y,POSITION_TRACKING_WEIGHT_Z, 0, 0,0, ANGLE_WEIGHT_ROLL, ANGLE_WEIGHT_PITCH}; // Position tracking weights x, y, z, 0, 0, 0, angle weights roll / pitch
    // P = 2*Qx; #final state weight
    std::list<double> Ru = {INPUT_WEIGHT_THRUST, INPUT_WEIGHT_ROLL, INPUT_WEIGHT_PITCH}; // input weights (Thrust, roll, pitch)
    std::list<double> Rd = {INPUT_RATE_WEIGHT_THRUST, INPUT_RATE_WEIGHT_ROLL, INPUT_RATE_WEIGHT_PITCH}; // input rate weights (thrust, roll, pitch)
    // print(x, u, N, dt)
    std::list<double> u_old = {9.81, 0, 0};
    std::list<double> u_ref = {9.81,0.0,0.0};
    std::list<double>::iterator x_ref_itterator = nmpc_ref.begin();
    for(int i = 0; i < N; i++){
      std::list<double>::iterator Qx_itterator = Qx.begin();
      std::list<double>::iterator x_itterator = x.begin();
      
      //Setting up itterators
      std::list<double>::iterator Ru_itterator = Ru.begin();
      std::list<double>::iterator Rd_itterator = Rd.begin();
      std::list<double>::iterator u_old_itterator = u_old.begin();
      
      for(int j = 0; j < 3; j++){
        cost = cost + *Ru_itterator * pow(u[3*i+j] - *u_old_itterator, 2);
        cost = cost + *Rd_itterator * pow(u[3*i+j] - *u_old_itterator, 2);
        Ru_itterator++;
        u_old_itterator++;
        Rd_itterator++;
      }
      u_old.clear();
      u_old = {u[3*i], u[3*i+1], u[3*i+2]};
      
      x_itterator = x.begin();
      std::list<double>::iterator x2_itterator = x.begin();
      std::advance(x2_itterator, 3);
      for(int j = 0; j < 3; j++){
        *x_itterator = *x_itterator + dt * *x2_itterator;
        x_itterator++;
        x2_itterator++;
      }
      std::list<double>::iterator x3_itterator = x.begin();
      std::advance(x3_itterator, 7); // x[7]
      *x_itterator = *x_itterator + dt * (sin(*x3_itterator) * cos(*x2_itterator) * u[3*i] - 1 * (*x_itterator));
      x_itterator++; // x[4]
      *x_itterator = *x_itterator + dt * (-sin(*x2_itterator) * u[3*i] - 1 * (*x_itterator));
      x_itterator++; // x[5]
      *x_itterator = *x_itterator + dt * (cos(*x3_itterator) * cos(*x2_itterator) * u[3*i] - 1 * *x_itterator - 9.81);
      x_itterator++; // x[6]
      *x_itterator = *x_itterator + dt * ((1.0 / 0.5) * (u[3*i + 1] - *x_itterator));
      x_itterator++; // x[7]
      *x_itterator = *x_itterator + dt * ((1.0 / 0.5) * (u[3*i + 2] - *x_itterator));
      std::advance(x_itterator, -7);
      
      for(int j = 0; j < 8; j++){
        if(j < 3){
          cost = cost + (*Qx_itterator) * pow((*x_itterator) - (*x_ref_itterator), 2);
          x_ref_itterator++;
        }else{
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
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// Callback functions

void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const& msg)
{
  if(ufomap_msgs::msgToUfo(msg->map, myMap)) {
    map_received = true;
  } else {
    std::cout << "Conversion failed" << std::endl;
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  position_received = true;
  position_x = msg->pose.pose.position.x;
  position_y = msg->pose.pose.position.y;
  position_z = msg->pose.pose.position.z;
  velocity_x = msg->twist.twist.linear.x;
  velocity_y = msg->twist.twist.linear.y;
  velocity_z = msg->twist.twist.linear.z;
  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

// End of callback functions
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// Main

typedef rrtCache* (*arbitrary)();
typedef rrtSolverStatus (*arbitrary2)(void*, double*, double*, double, double*);
typedef void (*rrt_clearer)(rrtCache*);

int main(int argc, char *argv[])
{ 
  // Subscribers and publishers
  ros::init(argc, argv, "RRT_TREE");
  ros::NodeHandle nh;
  ros::Publisher points_pub = nh.advertise<visualization_msgs::Marker>("RRT_NODES", 1);
  ros::Publisher chosen_path_visualization_pub = nh.advertise<visualization_msgs::Marker>("CHOSEN_RRT_PATH_VISUALIZATION", 1);
  ros::Publisher output_path_pub = nh.advertise<nav_msgs::Path>("chosen_path", 1);
  ros::Publisher chosen_path_pub = nh.advertise<nav_msgs::Odometry>("REFERENCE_OUT_", 1);
  ros::Publisher all_path_pub = nh.advertise<visualization_msgs::Marker>("RRT_PATHS", 1);
  ros::Publisher goal_pub = nh.advertise<visualization_msgs::Marker>("RRT_GOALS", 1);
  ros::Publisher map_pub = nh.advertise<ufomap_msgs::UFOMapStamped>("Internal_ufo_map", 11);
  ros::Subscriber map_sub = nh.subscribe("UFOMAP_IN_", 1, mapCallback);
  ros::Subscriber sub = nh.subscribe("ODOMETRY_IN_", 1, odomCallback);
  ros::Publisher hits_pub = nh.advertise<visualization_msgs::Marker>("HITS", 1);
  ros::Publisher position_pub = nh.advertise<visualization_msgs::Marker>("POSITION", 1);
  ros::Publisher taken_path_pub = nh.advertise<visualization_msgs::Marker>("PATH_TAKEN", 1);
  ros::Publisher execution_time_pub = nh.advertise<std_msgs::Float64MultiArray>("/errt_execution_time", 1);
  ros::Rate rate(10);
  
  // Initial point
  // This manually sets the first point which the drone will travel to.
  // For each point in the path CHOSEN_PATH, one needs to add the VREF vx, vy, vz to CHOSEN_PATH_VREF.
  ros::param::get("/INITIAL_POINT_", INITIAL_POINT);
  if(INITIAL_POINT){
    ros::param::get("/INITIAL_X_", INITIAL_X);
    ros::param::get("/INITIAL_Y_", INITIAL_Y);
    ros::param::get("/INITIAL_Z_", INITIAL_Z);
    CHOSEN_PATH.push_back(new node(INITIAL_X, INITIAL_Y, INITIAL_Z));
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
  }
  
  ros::param::get("/MAP_FRAME_ID_", MAP_FRAME_ID);
  
  ros::param::get("/RUN_BY_NODES_", RUN_BY_NODES);
  ros::param::get("/NUMBER_OF_NODES_", NUMBER_OF_NODES);
  ros::param::get("/NUMBER_OF_GOALS_", NUMBER_OF_GOALS);
  ros::param::get("/NUMBER_OF_ITTERATIONS_", NUMBER_OF_ITTERATIONS);
  ros::param::get("/RESOLUTION_", RESOLUTION);
  ros::param::get("/GLOBAL_STRATEGY_THRESHOLD_", GLOBAL_STRATEGY_THRESHOLD);
  ros::param::get("/GLOBAL_PATH_THRESHOLD_", GLOBAL_PATH_THRESHOLD);

  ros::param::get("/MIN_INFO_GOAL_", MIN_INFO_GOAL);
  ros::param::get("/GOAL_SENSOR_RANGE_", GOAL_SENSOR_RANGE);
  ros::param::get("/OBB_CHECK_RADIUS_", OBB_RADIUS);
  ros::param::get("/GOAL_CHECK_DIST_", GOAL_CHECK_DIST);
  

  ros::param::get("/DISTANCE_BETWEEN_NODES_", DISTANCE_BETWEEN_NODES);
  ros::param::get("/DISTANCE_BETWEEN_GOALS_", DISTANCE_BETWEEN_GOALS);
  ros::param::get("/MINIMUM_DISTANCE_TO_GOAL_", MINIMUM_DISTANCE_TO_GOAL);
  ros::param::get("/RADIOUS_", RADIOUS);
  ros::param::get("/PLANNING_DEPTH_", PLANNING_DEPTH);
  ros::param::get("/INFO_GAIN_CHECK_DEPTH_", INFO_GAIN_CHECK_DEPTH);
  
  ros::param::get("/SCALER_AABB_", SCALER_AABB);
  
  ros::param::get("/SCALER_DISTANCE_", SCALER_DISTANCE);
  ros::param::get("/SCALER_INFORMATION_GAIN_", SCALER_INFORMATION_GAIN);
  ros::param::get("/SCALER_ACTUATION_", SCALER_ACTUATION);
  ros::param::get("/NEXT_PATH_DISTANCE_", NEXT_PATH_DISTANCE);
  ros::param::get("/NEXT_POINT_DISTANCE_", NEXT_POINT_DISTANCE);
  ros::param::get("/PATH_IMPROVEMENT_MAX_", PATH_IMPROVEMENT_MAX);
  
  ros::param::get("/SENSOR_RANGE_", SENSOR_RANGE);
  ros::param::get("/SENSOR_MIN_", SENSOR_MIN);
  ros::param::get("/SENSOR_HORIZONTAL_", SENSOR_HORIZONTAL);
  ros::param::get("/SENSOR_VERTICAL_", SENSOR_VERTICAL);
  
  ros::param::get("/NMPC_POINTS_", NMPC_POINTS);
  ros::param::get("/NMPC_DT_", NMPC_DT);
  ros::param::get("/POSITION_TRACKING_WEIGHT_X_", POSITION_TRACKING_WEIGHT_X);
  ros::param::get("/POSITION_TRACKING_WEIGHT_Y_", POSITION_TRACKING_WEIGHT_Y);
  ros::param::get("/POSITION_TRACKING_WEIGHT_Z_", POSITION_TRACKING_WEIGHT_Z);
  ros::param::get("/ANGLE_WEIGHT_ROLL_", ANGLE_WEIGHT_ROLL);
  ros::param::get("/ANGLE_WEIGHT_PITCH_", ANGLE_WEIGHT_PITCH);
  ros::param::get("/INPUT_WEIGHT_THRUST_", INPUT_WEIGHT_THRUST);
  ros::param::get("/INPUT_WEIGHT_ROLL_", INPUT_WEIGHT_ROLL);
  ros::param::get("/INPUT_WEIGHT_PITCH_", INPUT_WEIGHT_PITCH);
  ros::param::get("/INPUT_RATE_WEIGHT_THRUST_", INPUT_RATE_WEIGHT_THRUST);
  ros::param::get("/INPUT_RATE_WEIGHT_ROLL_", INPUT_RATE_WEIGHT_ROLL);
  ros::param::get("/INPUT_RATE_WEIGHT_PITCH_", INPUT_RATE_WEIGHT_PITCH);
  
  // Main
  // When the ufomap and current position have been received through their respective callback functions, the rrt executes by:
  // 1) Tuning the generation and generating goals.
  // 2) Generating the RRT tree and attatching generated goals.
  // 3) Set the path.
  // 4) Evaluates the current point.
  // 5) Trigger global strategy if necessary.
  while(ros::ok()){
    high_resolution_clock::time_point start_total = high_resolution_clock::now();
    if(map_received and not GOALS_generated and position_received){
      if(CHOSEN_PATH.empty()){
        tuneGeneration(myMap, false, true, false, position_x, position_y, position_z, PLANNING_DEPTH);
      }else{
        tuneGeneration(myMap, false, true, false, (*(--CHOSEN_PATH.end()))->point->x(), (*(--CHOSEN_PATH.end()))->point->y(), (*(--CHOSEN_PATH.end()))->point->z(), PLANNING_DEPTH);
      }

        high_resolution_clock::time_point start_total = high_resolution_clock::now();

      generateGoals(myMap, true);

    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    cout << "\nGEN GOALS time: " << duration_total.count() << " mili seconds for " << endl;
    }
    if(map_received and not RRT_created and GOALS_generated){
      if(CHOSEN_PATH.empty()){


        generateRRT(position_x, position_y, position_z);



      }else{

        high_resolution_clock::time_point start_total = high_resolution_clock::now();

        generateRRT((*(--CHOSEN_PATH.end()))->point->x(), (*(--CHOSEN_PATH.end()))->point->y(), (*(--CHOSEN_PATH.end()))->point->z());

    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    cout << "\nGEN RRT AGAIN -- time: " << duration_total.count() << " mili seconds for " << endl;
      
      }
      if(RRT_created){
        findShortestPath();
      }
      itterations = 0; //DO NOT TOUCH!
    }
    if(map_received and RRT_created){
      if(!fetched_path){   
        high_resolution_clock::time_point start_total = high_resolution_clock::now();
        setPath();

    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    cout << "\nSET PATH time: " << duration_total.count() << " mili seconds for " << endl;
    

      }
      if(fetched_path and goalNode != nullptr){  
        itterations++;
        evaluateCurrentPoint(&chosen_path_pub);
      }
      if((goalNode == nullptr and GOALS_generated)){
        //Prints for the current path can be added here
        if(initialGoalInfo < GLOBAL_STRATEGY_THRESHOLD and not recoveryUnderway){
          tuneGeneration(myMap, false, true, false, position_x, position_y, position_z, PLANNING_DEPTH);
          for(int i = 0; i < 3; i++){
            generateGoals(myMap, false);
            generateRRT(position_x, position_y, position_z);
            allowNewPath = true;     
            setPath();
            if(initialGoalInfo > GLOBAL_STRATEGY_THRESHOLD){
              break;
            }
          }
        }
        if((initialGoalInfo < GLOBAL_STRATEGY_THRESHOLD and not recoveryUnderway)){
          globalStrategy();
        }
      }
    }
    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    // cout << "\nExecution time: " << duration_total.count() << " micro seconds for " << myGoals.size() << " path/s." << endl;
    
    std_msgs::Float64MultiArray planning_time;

    planning_time.data = {duration_total.count(), ros::Time::now().toSec()};
    if (!duration_total.count() == 0) {
      execution_time_pub.publish(planning_time);
    }
    
    updatePathTaken();
    visualize(&points_pub, &output_path_pub, &chosen_path_visualization_pub, &all_path_pub, &goal_pub, &hits_pub, &taken_path_pub, &map_pub, &position_pub);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

          // ***** fix path length after NMPC calculation ** //

          // ***** computation time plot ** //
