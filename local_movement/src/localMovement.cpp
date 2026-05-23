#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

const double PI = 3.1415926;

double speed = 0.2;
double yawRate = 0.4;
double slowDis = 0.1;
double slowAngle = 0.2;
double stopDis = 0.02;
double stopAngle = 0.04;
bool collisionStop = true;
double obstacleHeightThre = 0.05;
double vehicleLX = 0.5;
double vehicleLY = 0.5;
double stopMargin = 0.1;

pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());

float vehicleX = 0, vehicleY = 0, vehicleZ = 0, vehicleYaw = 0;
float goalX = 0, goalY = 0, goalYaw = 0;

float vehicleLSmallX = 0, vehicleLSmallY = 0;
float vehicleLLargeX = 0, vehicleLLargeY = 0;
float vehicleRLarge = 0;

double odomTime = 0;
int moveState = 0;

int frontStop = 0;
int backStop = 0;
int leftStop = 0;
int rightStop = 0;
int cwStop = 0;
int ccwStop = 0;

rclcpp::Node::SharedPtr nh;
shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Joy>> pubJoystick;
shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PointStamped>> pubWaypoint;

void pubJoyMsg(float joyFwd, float joyLeft, float joyYaw, float mode)
{
  sensor_msgs::msg::Joy joy;

  joy.axes.push_back(joyYaw);
  joy.axes.push_back(0);
  joy.axes.push_back(mode);
  joy.axes.push_back(joyLeft);
  joy.axes.push_back(joyFwd);
  joy.axes.push_back(-mode);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(1);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);

  joy.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
  joy.header.frame_id = "local_movement";
  pubJoystick->publish(joy);
}

void pubWaypointMsg()
{
  geometry_msgs::msg::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
  waypoint.point.x = vehicleX;
  waypoint.point.y = vehicleY;
  waypoint.point.z = vehicleZ;
  pubWaypoint->publish(waypoint);
}

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x;
  vehicleY = odomIn->pose.pose.position.y;
  vehicleZ = odomIn->pose.pose.position.z;
}

void moveHandler(const geometry_msgs::msg::Pose::ConstSharedPtr move)
{
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = move->orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  goalX = vehicleX + cos(vehicleYaw) * move->position.x - sin(vehicleYaw) * move->position.y;
  goalY = vehicleY + sin(vehicleYaw) * move->position.x + cos(vehicleYaw) * move->position.y;

  goalYaw = vehicleYaw + yaw;
  if (goalYaw > PI) goalYaw -= 2 * PI;
  else if (goalYaw < -PI) goalYaw += 2 * PI;

  moveState = 1;

  RCLCPP_INFO(nh->get_logger(), "Local movement started, x %.2f, y %.2f, heading %.2f.", move->position.x, move->position.y, yaw);
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if (joy->header.frame_id != "local_movement" && moveState == 1) {
    pubWaypointMsg();
    pubJoyMsg(0, 0, 0, 1.0);
    moveState = 0;

    RCLCPP_INFO(nh->get_logger(), "Local movement interrupted.");
  }
}

void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloud2)
{
  frontStop = 0;
  backStop = 0;
  leftStop = 0;
  rightStop = 0;
  cwStop = 0;
  ccwStop = 0;

  if (collisionStop) {
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

    pcl::PointXYZI point;
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      float disX = point.x - vehicleX;
      float disY = point.y - vehicleY;
      float disXY = sqrt(disX * disX + disY * disY);

      if (disXY > vehicleRLarge || terrainCloud->points[i].intensity < obstacleHeightThre) continue;

      float disX2 = cos(vehicleYaw) * disX + sin(vehicleYaw) * disY;
      float disY2 = -sin(vehicleYaw) * disX + cos(vehicleYaw) * disY;

      if (fabs(disY2) < vehicleLSmallY) {
        if (disX2 > -vehicleLLargeX && disX2 < 0) {
          if (disY2 < 0) cwStop++;
          else ccwStop++;
          backStop++;
        } else if (disX2 > 0 && disX2 < vehicleLLargeX) {
          if (disY2 < 0) ccwStop++;
          else cwStop++;
          frontStop++;
        }
      }

      if (fabs(disX2) < vehicleLSmallX) {
        if (disY2 > -vehicleLLargeY && disY2 < 0) {
          if (disX2 < 0) ccwStop++;
          else cwStop++;
          rightStop++;
        } else if (disY2 > 0 && disY2 < vehicleLLargeY) {
          if (disX2 < 0) cwStop++;
          else ccwStop++;
          leftStop++;
        }
      }
    }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("localMovement");

  nh->declare_parameter<double>("speed", speed);
  nh->declare_parameter<double>("yawRate", yawRate);
  nh->declare_parameter<double>("slowDis", slowDis);
  nh->declare_parameter<double>("slowAngle", slowAngle);
  nh->declare_parameter<double>("stopDis", stopDis);
  nh->declare_parameter<double>("stopAngle", stopAngle);
  nh->declare_parameter<bool>("collisionStop", collisionStop);
  nh->declare_parameter<double>("obstacleHeightThre", obstacleHeightThre);
  nh->declare_parameter<double>("vehicleLX", vehicleLX);
  nh->declare_parameter<double>("vehicleLY", vehicleLY);
  nh->declare_parameter<double>("stopMargin", stopMargin);

  nh->get_parameter("speed", speed);
  nh->get_parameter("yawRate", yawRate);
  nh->get_parameter("slowDis", slowDis);
  nh->get_parameter("slowAngle", slowAngle);
  nh->get_parameter("stopDis", stopDis);
  nh->get_parameter("stopAngle", stopAngle);
  nh->get_parameter("collisionStop", collisionStop);
  nh->get_parameter("obstacleHeightThre", obstacleHeightThre);
  nh->get_parameter("vehicleLX", vehicleLX);
  nh->get_parameter("vehicleLY", vehicleLY);
  nh->get_parameter("stopMargin", stopMargin);

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odomHandler);

  auto subMove = nh->create_subscription<geometry_msgs::msg::Pose> ("/local_movement", 5, moveHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);

  auto subTerrainCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_map", 5, terrainCloudHandler);

  pubJoystick = nh->create_publisher<sensor_msgs::msg::Joy>("/joy", 5);

  pubWaypoint = nh->create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 5);

  vehicleLSmallX = vehicleLX / 2;
  vehicleLLargeX = vehicleLSmallX + stopMargin;
  vehicleLSmallY = vehicleLY / 2;
  vehicleLLargeY = vehicleLSmallY + stopMargin;
  vehicleRLarge = sqrt(vehicleLLargeX * vehicleLLargeX + vehicleLLargeY * vehicleLLargeY);

  rclcpp::Rate rate(20);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (moveState == 1) {
      float deltaX = cos(vehicleYaw) * (goalX - vehicleX) + sin(vehicleYaw) * (goalY - vehicleY);
      if (deltaX < 0 && backStop > 20) deltaX = 0;
      else if (deltaX > 0 && frontStop > 20) deltaX = 0;

      float deltaY = -sin(vehicleYaw) * (goalX - vehicleX) + cos(vehicleYaw) * (goalY - vehicleY);
      if (deltaY < 0 && rightStop > 20) deltaY = 0;
      else if (deltaY > 0 && leftStop > 20) deltaY = 0;

      float deltaXY = sqrt(deltaX * deltaX + deltaY * deltaY);
      if (deltaXY < 0.001) deltaXY = 0.001;

      float deltaYaw = goalYaw - vehicleYaw;
      if (deltaYaw > PI) deltaYaw -= 2 * PI;
      else if (deltaYaw < -PI) deltaYaw += 2 * PI;
      if (deltaYaw < 0 && cwStop > 20) deltaYaw = 0;
      else if (deltaYaw > 0 && ccwStop > 20) deltaYaw = 0;

      float joyFwd = speed * deltaX / deltaXY;
      float joyLeft = 2.0 * speed * deltaY / deltaXY;
      if (deltaXY < slowDis) {
        joyFwd *= deltaXY / slowDis;
        joyLeft *= deltaXY / slowDis;
      }

      float joyYaw = yawRate;
      if (deltaYaw < 0) joyYaw *= -1;
      if (fabs(deltaYaw) < slowAngle) {
        joyYaw *= fabs(deltaYaw) / slowAngle;
      }

      if (deltaXY > stopDis || fabs(deltaYaw) > stopAngle) {
        pubJoyMsg(joyFwd, joyLeft, joyYaw, 1.0);
      } else {
        pubWaypointMsg();
        pubJoyMsg(0, 0, 0, 1.0);
        moveState = 0;

        RCLCPP_INFO(nh->get_logger(), "Local movement completed.");
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
