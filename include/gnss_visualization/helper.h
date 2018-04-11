#ifndef HELPER_H_
#define HELPER_H_

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include <Eigen/Dense>

namespace helper {
  
//Function to help setting data into a ROS Vector3
geometry_msgs::Vector3 SetVector3(float x, float y, float z);

//Function to help adding two ROS Vector3
geometry_msgs::Vector3 AddVector3(geometry_msgs::Vector3 Vec1,
                                  geometry_msgs::Vector3 Vec2);

//Function to help subtracting two ROS Vector3
geometry_msgs::Vector3 SubtractVector3(geometry_msgs::Vector3 Vec1,
                                       geometry_msgs::Vector3 Vec2);

//Function to set a ROS Vector3 to zero
geometry_msgs::Vector3 ZeroVector3();

//Function to calculate the 2-norm of a ROS Vector3
float normVector3(geometry_msgs::Vector3 Vec3);

//Function to copy a ROS Vector3 into a ROS Point structure
geometry_msgs::Point Vec3_2_Point(geometry_msgs::Vector3 Vec3);

//Function to set data into a ROS Point
geometry_msgs::Point SetPoint(float x, float y, float z);

//Function to add two ROS Points
geometry_msgs::Point AddPoint(geometry_msgs::Point Pt1,
                              geometry_msgs::Point Pt2);

//Function to subtract two ROS Points
geometry_msgs::Point SubtractPoint(geometry_msgs::Point Pt1,
                                   geometry_msgs::Point Pt2);

//Function to set a ROS Point to zero
geometry_msgs::Point ZeroPoint();

//Function to calculate the 2-norm of a ROS Point
float normPoint(geometry_msgs::Point Pt);

//Function to print the values of a ROS Point (debug purposes)
void printPoint(geometry_msgs::Point Pt);

//Function to normalize a Point to be unit-norm
geometry_msgs::Point NormalizePoint(geometry_msgs::Point Pt);

//Function to copy a ROS Point into a ROS Vector3 structure
geometry_msgs::Vector3 Point2vec3(geometry_msgs::Point Pt);

//Function to copy a ROS Point into an Eigen Vector3d structure
Eigen::Vector3d Point2vec3d(geometry_msgs::Point Pt);

//Function to copy an Eigen Vector3d into a ROS Point structure
geometry_msgs::Point Vec3d2point(Eigen::Vector3d Pt);

//Function to copy a ROS Vector3 into an Eigen Vector3d structure
Eigen::Vector3d Vec32vec3d(geometry_msgs::Vector3 Pt);

//Function to print the values of an Eigen Vector3d
void printVector3d(const std::string &string,
	               const Eigen::Vector3d &Pt);

//Function to generate a skew-symmetric matrix from a vector (based on kinematics)
Eigen::Matrix3d skew(float x, float y, float z);

//Function to generate a skew-symmetric matrix from a vector (based on kinematics)
Eigen::Matrix3d skew(Eigen::Vector3d vec);

// Get a zero version of the odometry message type
nav_msgs::Odometry GetZeroOdom();

// Return a rotation matrix with v1 aligned with z, and v2 projected into x
Eigen::Matrix3d Triad(Eigen::Vector3d v1, Eigen::Vector3d v2);

// Return a rotation matrix with v1 aligned with z, and v2 projected 
// the vector specified by the yaw angle
Eigen::Matrix3d Triad(Eigen::Vector3d v1, double yaw);

// Return a quaternion with v1 aligned with z, and v2 projected 
// the vector specified by the yaw angle
Eigen::Quaterniond TriadQuat(Eigen::Vector3d v1, double yaw);

// Same as previous, but returns ROS Quaternion
geometry_msgs::Quaternion TriadQuat(geometry_msgs::Vector3 v1, double yaw);

/* Minimum function */
float min(double x, double y);

/* Maximum function */
float max(double x, double y);

// Saturate a value between minimum boundary and maximum boundary
float saturate(double in, double min_val, double max_val);

//Convert degree to radians
double deg2rad(double degVal);

//Convert radians to degrees
double rad2deg(double radVal);

}  // namespace helper

#endif  // HELPER_H_