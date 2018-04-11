#include "gnss_visualization/helper.h"

namespace helper {

geometry_msgs::Vector3 SetVector3(float x, float y, float z){
	geometry_msgs::Vector3 Vec;
	Vec.x = x;
	Vec.y = y;
	Vec.z = z;
	return Vec;
}

geometry_msgs::Vector3 AddVector3(geometry_msgs::Vector3 Vec1, 
	                              geometry_msgs::Vector3 Vec2){
	geometry_msgs::Vector3 Vec_out;
	Vec_out.x = Vec1.x + Vec2.x;
	Vec_out.y = Vec1.y + Vec2.y;
	Vec_out.z = Vec1.z + Vec2.z;
	return Vec_out;
}

geometry_msgs::Vector3 SubtractVector3(geometry_msgs::Vector3 Vec1,
                                       geometry_msgs::Vector3 Vec2){
	geometry_msgs::Vector3 Vec_out;
	Vec_out.x = Vec1.x - Vec2.x;
	Vec_out.y = Vec1.y - Vec2.y;
	Vec_out.z = Vec1.z - Vec2.z;
	return Vec_out;
}

geometry_msgs::Vector3 ZeroVector3(){
	geometry_msgs::Vector3 Vec;
	Vec.x = 0.0;
	Vec.y = 0.0;
	Vec.z = 0.0;
	return Vec;
}

float normVector3(geometry_msgs::Vector3 Vec3){
	double norm;
	norm = sqrt(pow(Vec3.x,2) + pow(Vec3.y,2) + pow(Vec3.z,2));
	return norm;
}

geometry_msgs::Point Vec3_2_Point(geometry_msgs::Vector3 Vec3){
	geometry_msgs::Point Pt;
	Pt.x = Vec3.x;
	Pt.y = Vec3.y;
	Pt.z = Vec3.z;
	return Pt;
}

geometry_msgs::Point SetPoint(float x, float y, float z){
	geometry_msgs::Point Pt;
	Pt.x = x;
	Pt.y = y;
	Pt.z = z;
	return Pt;
}

geometry_msgs::Point AddPoint(geometry_msgs::Point Pt1,
                              geometry_msgs::Point Pt2){
	geometry_msgs::Point Pt_out;
	Pt_out.x = Pt1.x + Pt2.x;
	Pt_out.y = Pt1.y + Pt2.y;
	Pt_out.z = Pt1.z + Pt2.z;
	return Pt_out;
}

geometry_msgs::Point SubtractPoint(geometry_msgs::Point Pt1,
                                   geometry_msgs::Point Pt2){
	geometry_msgs::Point Pt_out;
	Pt_out.x = Pt1.x - Pt2.x;
	Pt_out.y = Pt1.y - Pt2.y;
	Pt_out.z = Pt1.z - Pt2.z;
	return Pt_out;
}

geometry_msgs::Point ZeroPoint(){
	geometry_msgs::Point Pt;
	Pt.x = 0.0;
	Pt.y = 0.0;
	Pt.z = 0.0;
	return Pt;
}

float normPoint(geometry_msgs::Point Pt){
	double norm;
	norm = sqrt(pow(Pt.x,2) + pow(Pt.y,2) + pow(Pt.z,2));
	return norm;
}

void printPoint(geometry_msgs::Point Pt){
	ROS_INFO("Value %f %f %f", Pt.x, Pt.y, Pt.z);
}

geometry_msgs::Point NormalizePoint(geometry_msgs::Point Pt) {
	double norm = normPoint(Pt);
	double epsilon = 0.001;
	if(norm > epsilon) {
		return SetPoint(Pt.x/norm, Pt.y/norm, Pt.z/norm);
	} else {
		return SetPoint(0.0, 0.0, 0.0);
	}
}

geometry_msgs::Vector3 Point2vec3(geometry_msgs::Point Pt){
	geometry_msgs::Vector3 Vec3;
	Vec3.x = Pt.x;
	Vec3.y = Pt.y;
	Vec3.z = Pt.z;
	return Vec3;
}

Eigen::Vector3d Point2vec3d(geometry_msgs::Point Pt) {
	return Eigen::Vector3d(Pt.x, Pt.y, Pt.z);
}

geometry_msgs::Point Vec3d2point(Eigen::Vector3d Pt) {
	return SetPoint(Pt[0], Pt[1], Pt[2]);
}

Eigen::Vector3d Vec32vec3d(geometry_msgs::Vector3 Pt) {
	return Eigen::Vector3d(Pt.x, Pt.y, Pt.z);
}

void printVector3d(const std::string &string,
	               const Eigen::Vector3d &Pt) {
	ROS_INFO("%s:\t%f\t%f\t%f", string.c_str(), Pt[0], Pt[1], Pt[2]);
}

Eigen::Matrix3d skew(float x, float y, float z){
	Eigen::Matrix3d M;

	M <<  0, -z,  y,
	      z,  0, -x,
	     -y, x,  0;

	return M;
} 

Eigen::Matrix3d skew(Eigen::Vector3d vec) {
	return skew(vec[0], vec[1], vec[2]);
}

nav_msgs::Odometry GetZeroOdom() {
	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "world";
	odom.pose.pose.position = SetPoint(0.0, 0.0, 0.0);
	odom.pose.pose.orientation.w = 1.0;
	odom.twist.twist.linear = SetVector3(0.0, 0.0, 0.0);
	odom.twist.twist.angular = SetVector3(0.0, 0.0, 0.0);
	return odom;
}

Eigen::Matrix3d Triad(Eigen::Vector3d v1, Eigen::Vector3d v2) {
	Eigen::Vector3d z_bdes, x_cdes, y_bdes, x_bdes;
	Eigen::Matrix3d Rot;
	z_bdes = v1.normalized();
	x_cdes = v2;
	y_bdes = (z_bdes.cross(x_cdes)).normalized();
	x_bdes = y_bdes.cross(z_bdes);
	Rot << x_bdes, y_bdes, z_bdes;
	return Rot;
}

Eigen::Matrix3d Triad(Eigen::Vector3d v1, double yaw) {
	Eigen::Vector3d v2;
	v2 << cos(yaw), sin(yaw), 0;
	return Triad(v1, v2);
}

Eigen::Quaterniond TriadQuat(Eigen::Vector3d v1, double yaw) {
	Eigen::Matrix3d R = Triad(v1, yaw);
	Eigen::Quaterniond q(R);
	return q;
}

geometry_msgs::Quaternion TriadQuat(geometry_msgs::Vector3 v1, double yaw) {
	Eigen::Vector3d v1_eigen(v1.x, v1.y, v1.z);
	Eigen::Quaterniond q_eigen = TriadQuat(v1_eigen, yaw);
	geometry_msgs::Quaternion q;
	q.w = q_eigen.w();
	q.x = q_eigen.x();
	q.y = q_eigen.y();
	q.z = q_eigen.z();
	return q;
}

/* Minimum function */
float min(double x, double y)
{
	return (x < y) ? x : y;
}

/* Maximum function */
float max(double x, double y)
{
	return (x > y) ? x : y;
}

// Saturate a value between minimum boundary and maximum boundary
float saturate(double in, double min_val, double max_val){
	return min(max(in, min_val), max_val);
}

//Convert degree to radians
double deg2rad(double degVal){
	return degVal*M_PI/180.0;
}

//Convert radians to degrees
double rad2deg(double radVal){
	return radVal*180.0/M_PI;
}

}  // namespace helper
