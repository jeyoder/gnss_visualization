// Cpp libraries
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include "gnss_visualization/helper.h"
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
//#include "gbx_ros_bridge_msgs/Attitude2D.h"
//#include "gbx_ros_bridge_msgs/SingleBaselineRTK.h"

ros::Publisher pub_vis;	 // Publisher for visualization in RVIZ

ros::Time tf_pub(const Eigen::Vector3d &point,
	             const Eigen::Quaterniond &quat,
	             const std::string parent_frame,
	             const std::string child_frame){
  static tf::TransformBroadcaster br;
  // ROS_INFO("Publishing into frame: %s", msg->header.frame_id.c_str());
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(point[0], point[1], point[2]));
  tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());
  transform.setRotation(q);
  ros::Time time_now = ros::Time::now();
  br.sendTransform(
  		tf::StampedTransform(transform, time_now,
                             parent_frame, child_frame));
  return time_now;
}

ros::Time tf_pub(const geometry_msgs::Point &point,
	             const geometry_msgs::Quaternion &quat,
	             const std::string parent_frame,
	             const std::string child_frame){
  static tf::TransformBroadcaster br;
  // ROS_INFO("Publishing into frame: %s", msg->header.frame_id.c_str());
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(point.x, point.y, point.z));
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  transform.setRotation(q);
  ros::Time time_now = ros::Time::now();
  br.sendTransform(
  		tf::StampedTransform(transform, time_now,
                             parent_frame, child_frame));
  return time_now;
}

void MeshMarker(const Eigen::Vector3d &point,
	            const Eigen::Quaterniond &quat,
	            const std::string &frame_id,
	            const std::string &ns,  // namespace
	            const std::string &file_3d,
	            const double &size,
	            const int &seqNumber,
	            visualization_msgs::MarkerArray *markerArray) {
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource = "package://gnss_visualization/meshes/" + file_3d;
	marker.action = visualization_msgs::Marker::ADD;
	// marker.color = color;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 0.0;
	marker.scale.x = size;
	marker.scale.y = size;
	marker.scale.z = size;
	marker.ns = ns;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();
	marker.pose.orientation.w = quat.w();
	marker.pose.orientation.x = quat.x();
	marker.pose.orientation.y = quat.y();
	marker.pose.orientation.z = quat.z();
	marker.mesh_use_embedded_materials = true;

	geometry_msgs::Point position;
	position.x = size*point(0);
	position.y = size*point(1);
	position.z = size*point(2);
	marker.pose.position = position;
	marker.id = seqNumber;
	// marker.lifetime = ros::Duration(1.0);
	markerArray->markers.push_back(marker);
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg,
	              visualization_msgs::MarkerArray& object_marker) {
	// Send object pose to tf tree
	ros::Time time_now = tf_pub(msg->pose.position, 
		                        msg->pose.orientation, 
		                        "world", "object");
	// Asteroid visualization
	object_marker.markers[0].header.stamp = time_now;
	pub_vis.publish(object_marker);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "gnss_visualization");
	ros::NodeHandle node("~");
	ROS_INFO("GNSS view started!");
	const double rate = 200.0;
	ros::Rate loop_rate(rate);

	// Get 3d file for rendering
	std::string file_3d;
	node.getParam("file_3d", file_3d);

	// Get scaling for rendered object
	double scale;
	node.getParam("scale_object", scale);
	
	// Get object mesh, offset and scale it
	Eigen::Vector3d obj_offset(0.0, 0.0, 0.0);
	Eigen::Vector3d origin(0.0, 0.0, 0.0);
	Eigen::Quaterniond q_0(1.0, 0.0, 0.0, 0.0);
	std::string frame_id = "object";
	std::string ns = "object";
	double size = scale;
	int seq_number = 1;
	visualization_msgs::MarkerArray object_marker;
	MeshMarker(obj_offset, q_0, frame_id, ns, file_3d,
	           size, seq_number, &object_marker);
	pub_vis = node.advertise
		<visualization_msgs::MarkerArray>("object_marker", 1);

	// Set subscribers to object and camera pose
	ros::Subscriber obj_subs;
	std::string obj_position_topic, obj_attitude2d_topic, obj_pose_topic;
	node.getParam("object_position_topic", obj_position_topic);
	node.getParam("object_attitude2d_topic", obj_attitude2d_topic);
	node.getParam("object_pose_topic", obj_pose_topic);
	ros::Subscriber obj_pose_sub = node.subscribe<geometry_msgs::PoseStamped>(obj_pose_topic, 10, 
		boost::bind(poseCallback, _1, object_marker));
	
	ROS_INFO("[gnss_visualization]: Subscribing to: %s", obj_pose_topic.c_str());
 	
	

	// ROS loop that starts callbacks/publishers
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}