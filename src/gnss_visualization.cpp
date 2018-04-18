#include "gnss_visualization/gnss_visualization.h"

ros::Publisher pub_vis;	 // Publisher for visualization in RVIZ
ros::Publisher arena_publisher;
visualization_msgs::MarkerArray marker_array_msg;
visualization_msgs::MarkerArray arena_msg;

/* Publish a coordinate transform definition, so it can be known by RVIZ */
ros::Time publish_transform(const Eigen::Vector3d &point,
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

/* Create the MarkerArray message for the arena */
void create_arena_marker(ros::NodeHandle node) {

    arena_publisher = node.advertise<visualization_msgs::MarkerArray>("arena_marker", 1);

    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker marker;

    auto arena_frame = "arena";

    marker.header.frame_id = "arena";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::CUBE;
    marker.ns = "arena";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = -0.05;
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;

    marker.scale.x = 20;
    marker.scale.y = 5;
    marker.scale.z = 0.05;

    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 0.5;

    msg.markers.push_back(marker);

    visualization_msgs::Marker chimney;

    chimney.header.frame_id = "arena";
    chimney.header.stamp = ros::Time::now();

    chimney.type = visualization_msgs::Marker::CUBE;
    chimney.ns = "chimney";
    chimney.id = 1;
    chimney.action = visualization_msgs::Marker::ADD;

    chimney.pose.position.x = 0;
    chimney.pose.position.y = 0;
    chimney.pose.position.z = +0.5;
    chimney.pose.orientation.w = 1;
    chimney.pose.orientation.x = 0;
    chimney.pose.orientation.y = 0;
    chimney.pose.orientation.z = 0;

    chimney.scale.x = 0.5;
    chimney.scale.y = 0.5;
    chimney.scale.z = 1;

    chimney.color.r = 0.5;
    chimney.color.g = 0.5;
    chimney.color.b = 0;
    chimney.color.a = 0.9;
    chimney.lifetime = ros::Duration();

    msg.markers.push_back(chimney);

    arena_msg = msg;
}


/* Create a mesh-based rviz Marker */
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

/* Generate a rotation matrix from ECEF to local ENU coordinates for 
 * the given ECEF position. Translated to Eigen from the corresponding 
 * function in GSS navtoolbox.cpp*/
Eigen::Matrix3d get_ecef2enu_matrix(Eigen::Vector3d rEcef) {
    
    /* WGS-84 Earth parameters */

    const double aa = 6378137.00000;
    const double bb = 6356752.31425;
    const double ee = 0.0818191908334158;
    const double ep = sqrt((aa*aa - bb*bb)/(bb*bb)); 


    double x = rEcef[0], y = rEcef[1], z = rEcef[2];
    double lambda = atan2(y, x);
    double p = sqrt(x*x + y*y);
    double theta = atan2(z*aa, p*bb);
    double phi = atan2(z + ep*ep*bb*pow(sin(theta),3), p - ee*ee*aa*pow(cos(theta),3));

    Eigen::Matrix3d ret;
    ret.setZero();

    ret(0,0) = -sin(lambda);
    ret(0,1) = cos(lambda);
    ret(0,2) = 0;
    ret(1,0) = -sin(phi)*cos(lambda);
    ret(1,1) = -sin(phi)*sin(lambda);
    ret(1,2) = cos(phi);
    ret(2,0) = cos(phi)*cos(lambda);
    ret(2,1) = cos(phi)*sin(lambda);
    ret(2,2) = sin(phi);

    return ret;
}

// global variables to keep track of camera / user state :)
Eigen::Vector3d rover_position_ecef;
Eigen::Vector3d rover_position_refnet_ecef;
Eigen::Vector3d rover_position_refnet_enu;
Eigen::Vector3d rover_secondary_position_relative_ecef;
Eigen::Vector3d rover_secondary_position_relative_enu;
Eigen::Vector3d local_reference_refnet_enu;
Eigen::Vector3d refnet_position_ecef;

Eigen::Vector3d arena_position_refnet_enu(2, 0, 0);

Eigen::Quaterniond rover_orientation_enu;

double rover_azimuth = 0;

bool got_first_message = false;

void publish_output() {
    Eigen::Quaterniond identity;
    identity.setIdentity();

    ros::Time now = publish_transform(rover_position_refnet_enu, rover_orientation_enu, "world", "rover"); 
    publish_transform(local_reference_refnet_enu, identity, "world", "local");
    publish_transform(rover_secondary_position_relative_enu+rover_position_refnet_enu, rover_orientation_enu, "world", "secondary");
    publish_transform(arena_position_refnet_enu, identity, "world", "arena");
    marker_array_msg.markers[0].header.stamp = now;
    pub_vis.publish(marker_array_msg);

    arena_msg.markers[0].header.stamp = now;
    arena_msg.markers[1].header.stamp = now;
    arena_publisher.publish(arena_msg);
}


void on_attitude2d_message(const gbx_ros_bridge_msgs::Attitude2D msg) {
    ROS_INFO("[gnss_visualization] Received Attitude2D message!");

    if(msg.bitfield != 7) return;
//    rover_position_ecef[0] = msg.azAngle;

    double PI = 3.14159265358979;

    // note: WXYZ
    rover_orientation_enu = Eigen::Quaterniond(cos((msg.azAngle - PI/2.0)/2), 0, 0, 
            -sin((msg.azAngle - PI/2.0)/2)); 

    rover_secondary_position_relative_ecef[0] = msg.rx; 
    rover_secondary_position_relative_ecef[1] = msg.ry; 
    rover_secondary_position_relative_ecef[2] = msg.rz; 

    publish_output();
}

void on_sbrtk_message(const gbx_ros_bridge_msgs::SingleBaselineRTK msg) {
    ROS_INFO("[gnss_visualization] Received SingleBaselineRTK message!");

    /* Only accept fixed solutions */
    if(msg.bitfield != 7) return;


    /* Recover Refnet ECEF position based on its r{x,y,z} messages
     * (yes, this is a bit silly) */
    refnet_position_ecef[0] = msg.rxRov - msg.rx;
    refnet_position_ecef[1] = msg.ryRov - msg.ry;
    refnet_position_ecef[2] = msg.rzRov - msg.rz;

    Eigen::Matrix3d xform = get_ecef2enu_matrix(refnet_position_ecef);

    /* Rover ECEF position (relative to refnet) */
    rover_position_refnet_ecef[0] = msg.rx;
    rover_position_refnet_ecef[1] = msg.ry;
    rover_position_refnet_ecef[2] = msg.rz;

    rover_position_refnet_enu = xform * rover_position_refnet_ecef;
    rover_secondary_position_relative_enu = xform * rover_secondary_position_relative_ecef;

    if(!got_first_message) {
        local_reference_refnet_enu = rover_position_refnet_enu;
        local_reference_refnet_enu[2] -= 1;
        got_first_message = true;
    }   

    publish_output();
}


/* Coordinate frames according to rviz: 
 *
 * refnet: Refnet centered ENU (default)
 */

int main(int argc, char** argv){
	ros::init(argc, argv, "gnss_visualization");
    ros::NodeHandle node = ros::NodeHandle("~");
	ROS_INFO("gnss_visualization node started!");

	// Get 3d file for rendering
	std::string file_3d;
	node.getParam("file_3d", file_3d);

	// Get scaling for rendered object
	double scale;
	node.getParam("scale_object", scale);
	
	// Get object mesh, offset and scale it
	Eigen::Vector3d obj_offset(0.28, -0.05, -0.2);
	Eigen::Vector3d origin(0.0, 0.0, 0.0);
    
    Eigen::Vector3d rot_axis(0.0, 0.0, 1.0);
    double angle = 3.14159 / 2;
    double sinTerm = sin(angle/2);
	Eigen::Quaterniond q_0(cos(angle / 2), rot_axis[0] * sinTerm, rot_axis[1] * sinTerm, rot_axis[2] * sinTerm);

	std::string frame_id = "rover";
	std::string ns = "rover";
	double size = scale;
	int seq_number = 1;
	MeshMarker(obj_offset, q_0, frame_id, ns, file_3d,
	           size, seq_number, &marker_array_msg);
	pub_vis = node.advertise
		<visualization_msgs::MarkerArray>("rover_marker", 1);

    create_arena_marker(node);

	// Set subscribers to ppengine output messages
	ros::Subscriber a2d_subscriber = node.subscribe<gbx_ros_bridge_msgs::Attitude2D>
        ("/yoga/Attitude2D", 10, on_attitude2d_message);

	ros::Subscriber sbrtk_subscriber = node.subscribe<gbx_ros_bridge_msgs::SingleBaselineRTK>
        ("/yoga/SingleBaselineRTK", 10, on_sbrtk_message);

	// ROS loop that starts callbacks/publishers

	const double rate = 200.0;
	ros::Rate loop_rate(rate);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
