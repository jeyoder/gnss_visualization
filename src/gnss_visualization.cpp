#include "gnss_visualization/gnss_visualization.h"
#include "gnss_visualization/arena_viz.h"
#include "gnss_visualization/quadcopter_viz.h"

#include <string>
#include <vector>

/* Coordinate frame tree! This is important *
 *
 *                  ecef          (ECEF origin (0,0,0))
 *                   |
 *                   V
 *                 refnet         (ENU frame, centered on WRW0 antenna)
 *                   |
 *         +---------+-----+--> local ("Local" frame, generated on first position fix, this is used so if you start in an unknown
 *         |               |            location, the visualization isn't just blank, most stuff should use the "wrw" frame)
 *         V               |       
 *        wrw              V            ("WRW" frame - centered on arena, rotated 6ish degrees from ENU. Used for arena/machine games stuff)
 *         |           yoga_primary     (Yoga GNSS primary position - refnet-relative ENU, based directly from SBRTK message data)
 *         |               |
 *         |               V
 *         |           yoga_secondary   (Yoga GNSS secondary position - refnet-relative ENU(??), based on A2D message data) 
 *         V
 *       yoga_odom                      (Yoga local_odom data - CURRENTLY reported in the wrw frame (based on gps_kf or gpsImuNode data)            
 */

// global variables to keep track of camera / user state :)
// Name scheme for these variables: (name)_(position|orientation)_(frame) 

Eigen::Vector3d     refnet_position_ecef;       /* Refnet antenna position in ECEF frame. Currently populated on receipt of first SBRTK message */
Eigen::Quaterniond  refnet_orientation_ecef;    /* On receipt of refnet_position_ecef, this quaternion is calculated from the Recef_enu matrix
                                                 * to generate an ENU frame centered at the refnet antenna */

Eigen::Vector3d     wrw_position_ecef(-742015.08, -5462218.80, 3198013.75);
Eigen::Vector3d     wrw_position_refnet(0,0,0);        /* WRW (arena center) position in refnet ENU frame. */
Eigen::Quaterniond  wrw_orientation_refnet;     /* WRW frame orientation in refnet ENU frame. */

Eigen::Vector3d     yoga_primary_position_refnet; /* Yoga primary GNSS antenna position, in refnet ENU frame. Calculated on receipt of SBRTK message.
                                                   * Note that the yoga_primary orientation is the identity quaternion, i.e. objects in the yoga_primary
                                                   * frame retain ENU orientation. */

Eigen::Vector3d     yoga_secondary_position_primary; /* Yoga secondary GNSS antenna position, in the primary antenna ENU frame. Calculated on A2D message. */

Eigen::Vector3d     yoga_odom_position_wrw(0, 0, 0);
Eigen::Quaterniond  yoga_odom_orientation_wrw(0, 1, 0, 0);

Eigen::Vector3d     local_position_refnet;

Eigen::Matrix3d Recef2enu_refnet; /* Rotation matrix to ENU at refnet antenna. populated on receipt of sbrtk message. */

/* Arena (WRW) frame parameters */

Eigen::Vector3d arena_offset_enu(0, 0, -0.75);
const double thetaWRW = 6.2*(3.141592654)/180;
const double wrwMat[] =  {
    cos(thetaWRW), -sin(thetaWRW), 0,
    sin(thetaWRW),  cos(thetaWRW), 0,
    0,              0,             1 
};
Eigen::Matrix3d arena_orientation_enu(wrwMat);


/* Quadcopter list */
std::vector<std::string> quad_names = {
    "phoenix",
    "hippogriff",
    "gryphon",
    "pegasus",
    "harry",
    "ron",
    "hermione",
    "luna"
};

bool got_first_message = false;

ros::Publisher pub_vis;	 // Publisher for visualization in RVIZ
visualization_msgs::MarkerArray marker_array_msg;

ArenaVisualization arena_viz;
QuadcopterVisualization quad_viz;

/* Convenience method to publish a transform to ROS, based on an offset & rotation from a given parent frame. */
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


/* Publish all output ROS messages */
/* the "world" reference frame is refnet ENU */
void publish_output() {
    Eigen::Quaterniond identity;
    identity.setIdentity();

    ros::Time now;
    
    /* publish refnet frame */
    now = publish_transform(refnet_position_ecef, refnet_orientation_ecef, 
            "ecef", "refnet");

    /* publish wrw frame */
    wrw_position_refnet = Recef2enu_refnet * (wrw_position_ecef - refnet_position_ecef);
    wrw_orientation_refnet = Eigen::Quaterniond(arena_orientation_enu);
    publish_transform(wrw_position_refnet, wrw_orientation_refnet,
            "refnet", "wrw");

    /* publish yoga GNSS frames */
    publish_transform(yoga_primary_position_refnet, identity,
            "refnet", "yoga_primary");
    publish_transform(yoga_secondary_position_primary, identity,
            "yoga_primary", "yoga_secondary");

    /* publish yoga local_odom frame (in WRW) */
    publish_transform(yoga_odom_position_wrw, yoga_odom_orientation_wrw,
            "wrw", "yoga_odom");


    marker_array_msg.markers[0].header.stamp = now;
    pub_vis.publish(marker_array_msg);

    arena_viz.publish();
    quad_viz.publish();
}


void on_attitude2d_message(const gbx_ros_bridge_msgs::Attitude2D msg) {

    if(msg.bitfield != 7) return;
//    rover_position_ecef[0] = msg.azAngle;

    double PI = 3.14159265358979;

    // note: WXYZ
//        rover_orientation_enu = Eigen::Quaterniond(cos((msg.azAngle - PI/2.0)/2), 0, 0, 
//            -sin((msg.azAngle - PI/2.0)/2)); 
       
        /* secondary position, relative to primary, in ecef frame */ 
        Eigen::Vector3d rover_secondary_position_relative_ecef;
        rover_secondary_position_relative_ecef[0] = msg.rx; 
        rover_secondary_position_relative_ecef[1] = msg.ry; 
        rover_secondary_position_relative_ecef[2] = msg.rz; 

        yoga_secondary_position_primary = Recef2enu_refnet * rover_secondary_position_relative_ecef;

        publish_output();
}

void on_sbrtk_message(const gbx_ros_bridge_msgs::SingleBaselineRTK msg) {

    /* Only accept fixed solutions */
    if(msg.bitfield != 7) return;

        /* Recover Refnet ECEF position based on its r{x,y,z} messages
         * (yes, this is a bit silly) */
        refnet_position_ecef[0] = msg.rxRov - msg.rx;
        refnet_position_ecef[1] = msg.ryRov - msg.ry;
        refnet_position_ecef[2] = msg.rzRov - msg.rz;

        Recef2enu_refnet = get_ecef2enu_matrix(refnet_position_ecef);
        refnet_orientation_ecef = Eigen::Quaterniond(Recef2enu_refnet);

        /* Rover ECEF position (relative to refnet) */
        Eigen::Vector3d primary_rel_ecef;
        primary_rel_ecef[0] = msg.rx;
        primary_rel_ecef[1] = msg.ry;
        primary_rel_ecef[2] = msg.rz;

        yoga_primary_position_refnet = Recef2enu_refnet * primary_rel_ecef;

        if(!got_first_message) {
            local_position_refnet = yoga_primary_position_refnet - Eigen::Vector3d(0, 0, -1.5);
            got_first_message = true;
        }   

        publish_output();
}

void on_local_odom_message(const  nav_msgs::Odometry msg) {

    yoga_odom_position_wrw[0] = msg.pose.pose.position.x;
    yoga_odom_position_wrw[1] = msg.pose.pose.position.y;
    yoga_odom_position_wrw[2] = msg.pose.pose.position.z;
    
    yoga_odom_orientation_wrw = Eigen::Quaterniond(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

    publish_output();
}

/* Event handlers for messages from quads */
void on_quad_sbrtk_message(const gbx_ros_bridge_msgs::SingleBaselineRTK::ConstPtr& msg, std::string& quad_name) {
//    ROS_INFO("[gnss_visualization] Received remote SBRTK message: ");

    Eigen::Quaterniond identity;
    identity.setIdentity();

    Eigen::Vector3d pos_ecef_rel(msg->rx, msg->ry, msg->rz);
    Eigen::Vector3d pos_refnet = Recef2enu_refnet * pos_ecef_rel;

    publish_transform(pos_refnet, identity, "refnet", quad_name+"_primary"); 
}

void on_quad_a2d_message(const gbx_ros_bridge_msgs::Attitude2D::ConstPtr& msg, std::string& quad_name) {
 //   ROS_INFO("[gnss_visualization] Received remote A2D message: ");

    Eigen::Quaterniond identity;
    identity.setIdentity();

    Eigen::Vector3d pos_ecef_rel(msg->rx, msg->ry, msg->rz);
    Eigen::Vector3d pos_refnet = Recef2enu_refnet * pos_ecef_rel;

    publish_transform(pos_refnet, identity, "refnet", quad_name+"_secondary"); 
}

void on_quad_local_odom_message(const nav_msgs::Odometry::ConstPtr& msg, std::string& quad_name) {

    Eigen::Vector3d pos_wrw(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond orientation_wrw(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    publish_transform(pos_wrw, orientation_wrw, "wrw", quad_name);
}

/* Coordinate frames according to rviz: 
 *
 * refnet: Refnet centered ENU (default)
 */

int main(int argc, char** argv){
	ros::init(argc, argv, "gnss_visualization");
    ros::NodeHandle node = ros::NodeHandle("~");
	ROS_INFO("gnss_visualization node started!");

    /* Load ROS launch parameters */
    double arena_center_ecef_x, arena_center_ecef_y, arena_center_ecef_z;
    node.getParam("arena_center_ecef_x", arena_center_ecef_x);
    node.getParam("arena_center_ecef_y", arena_center_ecef_y);
    node.getParam("arena_center_ecef_z", arena_center_ecef_z);
    wrw_position_ecef[0] = arena_center_ecef_x;
    wrw_position_ecef[1] = arena_center_ecef_y;
    wrw_position_ecef[2] = arena_center_ecef_z;
    std::cout << "Centering wrw frame on "<<arena_center_ecef_x<<" "<<arena_center_ecef_y<<" "<<arena_center_ecef_z << "\n";

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

	std::string frame_id = "yoga_primary";
	std::string ns = "yoga_primary";
	double size = scale;
	int seq_number = 1;
	MeshMarker(obj_offset, q_0, frame_id, ns, file_3d,
	           size, seq_number, &marker_array_msg);
	pub_vis = node.advertise
		<visualization_msgs::MarkerArray>("rover_marker", 1);

    // Initialize sub visualizations 
    arena_viz.initialize(node);
    quad_viz.initialize(node);

	// Set subscribers to ppengine output messages
	ros::Subscriber a2d_subscriber = node.subscribe<gbx_ros_bridge_msgs::Attitude2D>
        ("/yoga/Attitude2D", 10, on_attitude2d_message);

	ros::Subscriber sbrtk_subscriber = node.subscribe<gbx_ros_bridge_msgs::SingleBaselineRTK>
        ("/yoga/SingleBaselineRTK", 10, on_sbrtk_message);

	ros::Subscriber imunode_subscriber = node.subscribe<nav_msgs::Odometry>
        ("/yoga/local_odom", 10, on_local_odom_message);
    
    std::vector<ros::Subscriber> subs;
    // Subscribe to what the quads are saying 
    for(std::string& name : quad_names) {
        ROS_INFO("Registering subscription for");

        subs.push_back(node.subscribe<gbx_ros_bridge_msgs::SingleBaselineRTK>
            ("/" + name + "/SingleBaselineRTK", 10, boost::bind(on_quad_sbrtk_message, _1, name)));

        subs.push_back(node.subscribe<gbx_ros_bridge_msgs::Attitude2D>
            ("/" + name + "/Attitude2D", 10, boost::bind(on_quad_a2d_message, _1, name)));

        subs.push_back(node.subscribe<nav_msgs::Odometry>
            ("/" + name + "/local_odom", 10, boost::bind(on_quad_local_odom_message, _1, name)));
    }

	// ROS loop that starts callbacks/publishers

	const double rate = 200.0;
	ros::Rate loop_rate(rate);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
