#include <gnss_visualization/arena_viz.h>

ArenaVisualization::ArenaVisualization() {

}

void ArenaVisualization::initialize(ros::NodeHandle& node) {
    arena_publisher_ = node.advertise<visualization_msgs::MarkerArray>("arena_marker", 1);

    // arena floor
    Eigen::Vector3d floor_pos(0, 0, -0.05);
    Eigen::Vector3d floor_scale(20, 5, 0.05);
    create_marker(floor_pos, floor_scale, 0, 0, 1, 0, 0.5);

    // arena box
    Eigen::Vector3d box_1(-9.14, 2.23, 0);
    Eigen::Vector3d box_2(9.24, -2.40, 3.3);
    create_marker_corners(box_1, box_2, 0, 1, 0.75, 1, 0.1);

    Eigen::Vector3d balloon_b(-6.94, 0.68, 1.39);
    Eigen::Vector3d balloon_r(7.03, -0.80, 1.36);
    create_balloon(balloon_b, 0.25, 0.25, 1, 1);
    create_balloon(balloon_r, 1, 0.25, 0.25, 1);
}


void ArenaVisualization::publish() {

    auto now = ros::Time::now();

    for(auto& m : marker_msg_.markers) {
        m.header.stamp = now;
    }

    arena_publisher_.publish(marker_msg_);
}



void ArenaVisualization::create_marker_ecef(Eigen::Vector3d pos, Eigen::Vector3d scale, double yaw,
       double r, double g, double b, double a) {
    
} 

/* Create a box marker given corner positions, in the arena frame */
void ArenaVisualization::create_marker_corners(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double yaw,
       double r, double g, double b, double a) {
    
    Eigen::Vector3d center = (pos1 + pos2) / 2.0;
    Eigen::Vector3d scale = (pos2 - pos1);

    create_marker(center, scale, yaw, r,g,b,a);
} 

/* Create a box marker given center & size, in the arena frame */
void ArenaVisualization::create_marker(Eigen::Vector3d pos, Eigen::Vector3d scale, double yaw, 
                double r, double g, double b, double a) {

    visualization_msgs::Marker m;
    
    m.header.frame_id = "wrw";
    m.header.stamp = ros::Time::now();

    m.type = visualization_msgs::Marker::CUBE;
    m.ns = "wrw";
    m.id = last_id_++;
    m.action = visualization_msgs::Marker::ADD;

    m.pose.position.x = pos[0];
    m.pose.position.y = pos[1];
    m.pose.position.z = pos[2];

    m.pose.orientation.w = 1;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;

    m.scale.x = scale[0];
    m.scale.y = scale[1];
    m.scale.z = scale[2];

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;

    marker_msg_.markers.push_back(m);
}

void ArenaVisualization::create_balloon(Eigen::Vector3d pos,  
                double r, double g, double b, double a) {

    visualization_msgs::Marker m;
    
    m.header.frame_id = "wrw";
    m.header.stamp = ros::Time::now();

    m.type = visualization_msgs::Marker::SPHERE;
    m.ns = "wrw";
    m.id = last_id_++;
    m.action = visualization_msgs::Marker::ADD;

    m.pose.position.x = pos[0];
    m.pose.position.y = pos[1];
    m.pose.position.z = pos[2];

    m.pose.orientation.w = 1;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;

    m.scale.x = 0.5;
    m.scale.y = 0.5;
    m.scale.z = 0.75;

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;

    marker_msg_.markers.push_back(m);
}
