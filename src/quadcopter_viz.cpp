#include <gnss_visualization/quadcopter_viz.h>
#include <gnss_visualization/gnss_visualization.h>

QuadcopterVisualization::QuadcopterVisualization() {

}

void QuadcopterVisualization::initialize(ros::NodeHandle& node) {
    publisher_ = node.advertise<visualization_msgs::MarkerArray>("quadcopter_marker", 1);

    for(auto name : quad_names) {
        create_quad_marker(name, 0.5, 1, 1, 1);
    }
}

void QuadcopterVisualization::publish() {

    auto now = ros::Time::now();

    for(auto& m : marker_msg_.markers) {
        m.header.stamp = now;
    }

    publisher_.publish(marker_msg_);
}

void QuadcopterVisualization::create_quad_marker(std::string& name, double r, double g, double b, double a) {

    visualization_msgs::Marker m;
    
    m.header.frame_id = name;
    m.header.stamp = ros::Time::now();

    m.type = visualization_msgs::Marker::MESH_RESOURCE;
    m.mesh_resource = "package://gnss_visualization/meshes/Quadcopter.STL";
    m.ns = name;
    m.id = 0;
    m.action = visualization_msgs::Marker::ADD;

    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = -0.05;

    m.pose.orientation.w = 1;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;

    m.scale.x = 1;
    m.scale.y = 1;
    m.scale.z = 1;

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;

    marker_msg_.markers.push_back(m);

    // Text marker 
    visualization_msgs::Marker t;

    t.header.frame_id = name;
    t.header.stamp = ros::Time::now();

    t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    t.text = name;
    t.ns = name;
    t.id = 1;
    t.action = visualization_msgs::Marker::ADD;

    t.pose.position.x = 0;
    t.pose.position.y = 0;
    t.pose.position.z = +0.2;

    t.pose.orientation.w = 1;
    t.pose.orientation.x = 0;
    t.pose.orientation.y = 0;
    t.pose.orientation.z = 0;

    t.scale.x = 0.3;
    t.scale.y = 0.3;
    t.scale.z = 0.3;

    t.color.r = 1;
    t.color.g = 1;
    t.color.b = 1;
    t.color.a = 1;

    marker_msg_.markers.push_back(t);
}
