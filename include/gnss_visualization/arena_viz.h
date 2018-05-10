#ifndef _ARENA_VIZ_H
#define _ARENA_VIZ_H

#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <Eigen/Dense>

class ArenaVisualization {

    public:
        ArenaVisualization(void);

        void initialize(ros::NodeHandle& node);
        void publish(void);

    protected:
        visualization_msgs::MarkerArray marker_msg_;
        ros::Publisher arena_publisher_;
        int last_id_ = 1;

        void create_marker(Eigen::Vector3d pos, Eigen::Vector3d scale, double yaw, 
                double r, double g, double b, double a); 

        void create_marker_corners(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double yaw, 
                double r, double g, double b, double a); 

        void create_marker_ecef(Eigen::Vector3d pos, Eigen::Vector3d scale, double yaw,
                double r, double g, double b, double a);

        void create_balloon(Eigen::Vector3d pos, double r, double g, double b, double a);
};

#endif
