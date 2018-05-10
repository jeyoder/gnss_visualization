#ifndef _QUADCOPTER_VIZ_H
#define _QUADCOPTER_VIZ_H

#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <Eigen/Dense>

class QuadcopterVisualization {

    public:
        QuadcopterVisualization(void);

        void initialize(ros::NodeHandle& node);
        void publish(void);

    protected:
        visualization_msgs::MarkerArray marker_msg_;
        ros::Publisher publisher_;

        void create_quad_marker(std::string& name, double r, double g, double b, double a); 
};

#endif
