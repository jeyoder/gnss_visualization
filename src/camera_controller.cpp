#include <rviz/frame_manager.h>
#include <rviz/display_context.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/enum_property.h>

#include <OgreCamera.h>

#include "gnss_visualization/camera_controller.h"
#include <iostream>

namespace gnss_visualization_camera_controller {


    FullFPSViewController::FullFPSViewController() {
		 target_frame_property_ = new rviz::TfFrameProperty( "Target Frame", rviz::TfFrameProperty::FIXED_FRAME_STRING,
    	"TF frame representing camera position", this, NULL, true );
    }   

    void FullFPSViewController::reset() {
        /* maybe I should do something */
    }

    void FullFPSViewController::onInitialize() {
        target_frame_property_->setFrameManager( context_->getFrameManager() );
    }

    void FullFPSViewController::activate() {
        /* Register Qt slot when a different target frame is selected */
        connect( target_frame_property_, SIGNAL( changed() ), this, SLOT( onChangedTargetFrame() ));
    }

    /* set us up for a different target frame */
    void FullFPSViewController::onChangedTargetFrame() {
        std::cout << " Changed target frame \n";
    }

	void FullFPSViewController::update(float dt, float ros_dt) {

		std::cout << "Update [tf: " << target_frame_property_->getFrameStd() << "]\n";

		Ogre::Vector3 new_reference_position;
        Ogre::Quaternion new_reference_orientation;
  
        bool got_transform = context_->getFrameManager()->getTransform( target_frame_property_->getFrameStd(), 
                ros::Time(),  new_reference_position, new_reference_orientation );
        
        std::cout << got_transform;

        if( got_transform ) {
            camera_position_ = new_reference_position;
            camera_orientation_ = new_reference_orientation;
        }

        std::cout << "Position: X: " << camera_position_.x << " Y: " << camera_position_.y << " Z: " <<
            camera_position_.z << "\n";

        Ogre::Vector3 look_up_axis(1, 0, 0);
        Ogre::Radian look_up_angle(3.141592654/2);
        Ogre::Quaternion look_up; 
        look_up.FromAngleAxis(look_up_angle, look_up_axis);

        camera_->setPosition(camera_position_);
        camera_->setOrientation(look_up * camera_orientation_);
	}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( gnss_visualization_camera_controller::FullFPSViewController , rviz::ViewController )
