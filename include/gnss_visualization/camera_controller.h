#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

#include <rviz/properties/tf_frame_property.h>
#include <rviz/default_plugin/view_controllers/fps_view_controller.h>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

namespace gnss_visualization_camera_controller {

    class FullFPSViewController : public rviz::ViewController {
		Q_OBJECT
        public:
        FullFPSViewController();
	
        void reset();
        void activate();
        void onInitialize();
        void onChangedTargetFrame();
		void update(float dt, float ros_dt);

		protected:
        rviz::TfFrameProperty* target_frame_property_;
		Ogre::Quaternion camera_orientation_;
		Ogre::Vector3 camera_position_;
    };
}

#endif
