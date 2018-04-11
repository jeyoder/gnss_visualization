# Dependencies

- Eigen

```sudo apt-get install libeigen3-dev ```

# Compiling

```
cd ~/catkin_ws/src
git clone https://github.com/radionavlab/gnss_visualization.git
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

# Running
roslaunch gnss_visualization gnss.launch

# Parameters in launch file
file_3d -  name of a 3d object in .stl or .dae format. This object will be published into RVIZ
scale_object -  If object is too small, too big, or not in meters, it can be scaled using this parameter
object_pose_topic - topic that publishes the pose of the object. The message has to be of the type geometry_msgs/PoseStamped
