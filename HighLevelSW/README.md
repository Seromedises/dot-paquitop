# High Level Software

The folder [HighLevelSW/src](https://github.com/Seromedises/dot-paquitop/tree/main/HighLevelSW/src) collects all the file needed for the High Level funtioning of PAQUITOP platform.

## Subfolders Contained

The High Level is structured in Subfolder where each perform a particular task:

- [follow_waypoints](src/follow_waypoints) contain the code that performs the periodically pubblication of the next waypoint when the actual waypoint is sufficient near to robot platform;
- [gui_interface](src/gui_interface) contain the codes that produces the two graphic user interfaces that allow both to the amministrative user and to the patient to interact with the robot during his working.
- [kinova](src/kinova) cointain the codes that perform all the movement and the interfaces with the robotic arm:

  - [camera_robot_interaction](src/kinova/camera_robot_interaction/) is the package that performs the interaction between the RGB-D camera, [Intel L515](https://www.intelrealsense.com/lidar-camera-l515/) and the [Kinova Gen3 Lite](https://www.kinovarobotics.com/product/gen3-lite-robots).

  - [kortex_movement](src/kinova/kortex_movement/) is the package that performs all the general purpose movements executed by the robotic arm.

  - [kortex_tools](src/kinova/kortex_tools/) is the package that performs all the tools useful for the calibration between RGB-D camera and robotic arm.
  
  - [ros_kortex](src/kinova/ros_kortex/) contain all the [ROS packages developed by Kinova](https://github.com/Kinovarobotics/ros_kortex) that allow the initialization of Robotic Arm in ROS.

- [navstack_pub](src/navstack_pub) contain all the launch files and the configuration parameters to correctly use the packages used by the autonomous navigation. The packages are:
  - [Extended Kalman Filter](http://wiki.ros.org/robot_pose_ekf) used to have a better estimate of position of the robot;
  - [Hector Maping](http://wiki.ros.org/hector_mapping) used to have the map of the enviroment;
  - [Move Base](http://wiki.ros.org/move_base) used to plan the trajectory of the robot reading the obstacle occupancy from the map and the lidar;
  - [Intel T265 Camera](http://wiki.ros.org/realsense2_camera) used for odometry node from camera;
  - [Rplidar](http://wiki.ros.org/rplidar) used for start lidar node
