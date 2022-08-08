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
  - []
