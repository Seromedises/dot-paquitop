# High Level Software

The folder [HighLevelSW/src](https://github.com/Seromedises/dot-paquitop/tree/main/HighLevelSW/src) collects all the file needed for the High Level funtioning of PAQUITOP platform.

## Table of content

- [High Level Software](#high-level-software)
  - [Table of content](#table-of-content)
  - [Subfolders Contained](#subfolders-contained)
  - [Installation instructions](#installation-instructions)
    - [ROS Installation](#ros-installation)
    - [Catkin Make install](#catkin-make-install)
    - [Conan Install](#conan-install)
    - [Matlab Engine](#matlab-engine)
    - [OpenCV](#opencv)
    - [Realsense](#realsense)
    - [Compiling al the local ROS Code](#compiling-al-the-local-ros-code)

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
  - [Rplidar](http://wiki.ros.org/rplidar) used for start lidar node.

- [paquitop](src/paquitop) this package contain the general launch for all the code that are required to use the platform. The most complete launch is [paquitop_twomachine.launch](src/paquitop/launch/paquitop_twomachine.launch), while the most complete without the use of the robotic arm is [paquitop_twomachine_kinovaless.launch](src/paquitop/launch/paquitop_twomachine_kinovaless.launch). The following packages launch specific node that are usefull for the overall functioning:

  - [tf.launch](src/paquitop/launch/tf.launch) that launches all the static transform between the origins of the reference system of the elements that compose the system.
  
  - [rviz.launch](src/paquitop/launch/rviz.launch) that launches the visualizer of the ROS system.
  
  - [rviz_camera.launch](src/paquitop/launch/rviz_camera.launch) is a package that share the graphic visualization of RVIZ in the Graphic User Interface.

## Installation instructions

### ROS Installation

[ROS](http://wiki.ros.org/) is Robot Operating System, usefull to put in comunication all the components of the ropotic system.
Follow these instruction yo install ROS on your PC:

```text
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $ (lsb_release -sc) main" > /etc/apt/sources.list.d/roslatest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.as c | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall pythonrosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

### Catkin Make install

[Catkin Make](wiki.ros.org/catkin) is usefull to compile local packages that are not downoladed from an online repository, using  `sudo apt`.

The following instruction are usefull to install Catkin Make:

```text
sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev build-essential
sudo apt-get install ros-melodic-catkin
```

If there are any problem to the compiler installation path, use this instructions:

```text
sudo apt install ccache
sudo /usr/sbin/update-ccache-symlinks
export PATH="/usr/lib/ccache/:$PATH"
```

### Conan Install

[Conan](https://pypi.org/project/conan/) is a package that manage the execution of C and C++ code.
The instruccion for install the package is `pip install conan`

### Matlab Engine

[Matlab engine](https://it.mathworks.com/help/matlab/matlab-engine-for-python.html) is a package that make possible the interaction between Matlab and Python. It makes possibile to execute Matlab scripts in a Python code.
To install the Matlab engine write the following instructions in the **Matlab Command Window**:

```matlab
cd (fullfile(matlabroot,'extern','engines','python'))
system('sudo python setup.py install')
```

### OpenCV

[OpenCV](https://opencv.org/) is a python library used to image and video manipulation. Is used in AI context to recognize the framed objects.
To install the library use the following instruction: `sudo apt install python3-opencv`

### Realsense

[Realsense](https://github.com/IntelRealSense/realsense-ros) is a library usefull to interact with the camera developed by Intel, such as L515 and T265.

The ROS package can be installed as follow:

```text
sudo apt install ros-melodic-ddynamic-reconfigure-* 
sudo apt install ros-melodic-realsense2-*
```

The python library can be installed as follow:

```text
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt install librealsense2-dkms 
sudo apt install librealsense2-utils 
sudo apt install librealsense2-dev 
sudo apt install librealsense2-dbg
```

### Compiling al the local ROS Code

These are the instructions to run in a terminal to create the workspace, clone the paquitop repository and install the necessary ROS dependencies:

```text
sudo apt install python3 python3-pip
sudo python3 -m pip install conan
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default
cd && git clone https://github.com/Seromedises/dot-paquitop.git
cd dot-paquitop/HighLevelSW
rosdep install --from-paths src --ignore-src -y
```

Then, to build and source the workspace:

```text
catkin_make
echo "source ~/dot-paquitop/HighLevelSW/devel/setup.bash" >> ~/.bashrc
```

You can also build against one of the ARMv8 builds of the Kortex API with Conan if you specify the CONAN_TARGET_PLATFORM CMake argument when using catkin_make. The following platforms are supported:

- Artik 710:

```text
catkin_make --cmake-args -DCONAN_TARGET_PLATFORM=artik710
echo "source ~/dot-paquitop/HighLevelSW/devel/setup.bash" >> ~/.bashrc
```

- IMX6:

```text
catkin_make --cmake-args -DCONAN_TARGET_PLATFORM=imx6
echo "source ~/dot-paquitop/HighLevelSW/devel/setup.bash" >> ~/.bashrc
```

- NVidia Jetson:

```text
catkin_make --cmake-args -DCONAN_TARGET_PLATFORM=jetson
echo "source ~/dot-paquitop/HighLevelSW/devel/setup.bash" >> ~/.bashrc
```
