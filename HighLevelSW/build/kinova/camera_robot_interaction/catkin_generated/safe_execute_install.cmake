execute_process(COMMAND "/home/paquitop/dot-paquitop/HighLevelSW/build/kinova/camera_robot_interaction/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/paquitop/dot-paquitop/HighLevelSW/build/kinova/camera_robot_interaction/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
