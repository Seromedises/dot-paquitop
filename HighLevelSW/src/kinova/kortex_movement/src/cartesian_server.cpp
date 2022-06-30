/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2019 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */
#include <thread>
#include <atomic>

#include "ros/ros.h"
//#include <kortex_driver/Base_ClearFaults.h>
//#include <kortex_driver/PlayCartesianTrajectory.h>
#include <kortex_driver/CartesianSpeed.h>
// #include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/PlayJointTrajectory.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/CartesianReferenceFrame.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/ActionType.h>
#include <kortex_driver/GetMeasuredCartesianPose.h>
#include <kortex_driver/OnNotificationActionTopic.h>

#include <ros/callback_queue.h>
#include "kortex_movement/cartesian_movement.h"  //Aggiunto per avere il riferimento sul server


#define HOME_ACTION_IDENTIFIER 2

int identifier = 1001;

bool all_notifs_succeeded = true;
float command[6] = {0,0,0,0,0,0};


std::atomic<int> last_action_notification_event{0};
std::atomic<int> last_action_notification_id{0};

void notification_callback(const kortex_driver::ActionNotification& notif)
{
  last_action_notification_event = notif.action_event;
  last_action_notification_id = notif.handle.identifier;
}

bool wait_for_action_end_or_abort()
{
  while (ros::ok())
  {
    if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_END)
    {
      ROS_INFO("Received ACTION_END notification for action %d", last_action_notification_id.load());
      return true;
    }
    else if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_ABORT)
    {
      ROS_INFO("Received ACTION_ABORT notification for action %d", last_action_notification_id.load());
      all_notifs_succeeded = false;
      return false;
    }
    ros::spinOnce();
  }
  return false;
}

bool example_send_joint_angles(ros::NodeHandle n, const std::string &robot_name, int degrees_of_freedom)
{
  last_action_notification_event = 0;
  // Initialize the ServiceClient
  ros::ServiceClient service_client_play_joint_trajectory = n.serviceClient<kortex_driver::PlayJointTrajectory>("/" + robot_name + "/base/play_joint_trajectory");
  kortex_driver::PlayJointTrajectory service_play_joint_trajectory;

  float angles_to_send[degrees_of_freedom];
  /*
  for (unsigned int i = 0; i < degrees_of_freedom; i++)
  {
    angles_to_send.push_back(0.0);
  }
  angles_to_send[0] = (-3.0);
  angles_to_send[1] = (21.0);
  angles_to_send[2] = (145.0);
  angles_to_send[3] = (-88.0+180);
  angles_to_send[4] = (-33.0);
  angles_to_send[5] = (-87.0);
  */
  angles_to_send[0] = (0.0);
  angles_to_send[1] = (105.0);
  angles_to_send[2] = (148.0);
  angles_to_send[3] = (90.0);
  angles_to_send[4] = (45.0);
  angles_to_send[5] = (90.0);

  for (int i = 0; i < degrees_of_freedom; i++)
  {
    kortex_driver::JointAngle temp_angle;
    temp_angle.joint_identifier = i;
    temp_angle.value = angles_to_send[i];
    service_play_joint_trajectory.request.input.joint_angles.joint_angles.push_back(temp_angle);
  }

  if (service_client_play_joint_trajectory.call(service_play_joint_trajectory))
  {
    ROS_INFO("The robot is sent to the standard pose.");
  }
  else
  {
    std::string error_string = "Failed to call PlayJointTrajectory";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return wait_for_action_end_or_abort();
}


bool example_home_the_robot(ros::NodeHandle n, const std::string &robot_name)
{
  ros::ServiceClient service_client_read_action = n.serviceClient<kortex_driver::ReadAction>("/" + robot_name + "/base/read_action");
  kortex_driver::ReadAction service_read_action;

  // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
  service_read_action.request.input.identifier = HOME_ACTION_IDENTIFIER;

  if (!service_client_read_action.call(service_read_action))
  {
    std::string error_string = "Failed to call ReadAction";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // We can now execute the Action that we read 
  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input = service_read_action.response.output;
  
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("The Home position action was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return wait_for_action_end_or_abort();
}


bool example_cartesian_action(ros::NodeHandle n, const std::string &robot_name)
{
  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.1f;
  my_cartesian_speed.orientation = 15.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  my_constrained_pose.target_pose.x = command[0];
  my_constrained_pose.target_pose.y = command[1];
  my_constrained_pose.target_pose.z = command[2];
  my_constrained_pose.target_pose.theta_x = command[3];
  my_constrained_pose.target_pose.theta_y = command[4];
  my_constrained_pose.target_pose.theta_z = command[5];

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  service_execute_action.request.input.name = "pose";
  service_execute_action.request.input.handle.identifier = identifier;
  service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::REACH_POSE;
  
  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Waiting for the pose 1 to end
  wait_for_action_end_or_abort();

  identifier = identifier +1;

  return true;
}

// This function sets the reference frame to the robot's base
bool example_set_cartesian_reference_frame(ros::NodeHandle n, const std::string &robot_name)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_set_cartesian_reference_frame = n.serviceClient<kortex_driver::SetCartesianReferenceFrame>("/" + robot_name + "/control_config/set_cartesian_reference_frame");
  kortex_driver::SetCartesianReferenceFrame service_set_cartesian_reference_frame;

  service_set_cartesian_reference_frame.request.input.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  if (!service_client_set_cartesian_reference_frame.call(service_set_cartesian_reference_frame))
  {
    std::string error_string = "Failed to call SetCartesianReferenceFrame";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  else
  {
    ROS_INFO("Set reference to base frame.");
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  return true;
}

bool recieve_command(kortex_movement::cartesian_movement::Request &req, kortex_movement::cartesian_movement::Response &res)
{
  command[0] = req.x;
  command[1] = req.y;
  command[2] = req.z;
  command[3] = req.thetax;
  command[4] = req.thetay;
  command[5] = req.thetaz;

  for(int i = 0; i<6; i++)
    {
      ROS_INFO("%f", command[i]);
      
    }
    ROS_INFO("\n");

  res.output = 1;

  return 1;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "cartesian_server");
  ros::NodeHandle n;
  std::string robot_name = "my_gen3_lite";
  int degrees_of_freedom = 6;

  ROS_INFO("LOW LEVEL CODE. Not calculate spaces for camera!");
  // Parameter robot_name
  if (!ros::param::get("~robot_name", robot_name))
  {
    std::string error_string = "Parameter robot_name was not specified, defaulting to " + robot_name + " as namespace";
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using robot_name " + robot_name + " as namespace";
    ROS_INFO("%s", error_string.c_str());
  }

  // We need to call this service to activate the Action Notification on the kortex_driver node.
  ros::ServiceClient service_client_activate_notif = n.serviceClient<kortex_driver::OnNotificationActionTopic>("/" + robot_name + "/base/activate_publishing_of_action_topic");
  kortex_driver::OnNotificationActionTopic service_activate_notif;
  if (service_client_activate_notif.call(service_activate_notif))
  {
    ROS_INFO("Action notification activated!");
  }
  else 
  {
    std::string error_string = "Action notification publication failed";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  if (!ros::param::get("/" + robot_name + "/degrees_of_freedom", degrees_of_freedom))
  {
    std::string error_string = "Parameter /" + robot_name + "/degrees_of_freedom was not specified, defaulting to " + std::to_string(degrees_of_freedom) + " as degrees of freedom";
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using degrees_of_freedom " + std::to_string(degrees_of_freedom) + " as degrees_of_freedom";
    ROS_INFO("%s", error_string.c_str());
  }

  ros::Duration(1.00).sleep();
  ros::Subscriber sub = n.subscribe("/" + robot_name  + "/action_topic", 1000, notification_callback);

  // Run 
  example_set_cartesian_reference_frame(n, robot_name);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  example_send_joint_angles(n, robot_name, degrees_of_freedom);
  
  // last for control the last pose
  float last[6] = {0,0,0,0,0,0};
  ROS_INFO("Server Ready");

  while (ros::ok())
  {
    ros::ServiceServer service = n.advertiseService("cartesian_movement", recieve_command);
    
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  
  
    bool equal = 1;
    for(int i = 0; i<6; i++)
    {
      // controll last command to the new input
      if(last[i] != command[i])
      {
        equal = 0;
      }
    }

    if (equal == 0)
    {
      example_cartesian_action(n, robot_name);  
      for(int i = 0; i<6; i++)
      {
        last[i] = command[i];
      }
      ROS_INFO ("Server Ready");
    }

    // ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
      
  }
  

  

  

  // Report success for testing purposes
  // ros::param::set("/kortex_examples_test_results/cartesian_poses_with_notifications_cpp", success);
  

  return 0;
}
