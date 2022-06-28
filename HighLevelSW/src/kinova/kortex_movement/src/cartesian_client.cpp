
#include "ros/ros.h"
#include "kortex_movement/cartesian_movement.h"  //Aggiunto per avere il riferimento sul server

int main(int argc, char  **argv)
{
	ros::init(argc, argv, "cartesian_client");
	ROS_INFO("LOW LEVEL CODE. Not calculate spaces for camera!");
	ROS_INFO("usage: put x, y, z coordinates and thetax, thetay, thetaz angles: ");

	if (argc != 7)
	{
		ROS_ERROR("Wrong number of argument");
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<kortex_movement::cartesian_movement>("cartesian_movement");

	kortex_movement::cartesian_movement srv;

	srv.request.x = atof(argv[1]);
	srv.request.y = atof(argv[2]);
	srv.request.z = atof(argv[3]);
	srv.request.thetax = atof(argv[4]);
	srv.request.thetay = atof(argv[5]);
	srv.request.thetaz = atof(argv[6]);

	if(client.call(srv) && srv.response.output == true)
	{
		ROS_INFO("Action delivered to server");
	}
	else
	{
		ROS_ERROR("FAILED TO CALL SERVICE");
		return 1;
	}
/*
	if (srv.response.output == true)
	{
		ROS_INFO("Action Done");
	}
*/
	return 0;
}
