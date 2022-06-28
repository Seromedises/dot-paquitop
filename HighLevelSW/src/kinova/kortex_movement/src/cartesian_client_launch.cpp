
#include <string>
#include <string.h>
#include "ros/ros.h"
#include "kortex_movement/cartesian_movement.h"  //Aggiunto per avere il riferimento sul server

int main(int argc, char  **argv)
{
	struct coordinates {
	   double  x;
	   double   y;
	   double	 z;
	   double   thetax;
	   double   thetay;
	   double	 thetaz;

	} req;

	ros::init(argc, argv, "cartesian_client_launch");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<kortex_movement::cartesian_movement>("cartesian_movement");
	ROS_INFO("LOW LEVEL CODE. Not calculate spaces for camera!");
	ROS_INFO("usage: put x, y, z coordinates and thetax, thetay, thetaz angles: ");

	ros::param::get("/cartesian_client_launch/x", req.x);
	ros::param::get("/cartesian_client_launch/y", req.y);
	ros::param::get("/cartesian_client_launch/z", req.z);
	ros::param::get("/cartesian_client_launch/thetax", req.thetax);
	ros::param::get("/cartesian_client_launch/thetay", req.thetay);
	ros::param::get("/cartesian_client_launch/thetaz", req.thetaz);
/*
	n.getParam("x", req.x);
	n.getParam("y", req.y);
	n.getParam("z", req.z);
	n.getParam("thetax", req.thetax);
	n.getParam("thetay", req.thetay);
	n.getParam("thetaz", req.thetaz);
*/

	kortex_movement::cartesian_movement srv;

	srv.request.x =  (float)(req.x);
	srv.request.y =  (float)(req.y);
	srv.request.z =  (float)(req.z);
	srv.request.thetax =  (float)(req.thetax);
	srv.request.thetay =  (float)(req.thetay);
	srv.request.thetaz =  (float)(req.thetaz);

	if(client.call(srv) && srv.response.output == true)
	{
		ROS_INFO("Action delivered to client");
	}
	else
	{
		ROS_ERROR("FAILED TO CALL SERVICE");
		return 1;
	}

	return 0;
}
