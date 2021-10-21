#include <ros/ros.h>
#include <turtlesim/GetCircles.h>
#include <turtlesim/SpawnCircle.h>
#include <string.h>

bool get_circle(turtlesim::GetCircles::Request &richiesta, turtlesim::GetCircles::Response &risposta){
	ROS_INFO("Richiesta get");
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<turtlesim::SpawnCircle>("spawnCircle");
	turtlesim::SpawnCircle srv;
	
	srv.request.x=-1.0;
	srv.request.y=-1.0;
	if (client.call(srv)){
		ROS_INFO("Call to spawn_circle to get Circles[]: Done");
	}else{
		ROS_ERROR("Failed to spawn_circle to get Circles[]");
		return 0;
	}
	risposta.circles=srv.response.circles;
	return 1;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "tutorial_get_circle_server");
	ros::NodeHandle n;
  
	ros::ServiceServer servizio = n.advertiseService("/getCircle", get_circle);
	ROS_INFO("Server get_Circle Operativo");
	ros::spin();
  
  return 0;
}
