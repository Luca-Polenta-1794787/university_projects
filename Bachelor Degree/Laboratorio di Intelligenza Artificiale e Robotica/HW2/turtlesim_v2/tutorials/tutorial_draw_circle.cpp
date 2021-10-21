#include <ros/ros.h>
#include <turtlesim/SpawnCircle.h>
#include <time.h>
#include <math.h>
#include <limits.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "tutorial_spawn_circle");
	ros::NodeHandle nh;

	ros::ServiceClient client = nh.serviceClient<turtlesim::SpawnCircle>("spawnCircle");
	ROS_INFO("Client spawn_Circle avviato: verranno generatore n tartarughe (con n valore random tra 2 e 10) in posizioni random e tutte rivolte verso l'alto per default (quest'ultima scelta del programmatore)");

	turtlesim::SpawnCircle srv;
	srand(time(NULL));
	int i;
	int n = rand()%9+2;
	float minXY = 0.5;
	float masXY = 10.5;
	for(i=0; i<n; i++){
		srv.request.x = (float(rand())/float((RAND_MAX)))*(masXY-minXY)+minXY;
		srv.request.y = (float(rand())/float((RAND_MAX)))*(masXY-minXY)+minXY;		
		if (client.call(srv)){
			ROS_INFO("Call to draw_circle: Done");
		}else{
			ROS_ERROR("Failed to call service draw_circle");
			return 1;
		}
	}
  
  return 0;
}
