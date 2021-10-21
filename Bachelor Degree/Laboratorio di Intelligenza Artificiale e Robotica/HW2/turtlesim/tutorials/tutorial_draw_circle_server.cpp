#include <ros/ros.h>
#include <turtlesim/SpawnCircle.h>
#include <turtlesim/Spawn.h>
#include <vector>

int id = 2;
std::vector<turtlesim::Circle> cerchi;

bool spawn_circle(turtlesim::SpawnCircle::Request &richiesta, turtlesim::SpawnCircle::Response &risposta){
	if(richiesta.x!=-1.0 && richiesta.y!=-1.0){
		ROS_INFO("Richiesta di generare una tartaruga in posizione: x=%f, y=%f", (float)richiesta.x, (float)richiesta.y);
		ros::NodeHandle nh;
		ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("spawn");		
		turtlesim::Spawn srv;
		
		srv.request.x = (float)richiesta.x;
		srv.request.y = (float)richiesta.y;
		srv.request.theta = M_PI/2;	//Per orientare le tartarughe verso l'alto, scelta personale
		
		turtlesim::Circle cerchio;
		cerchio.id=id;
		cerchio.x=srv.request.x;
		cerchio.y=srv.request.y;
		cerchi.push_back(cerchio);
		id++;
		
		if (client.call(srv)){
			ROS_INFO("Call to spawn Turtle%d: Done.", cerchio.id);
		}else{
			ROS_ERROR("Failed to call service spawn");
			return 0;
		}
		risposta.circles=cerchi;
	}else{
		ROS_INFO("Richiesta da una get dell'array dei cerchi attualmente disponibile");
		risposta.circles=cerchi;
	}
	return 1;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "tutorial_draw_circle_server");
	ros::NodeHandle n;

	//Inizializzazione dell'array globale (nel caso di una get prima della spawn)
	cerchi = std::vector<turtlesim::Circle>();
	
	ros::ServiceServer servizio = n.advertiseService("/spawnCircle", spawn_circle);
	ROS_INFO("Server draw_Circle Operativo");
	ros::spin();
  
  return 0;
}
