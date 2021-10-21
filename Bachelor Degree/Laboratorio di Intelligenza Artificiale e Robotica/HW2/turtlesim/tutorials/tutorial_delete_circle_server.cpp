#include <ros/ros.h>
#include <turtlesim/RemoveCircle.h>
#include <turtlesim/Kill.h>
#include <string.h>

bool remove_circle(turtlesim::RemoveCircle::Request &richiesta, turtlesim::RemoveCircle::Response &risposta){
	ROS_INFO("Richiesta di eliminazione per la tartaruga con l'id=%d", (uint8_t)richiesta.id);
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<turtlesim::Kill>("kill");
	turtlesim::Kill srv;
		
	std::string nome = "turtle";
	char idstr[8];
	sprintf(idstr, "%d", richiesta.id);
	nome=nome+idstr;
	
	srv.request.name=nome;
	
	if (client.call(srv)){
		ROS_INFO("Call to kill Turtle%d: Done.", richiesta.id);
	}else{
		ROS_ERROR("Failed to call service kill");
		return 0;
	}
	return 1;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "tutorial_delete_circle_server");
	
	ros::NodeHandle n;
	ros::ServiceServer servizio = n.advertiseService("/deleteCircle", remove_circle);
	ROS_INFO("Server remove_Circle Operativo");
	ros::spin();
  
  return 0;
}
