#include <ros/ros.h>
#include <turtlesim/SpawnCircle.h>
#include <turtlesim/GetCircles.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/RemoveCircle.h>
#include <turtlesim/Kill.h>
#include <time.h>
#include <math.h>
#include <limits.h>
#include <vector>

int id = 2;
std::vector<turtlesim::Circle> cerchi;

bool spawn_circle(turtlesim::SpawnCircle::Request &richiesta, turtlesim::SpawnCircle::Response &risposta){
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
	return 1;
}

bool get_circle(turtlesim::GetCircles::Request &richiesta, turtlesim::GetCircles::Response &risposta){
	//ROS_INFO("Richiesta get");
		/* Causa eccessive stampe, dato che viene costantemente chiamata
		 * per mantenere il delete client sempre aggiornato, quindi la commento
		*/
	risposta.circles=cerchi;
}

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
	
	/*Cerco l'indice che rimuoverò se va a buon fine la chiamata alla kill.
	 * Lo cerco prima per non appesantire la chiamata alla kill e la delete
	 * in attesa che potrebbe andare in errore.
	 */ 
	int indexToRemove;
	for(indexToRemove=0; indexToRemove<cerchi.size(); indexToRemove++){
		if(cerchi[indexToRemove].id==richiesta.id){
			break;
		}
	}
	
	if (client.call(srv)){
		ROS_INFO("Call to kill Turtle%d: Done.", richiesta.id);
		//Aggiorno l'array dei cerchi rimuovendo la tartaruga eliminata
		cerchi.erase(cerchi.begin()+indexToRemove);
		risposta.circles=cerchi;
	}else{
		ROS_ERROR("Failed to call service kill");
		return 0;
	}
	return 1;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "tutorial_all_servers");
	ros::NodeHandle n;

	//Inizializzazione dell'array globale
	cerchi = std::vector<turtlesim::Circle>();
	
	ros::ServiceServer srvspawn = n.advertiseService("/spawnCircle", spawn_circle);
	ROS_INFO("Server draw_Circle Operativo");
	ros::ServiceServer srvget = n.advertiseService("/getCircle", get_circle);
	ROS_INFO("Server get_circle Operativo");
	ros::ServiceServer srvdelete = n.advertiseService("/deleteCircle", remove_circle);
	ROS_INFO("Server remove_circle Operativo");

	ros::spin();
  
  return 0;
}
