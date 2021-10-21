#include <ros/ros.h>
#include <turtlesim/RemoveCircle.h>
#include <turtlesim/GetCircles.h>
#include <turtlesim/Pose.h>

turtlesim::Pose g_pose;
std::vector<turtlesim::Circle> cerchi;

void poseCallback(const turtlesim::Pose& pose){
	g_pose = pose;
	//printf("POSECALL BACK: x=%f, y=%f, theta=%f\n", g_pose.x, g_pose.y, g_pose.theta);	//Stampa di prova
}

int updateCerchi(){
	ros::NodeHandle nget; 
	ros::ServiceClient clientget = nget.serviceClient<turtlesim::GetCircles>("getCircle");
	turtlesim::GetCircles srvget;
	if (clientget.call(srvget)){
		// ROS_INFO("Call to get_circle: Done");
			/* Causa eccessive stampe, dato che viene costantemente chiamata
			* per mantenere il delete client sempre aggiornato, quindi la commento
			*/
		cerchi=srvget.response.circles;
		return 1;
	}else{
		ROS_ERROR("Failed to call service get_circle: client close");
		return -1;
	}
}

int CallServer(){
	if(updateCerchi()==-1){
		return -1; //Nel caso non possa aggiornare i cerchi qualcosa non va nel server e chiudo tutto 
	}
	int trovato, indexToRemove;
	for(indexToRemove=0; indexToRemove<cerchi.size(); indexToRemove++){
		float posx = cerchi[indexToRemove].x;
		float posy = cerchi[indexToRemove].y;
		//ROS_INFO("posx:%f	posy:%f", posx, posy);	//Stampa di prova
		//ROS_INFO("gp_x:%f	gp_y:%f", g_pose.x, g_pose.y);	//Stampa di prova
		if((g_pose.y<posy+0.5 && g_pose.y>posy-0.5) && (g_pose.x<posx+0.5 && g_pose.x>posx-0.5)){
			trovato=1;
			break;
		}
	}
	if(trovato==1){
		ros::NodeHandle nh;
		ros::ServiceClient client = nh.serviceClient<turtlesim::RemoveCircle>("deleteCircle");
		turtlesim::RemoveCircle srv;
		srv.request.id = cerchi[indexToRemove].id;
		if (client.call(srv)){
			ROS_INFO("Call to remove_circle with id %d: Done", cerchi[indexToRemove].id);
		}else{
			ROS_ERROR("Failed to call service remove_circle");
		}
	}
	return cerchi.size();
}


int main(int argc, char** argv){
	ros::init(argc, argv, "tutorial_delete_circle");
	ros::NodeHandle n;
	
	//Mi sottoscrivo al topic per la posizione della tartaruga dell'utente
	ros::Subscriber pose_sub = n.subscribe("turtle1/pose", 1, poseCallback);
	ROS_INFO("Client delete_Circle avviato: rimarra' in esecuzione fintanto che ci sono tartarughe in campo da rimuovere (o in caso di errori)");

	int ritorno=0;
	do{
		ros::spinOnce();
		ritorno=CallServer();
	}while(ritorno>0);
	if(ritorno==0){
		ROS_INFO("Non ci sono tartarughe da eliminare: Termino il client delete");
	}else if(ritorno==-1){
		ROS_INFO("Errore nella get dei cerchi: Termino il client");

	}
  return 0;

}

