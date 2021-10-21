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

int CallServer(){
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
			//Aggiorno l'array dei cerchi al ritorno della chiamata rimuovendo la tartaruga cancellata
			cerchi.erase(cerchi.begin()+indexToRemove);
		}else{
			ROS_ERROR("Failed to call service remove_circle");
		}
	}
	return cerchi.size();
}


int main(int argc, char** argv){
	ros::init(argc, argv, "tutorial_delete_circle");
	ros::NodeHandle n;
	ROS_INFO("Client delete_Circle avviato:");

	//Ottengo le tartarughe in campo
	ros::ServiceClient clientget = n.serviceClient<turtlesim::GetCircles>("getCircle");
	turtlesim::GetCircles srvget;
	if (clientget.call(srvget)){
		ROS_INFO("Call to get_circle: Done");
		cerchi=srvget.response.circles;
	}else{
		ROS_ERROR("Failed to call service get_circle");
		return 1;
	}
	//Ho ottenuto le tartarughe in campo e non ne vedrò di nuove
	ROS_INFO("Ho ottenuto le tartarughe nel campo e non ne vedro' di nuove fino al riavvio di questo client della delete.");
	ROS_INFO("Questo client terminera' l'esecuzione quando tutte le tartarughe ottenute all'avvio saranno rimosse.");

	//Mi sottoscrivo al topic per la posizione della tartaruga dell'utente
	ros::Subscriber pose_sub = n.subscribe("turtle1/pose", 1, poseCallback);
	int ritorno;
	do{
		ros::spinOnce();
		ritorno=CallServer();
	}while(ritorno>0);
	if(ritorno==0){
		ROS_INFO("Tartarughe da eliminare finite: Termino il client");
	}
	
  return 0;

}

