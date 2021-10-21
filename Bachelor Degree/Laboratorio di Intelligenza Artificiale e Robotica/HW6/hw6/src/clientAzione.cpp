#include <ros/ros.h>
//#include <chores/DoDishesAction.h> // Note: "Action" is appended
#include <hw6/AzioneAction.h> 
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<hw6::AzioneAction> Client;

int main(int argc, char** argv){
	ros::init(argc, argv, "clientAzione");
	Client client("client_azione", true); // true -> don't need ros::spin()
	client.waitForServer();
	hw6::AzioneGoal goal;
	//Fill in goal here
	client.sendGoal(goal);
    client.waitForResult(ros::Duration(5.0));
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		printf("Yay! Ma chi me l'ha fatto fa");
	}
	printf("Current State: %s\n", client.getState().toString().c_str());
	return 0;
}
