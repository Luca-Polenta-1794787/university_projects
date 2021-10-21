#include <ros/ros.h>
//#include <chores/DoDishesAction.h> // Note: "Action" is appended
#include <hw6/AzioneAction.h> 
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<hw6::AzioneAction> Server;

// Note: "Action" is not appended to serverAzione here
void execute(const hw6::AzioneGoalConstPtr& goal, Server* as){
	 // Note: "Action" is not appended to serverAzione here
	 as->setSucceeded();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "serverAzione");
	ros::NodeHandle n;
	Server server(n, "server_azione", boost::bind(&execute, _1, &server), false);
	server.start();
	ros::spin();
	return 0;
}
