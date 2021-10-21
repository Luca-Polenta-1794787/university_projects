#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h> 
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	ros::init(argc, argv, "hw6client");
	ros::NodeHandle n;
	
	//Salvo la posizione iniziale
	/* La waitForMessage restituisce un puntatore ad un messaggio del topic 
	 * che ascolta, e se il puntatore è diverso da NULL lo salvo in un'altra variabile
	 * */
	boost::shared_ptr<nav_msgs::Odometry const> msg_ptr;
	nav_msgs::Odometry messaggioTopic;
	msg_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/base_pose_ground_truth", n);
	if(msg_ptr != NULL){
		messaggioTopic = *msg_ptr;
	}
	//Perché si stampa così? Guarda commenti fine file
	ROS_INFO("Provo a leggere la posizione iniziale: x=%f", messaggioTopic.pose.pose.position.x);
	ROS_INFO("OSS.: Purtroppo Rviz e stage_ros sono sfasati, quindi non coincide con quella che vorrei");
	//Quindi dopo non la uso

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	
	move_base_msgs::MoveBaseGoal goal;
	//we'll send a goal to the robot to move away from the initial pose
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = 4.363;
	goal.target_pose.pose.position.y = 0.022;
	//goal.target_pose.pose.position.z = 3.142;
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	
	//Aspetto 10 secondi
	ac.waitForResult(ros::Duration(40.0));

	//Cancello il vecchio goal (o meglio, tutti i goals)
	ac.cancelAllGoals();
	ROS_INFO("Cancellato vecchio goal");

	//Gli dico di tornare alla posizione originale
	move_base_msgs::MoveBaseGoal goal2;
	goal2.target_pose.header.frame_id = "base_link";
	goal2.target_pose.header.stamp = ros::Time::now();
	goal2.target_pose.pose.position.x = 0.01; //messaggioTopic.pose.pose.position.x;
	goal2.target_pose.pose.position.y = 0.01; //messaggioTopic.pose.pose.position.y;
	//goal.target_pose.pose.position.z = 3.142;
	ROS_INFO("Go home");
	ac.sendGoal(goal2);
	//Verifico che sia tutto okay
	ac.waitForResult();
	if(ac.getState()==actionlib::SimpleClientGoalState ::SUCCEEDED){
		ROS_INFO("Tutto okay");
	}else{
		ROS_INFO("Errore nell'esecuzione");
	}
	return 0;
}

/*
 * nav_msgs/Odometry Message
 * File: nav_msgs/Odometry.msg
	  	Header header
		string child_frame_id
		geometry_msgs/PoseWithCovariance pose
		geometry_msgs/TwistWithCovariance twist

 * Dentro PoseWithCovariance:
 * geometry_msgs/Pose Message
 * File: geometry_msgs/Pose.msg
		geometry_msgs/Point position
		geometry_msgs/Quaternion orientation
 * */
