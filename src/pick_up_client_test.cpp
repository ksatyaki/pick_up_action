/*
 * pick_up_client_test.cpp
 *
 *  Created on: Apr 21, 2014
 *      Author: ace
 */

#include <pick_up_action/pick_up_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pick_up_action/PickUpAction.h>

#include <iostream>
#include <geometry_msgs/Twist.h>

#define _times_ 36

int main(int argn, char* args[])
{
	ros::init(argn, args, "cliend");

	pick_up_action::PickUpGoal _goal;
	_goal.object_location.header.frame_id = "base_link";
	std::cin>>_goal.object_location.point.x;
	std::cin>>_goal.object_location.point.y;
	std::cin>>_goal.object_location.point.z;


	actionlib::SimpleActionClient <pick_up_action::PickUpAction> _sac ("pick_up", true);

	ROS_INFO("WAITING FOR SERVER!");
	_sac.waitForServer();
	ROS_INFO("FOUND THE SERVER!");

	_sac.sendGoal(_goal);

	_sac.waitForResult();
	return 0;

}





