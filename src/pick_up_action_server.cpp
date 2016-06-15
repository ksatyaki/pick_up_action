/*
 * pick_up_action_server.cpp
 *
 *  Created on: Apr 21, 2014
 *      Author: ace
 */

#include <pick_up_action/pick_up_action_server.h>

namespace pick_up_action
{

PickUpActionServer::PickUpActionServer() :
		server_ (nh_, "pick_up", boost::bind(&PickUpActionServer::processGoal, this, _1), false),
		pam_client_ ("plan_and_move_arm", true)
{
	gpg_client_ = nh_.serviceClient <jaco_manipulation::GenerateGraspPoses> ("generate_grasp_poses", false);

	server_.start();
	ROS_INFO("Server Started!");
}

PickUpActionServer::~PickUpActionServer()
{
}

void PickUpActionServer::processGoal(const pick_up_action::PickUpGoalConstPtr& goal)
{
	ROS_INFO("Picking up object.");
	
	jaco_manipulation::GenerateGraspPoses gpg_message;

	gpg_message.request.object_location = goal->object_location;

	gpg_message.request.object_location.header.stamp = ros::Time::now();

	if(gpg_client_.call(gpg_message))
	{
		// The main routine.
		ROS_INFO("Generated Grasp Poses... SUCCESS");
		ROS_INFO("Proceeding to grasp object...");

		jaco_manipulation::PlanAndMoveArmGoal pam_goal;
		pam_goal.goal_type = "pose";

		bool grasp_success = false;

		for(int round = 0; round < gpg_message.response.grasp_poses.size(); round++)
		{
			pam_goal.target_pose = gpg_message.response.pregrasp_poses[round];
			ROS_INFO("TARGET POSE IN %s FRAME...", pam_goal.target_pose.header.frame_id.c_str());
			pam_client_.waitForServer();
			ROS_INFO("Moving to pre-grasp pose...");
			pam_client_.sendGoal(pam_goal);
			pam_client_.waitForResult();

			if(pam_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				// Pre-grasp has succeeded now proceed for grasp
				ROS_INFO("Moving to pre-grasp succeeded.");
				pam_goal.target_pose = gpg_message.response.grasp_poses[round];
				ROS_INFO("Moving to grasp pose...");
				pam_client_.sendGoal(pam_goal);
				pam_client_.waitForResult();

				if(pam_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					pam_goal.goal_type = "close";
					ROS_INFO("Grasping object...");
					pam_client_.sendGoal(pam_goal);
					pam_client_.waitForResult();

					if(pam_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						grasp_success = true;
						break;
							}
					}

				else
					continue;

					}
			}

		if(grasp_success)
		{
			ROS_INFO("Grasp success. Retracting arm now.");
		}

		pam_goal.goal_type = "home";

		ROS_INFO("Moving to grasp pose...");
		pam_client_.sendGoal(pam_goal);
		pam_client_.waitForResult();

		if(pam_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Sucks. Can't retract arm. We should stop here. Sorry, pal.");
			server_.setAborted();
			return;
		}

		ROS_INFO("\'Smooth as a whistle and it don't cost much\'.");
		ROS_INFO("The object is in safe hands! :-)");
		ROS_INFO("DONE.");
		server_.setSucceeded();

	}
	else
	{
		ROS_INFO("Isn't it a pity that things should fail here? Grasp Pose Generator has failed.");
		ROS_INFO("Action aborted.");
		server_.setAborted();
		return;
	}

}

}

int main(int argn, char* args[])
{
	ros::init(argn, args, "pick_up_action_server");
	pick_up_action::PickUpActionServer P_U_A_S;

	ros::spin();
}
