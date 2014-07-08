/*
 * pick_up_action_server.cpp
 *
 *  Created on: Apr 21, 2014
 *      Author: ace
 */

#include <pick_up_action/pick_up_action_server.h>

namespace pick_up_action
{

PickUpActionServer::PickUpActionServer() : server_(nh_, "pick_up", boost::bind(&PickUpActionServer::processGoal, this, _1), false) // , a_sync_(2)
{
	D_M_T = new doro_manipulation::DoroManipulation;
	ROS_INFO("Moveit Initialized. Starting server.");

	//server_.registerGoalCallback();
	//server_.registerPreemptCallback(boost::bind(&PickUpActionServer::preemptGoal, this));

	server_.start();
	ROS_INFO("Server Started!");
}

PickUpActionServer::~PickUpActionServer()
{
	if(D_M_T) delete D_M_T;
	if(G_P_G) delete G_P_G;
}

void PickUpActionServer::preemptGoal()
{
	server_.setPreempted();
	ROS_INFO("GOAL PREEMPTED");
}

void PickUpActionServer::processGoal(const pick_up_action::PickUpGoalConstPtr& goal)
{
	 //= server_.acceptNewGoal();

	//pthread_t spin_thread_id;
	//pthread_create(&spin_thread_id, NULL, &PickUpActionServer::spinThread, (void *) this);
	
	// Createt the GraspPoseGenerator Object. This will also start the extraction.

	ROS_INFO("Parameter is: %s",goal->signature.data.c_str());
	
	D_M_T->resetValues();

	geometry_msgs::PoseStamped home_pose;
	HOME_POSE(home_pose);

	if(goal->signature.data.compare("pick") == 0 ||
			goal->signature.data.compare("Pick") == 0)
	{

		G_P_G = new doro_manipulation::GraspPoseGenerator;
		ROS_INFO("STARTING CYLINDER EXTRACTION!");
		sleep(5);
		ROS_INFO("Listened for 5 seconds!");

		D_M_T->addTableAsObstacle();
		D_M_T->addTargetAsObstacle();

		// Perform actual work
		doro_msgs::GraspPoses poses = D_M_T->getCylinderPose();

		while((poses.pregrasp_poses[0].pose.position.x == poses.pregrasp_poses[0].pose.position.y) && (poses.pregrasp_poses[0].pose.position.x == poses.pregrasp_poses[0].pose.position.z)
				&& (poses.pregrasp_poses[0].pose.position.x == 0.0) && (poses.pregrasp_poses[0].header.seq < 5) )
		{
			ROS_INFO("WE ARE WAITING FOR CYLINDER POSE!");
			poses = D_M_T->getCylinderPose();
			sleep(1);
		}
		//D_M_T->adjustDoro();
		//poses = D_M_T->getCylinderPose();

		std::vector<geometry_msgs::PoseStamped>::iterator gp, pgp;

		gp = poses.grasp_poses.begin();
		pgp = poses.pregrasp_poses.begin();

		bool moved = false;

		while(gp!= poses.grasp_poses.end() && pgp != poses.pregrasp_poses.end())
		{
			// Move near
			D_M_T->openHand();
			
			if(!D_M_T->plan(*gp))
			{
				pgp++;
				gp++;
				continue;
			}
				
			moved = D_M_T->planAndMove(*pgp);
			if(moved)
			{
				// Move to the spot
				D_M_T->openHand();
				moved = D_M_T->planAndMove(*gp);
				
				if(!moved)
				{
					geometry_msgs::PoseStamped adj = *gp;
					adj.pose.position.x -= 0.02;
					adj.pose.position.y -= 0.02;
					moved = D_M_T->planAndMove(adj);
				}
				
				if(moved)	
				D_M_T->attachTarget();

				D_M_T->closeHand();
				moved = D_M_T->planAndMove(home_pose);

				if(moved)
				{
					result_.result.data = "SUCCESS";
					server_.setSucceeded(result_);
					break;
				}
			}

			pgp++;
			gp++;

		}

		if(!moved)
		{
			server_.setAborted();
		}

		delete G_P_G;
		//pthread_cancel(spin_thread_id);
	}
	
	else if(goal->signature.data.compare("release") == 0 ||
			goal->signature.data.compare("Release") == 0 )
	{
		geometry_msgs::PoseStamped hand_over;
		HANDOVER_POSE(hand_over);
		D_M_T->planAndMove(hand_over);
		sleep(5);
		D_M_T->openHand();
		D_M_T->detachTarget();

		bool moved = D_M_T->planAndMove(home_pose);

		if(moved)
		{
			result_.result.data = "SUCCESS";
			server_.setSucceeded(result_);
		}
		if(!moved)
		{
			server_.setAborted();
		}
	}

	else
	{
		ROS_INFO("Unrecognized signature.");
		server_.setAborted();
	}
	D_M_T->removeTable();


}
}

/* void* PickUpActionServer::spinThread(void *dummy)
{
	((PickUpActionServer*) dummy)->a_sync_.start();
}
*/

int main(int argn, char* args[])
{
	ros::init(argn, args, "pick_up_action_server");
	pick_up_action::PickUpActionServer P_U_A_S;

	ros::spin();
}
