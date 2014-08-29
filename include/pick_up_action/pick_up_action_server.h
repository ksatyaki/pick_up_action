/*
 * pick_up_action_server.h
 *
 *  Created on: Apr 21, 2014
 *      Author: Chittaranjan S Srinivas
 */
#ifndef PICK_UP_ACTION_SERVER_H_
#define PICK_UP_ACTION_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pick_up_action/PickUpAction.h>

#include <doro_msgs/GraspPoses.h>

#include <acquire_objects/AcquireObjects.h>
#include <doro_manipulation/GenerateGraspPoses.h>
#include <doro_manipulation/PlanAndMoveArmAction.h>

namespace pick_up_action
{
/**
 * The Class for the pickup-action-server.
 * We are using the execute-callback method. This allows the goal to be preempted.
 */
class PickUpActionServer
{
protected:

	/**
	 * A client to the "generate_grasp_poses" server.
	 */
	ros::ServiceClient gpg_client_;

	/**
	 * A client to the "acquire_objects" server.
	 */
	ros::ServiceClient ao_client_;

	/**
	 * A client to the "plan_and_move_arm" server.
	 */
	actionlib::SimpleActionClient <doro_manipulation::PlanAndMoveArmAction> pam_client_;

	/**
	 * A Nodehandle for this class.
	 * Note: A nodehandle has to be created before an action server class is declared.
	 * This is what the tutorials say.
	 */
	ros::NodeHandle nh_;

	/**
	 * A SimpleActionServer object.
	 * Our Action is the template argument.
	 */
	actionlib::SimpleActionServer<pick_up_action::PickUpAction> server_;

	/**
	 * Message for publishing the feedback.
	 */
	pick_up_action::PickUpFeedback feedback_;

	/**
	 * Message for publishing the result.
	 */
	pick_up_action::PickUpResult result_;

	/**
	 * The function which processes the goal.
	 * Note: We use the execute callback method.
	 */
	void processGoal(const pick_up_action::PickUpGoalConstPtr& goal);

public:
	/*
	 * A spinner.
	 */
	//ros::AsyncSpinner a_sync_;

	/**
	 * Constructor.
	 */
	PickUpActionServer();
	virtual ~PickUpActionServer();
};

}

#endif /* PICK_UP_ACTION_SERVER_H_ */
