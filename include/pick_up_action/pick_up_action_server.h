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
#include <pick_up_action/PickUpAction.h>

#include <doro_msgs/GraspPoses.h>

#include <doro_manipulation/grasp_pose_generator.h>
#include <doro_manipulation/doro_manipulation.h>

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

	/**
	 * The function that preempts the current goal.
	 * Note: This is not used currently.
	 */
	void preemptGoal();

	/**
	 * The Grasp Pose Generation Object.
	 * This we create only after we accept the goal.
	 */
	doro_manipulation::GraspPoseGenerator *G_P_G;

	/**
	 * The DoroMoveit Object.
	 * This is created and stays on all the time. Used only when a grasp pose is available.
	 */
	doro_manipulation::DoroManipulation *D_M_T;

	/*
	 * Thread for spinning from within the callback.
	 */
	//static void* spinThread(void* dummy);

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
