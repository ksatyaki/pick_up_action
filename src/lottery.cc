/*
 * #######
 * #READY#
 * #######
 *- Translation: [0.454, -0.035, 0.887]
 *- Rotation: in Quaternion [-0.003, 0.987, 0.090, 0.130]
 *           in RPY [2.957, 0.260, 3.124]
 *
 *
 * ########
 * #STEADY#
 * ########
 * - Translation: [0.450, -0.046, 0.667]
 * - Rotation: in Quaternion [-0.131, 0.990, 0.022, 0.054]
 *           in RPY [3.112, 0.112, -2.881]
 *
 * ######
 * #GO 1#
 * ######
 *	- Translation: [0.497, 0.064, 0.667]
 *	- Rotation: in Quaternion [-0.191, 0.980, 0.025, 0.052]
 *           in RPY [3.112, 0.112, -2.758]
 *
 * ######
 * #GO 2#
 * ######
 * - Translation: [0.518, -0.180, 0.667]
 * - Rotation: in Quaternion [-0.001, 0.998, 0.015, 0.056]
 *           in RPY [3.112, 0.112, 3.141]
 *
 *
 * ######
 * #GO 3#
 * ######
 * - Translation: [0.417, -0.028, 0.657]
 * - Rotation: in Quaternion [-0.159, 0.986, 0.023, 0.053]
 *           in RPY [3.112, 0.112, -2.824]
 *
 * ######
 * #HANDOVER#
 * ######
 *
 * - Translation: [0.508, 0.089, 1.128]
- Rotation: in Quaternion [0.098, 0.788, 0.566, 0.221]
            in RPY [1.844, 0.239, 2.712]
 */

#include <ros/ros.h>
#include <doro_manipulation/doro_manipulation.h>
#include <pthread.h>
#include <geometry_msgs/PoseStamped.h>
#include <signal.h>

#include <actionlib/client/simple_action_client.h>
#include <doro_manipulation/PlanAndMoveArmAction.h>

#include <iostream>
#include <geometry_msgs/Twist.h>

namespace lottery
{
	bool done;
	pthread_t id;
}

#define VALUES_TO_POSE(_pose, px, py, pz, qx, qy, qz, qw) { \
	_pose.pose.position.x = px;\
	_pose.pose.position.y = py;\
	_pose.pose.position.z = pz;\
	_pose.pose.orientation.x = qx;\
	_pose.pose.orientation.y = qy;\
	_pose.pose.orientation.z = qz;\
	_pose.pose.orientation.w = qw;\
	_pose.header.frame_id = "/base_link";\
}

/*
void* spin_t(void * dummy)
{
	while(ros::ok())
		ros::spin();

	return NULL;
}
*/

void handler(int signal_number)
{
	if(signal_number == SIGINT)
	{
		printf("Ctrl-C caught...\n");
		pthread_cancel(lottery::id);
		ros::shutdown();
		abort();

	}
	else
	{
		printf("Ctrl-Z caught...\n");
		lottery::done = !lottery::done;
	}
}

int main(int argn, char* args[])
{
	ros::init(argn, args, "lottery");

	ros::NodeHandle nh;
	ros::Publisher cmd_vel_pub = nh.advertise <geometry_msgs::Twist> ("/cmd_vel", 1);

	actionlib::SimpleActionClient <doro_manipulation::PlanAndMoveArmAction> sac_lient("plan_and_move_arm", true);

	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = &handler;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTSTP, &sa, NULL);

	geometry_msgs::PoseStamped trial;

	double poses[7];

	ROS_INFO("Enter: ");
	scanf("%lf %lf %lf %lf %lf %lf %lf", &poses[0], &poses[1], &poses[2], &poses[3], &poses[4], &poses[5], &poses[6]);
	VALUES_TO_POSE(trial, poses[0], poses[1], poses[2], poses[3], poses[4], poses[5], poses[6]);

	doro_manipulation::PlanAndMoveArmGoal _goal;
	_goal.goal_type = "pose";
	_goal.target_pose = trial;

	sac_lient.sendGoal(_goal);
	ROS_INFO("waits");
	sac_lient.waitForResult();

	if(sac_lient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("You win. Fatality.");
	}
	else
	{
		ROS_INFO("You suck.");
	}

	return 0;
}


