/*
 * pick_up_client_test.cpp
 *
 *  Created on: Apr 21, 2014
 *      Author: ace
 */

#include <pick_up_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pickup_action/PickUpAction.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>

#define _times_ 36

int main(int argn, char* args[])
{
	ros::init(argn, args, "cliend");

/*
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _ac ("move_base", true);

	move_base_msgs::MoveBaseGoal pose_goal;

	pose_goal.target_pose.header.frame_id = "map";

	/
	  Translation: [0.349, -0.784, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.506, 0.863]
            in RPY [0.000, -0.000, 1.061]

            - Translation: [-0.062, -0.404, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.232, 0.973]
            in RPY [0.000, -0.000, 0.468]
 Entrance co-ords =>
 Position(-0.252, -2.436, 0.000), Orientation(0.000, 0.000, -0.746, 0.665)
 Near Balcony =>
 Translation: [-4.453, -0.785, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 1.000, 0.008]

Balcony Cognidrive map:=>
Translation: [0.182, -0.122, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.999, -0.035]
            in RPY [0.000, -0.000, -3.072]

/

	pose_goal.target_pose.header.stamp = ros::Time::now();
	pose_goal.target_pose.pose.position.x = 0.182;
	pose_goal.target_pose.pose.position.y = -0.122;
	pose_goal.target_pose.pose.orientation.z = 0.999;
	pose_goal.target_pose.pose.orientation.w = -0.035;

	ROS_INFO("WAITING FOR SERVER!");
	_ac.waitForServer();
	ROS_INFO("FOUND THE SERVER!");

	_ac.sendGoal(pose_goal);

	_ac.waitForResult();
*/

	pickup_action::PickUpGoal _goal;
	_goal.signature.data = "pick";

	actionlib::SimpleActionClient<pickup_action::PickUpAction> _sac ("pick_up", true);

	ROS_INFO("WAITING FOR SERVER!");
	_sac.waitForServer();
	ROS_INFO("FOUND THE SERVER!");

	_sac.sendGoal(_goal);

	_sac.waitForResult();

	ros::NodeHandle nh;
	ros::Publisher cmdvel_p = nh.advertise <geometry_msgs::Twist> ("/cmd_vel", 1);

	geometry_msgs::Twist twist;
	twist.linear.x = 0.0;
	twist.angular.z = -0.2;

	ros::Rate rate_(4);

	int times = _times_;
	while(times--)
	{
			cmdvel_p.publish(twist);
			rate_.sleep();
	}

	/*
	- Translation: [-1.457, -0.339, 0.000]
	- Rotation: in Quaternion [0.000, 0.000, 0.729, 0.684]
	            in RPY [0.000, -0.000, 1.634]

	            - Translation: [2.519, 0.085, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.943, -0.334]
            in RPY [0.000, -0.000, -2.461]

	            /

	pose_goal.target_pose.header.stamp = ros::Time::now();
		pose_goal.target_pose.pose.position.x = 2.519;
		pose_goal.target_pose.pose.position.y = 0.085;
		pose_goal.target_pose.pose.orientation.z = 1.00;
		pose_goal.target_pose.pose.orientation.w = 0.00;

		ROS_INFO("WAITING FOR SERVER!");
		_ac.waitForServer();
		ROS_INFO("FOUND THE SERVER!");

		_ac.sendGoal(pose_goal);

		_ac.waitForResult(); 

		*/

	_goal.signature.data = "release";

	_sac.sendGoal(_goal);

	ROS_INFO("Release requested.");
	_sac.waitForResult();

	sleep(2);

	twist.linear.x = 0.0;
	twist.angular.z = 0.2;

	times = _times_;
	while(times--)
	{
		cmdvel_p.publish(twist);
		rate_.sleep();
	}


	return 0;

}





