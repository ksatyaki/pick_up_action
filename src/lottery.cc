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

void* spin_t(void * dummy)
{
	while(ros::ok())
		ros::spin();

	return NULL;
}

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

	pthread_create(&lottery::id, NULL, spin_t, NULL);

	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = &handler;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTSTP, &sa, NULL);

	geometry_msgs::PoseStamped ready, steady, go[4], test, home, handover;

	VALUES_TO_POSE(ready, 0.176, 0.146, 0.687, 0.759, 0.415, 0.171, -0.472);

	VALUES_TO_POSE(test, 0.454, -0.035, 0.487, -0.003, 0.987, 0.090, 0.130);

	VALUES_TO_POSE(steady, 0.450, -0.046, 0.167, -0.131, 0.990, 0.022, 0.054);

	VALUES_TO_POSE(go[0], 0.497, 0.064, 0.163, -0.191, 0.980, 0.025, 0.052);

	VALUES_TO_POSE(go[1], 0.518, -0.180, 0.163, -0.001, 0.998, 0.015, 0.056);

	VALUES_TO_POSE(go[3], 0.417, -0.028, 0.163, -0.159, 0.986, 0.023, 0.053);

	VALUES_TO_POSE(go[2], 0.384, -0.156, 0.163, -0.159, 0.986, 0.023, 0.053);

	VALUES_TO_POSE(handover, 0.508, 0.089, 0.528, 0.098, 0.788, 0.566, 0.221);

	doro_manipulation::DoroManipulation _dmt;
	
	_dmt.group.startStateMonitor();
	sleep(1);
	ready = _dmt.group.getCurrentPose();

	ROS_INFO("The pose now: (%f,%f,%f) ; (%f,%f,%f,%f)",
			ready.pose.position.x,
			ready.pose.position.y,
			ready.pose.position.z,
			ready.pose.orientation.x,
			ready.pose.orientation.y,
			ready.pose.orientation.z,
			ready.pose.orientation.w);
	ROS_INFO("FRAME FOR CURRENT POSE:= %s",ready.header.frame_id.c_str());
	
	ready.pose.position.x += 0.1;

	sleep(1);

	ROS_INFO("INITIALIZED!");

	std::cout<<_dmt.group.getEndEffectorLink();

	_dmt.planAndMove(ready);


	sleep(15);

	/*
	lottery::done = false;
	int i = 0;

	while(!lottery::done)
	{
		_dmt.openHand();
		sleep(1);
		_dmt.planAndMove(ready);
		_dmt.closeHand(0.5);
		_dmt.planAndMove(steady);


		while(!lottery::done)
		{
			if(i > 3)
				i = 0;
			_dmt.planAndMove(go[i]);

			if(lottery::done)
			{
				geometry_msgs::PoseStamped pregrasp = go[i];
				pregrasp.pose.position.z += 0.05;
				_dmt.planAndMove(pregrasp);
				_dmt.openHand();
				_dmt.planAndMove(go[i]);
			}
			i++;
		}

		_dmt.closeHand();

		_dmt.planAndMove(steady);
		ROS_INFO("Press Ctrl-Z again to redo. Or wait for pick-up...");
		sleep(5);
	}

	_dmt.planAndMove(test);
	handover.pose.position.z = test.pose.position.z;
	 */
	//_dmt.planAndMove(handover);

	pthread_cancel(lottery::id);
	return 1;
}


