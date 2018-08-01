/*
 * pick_up_action_server.cpp
 *
 *  Created on: Apr 21, 2014
 *      Author: ace
 */

#include <pick_up_action/pick_up_action_server.h>

namespace pick_up_action {

PickUpActionServer::PickUpActionServer()
    : server_(nh_, "pick_up",
              boost::bind(&PickUpActionServer::processGoal, this, _1), false),
      pam_client_("plan_and_move_arm", true) {
  gpg_client_ = nh_.serviceClient<jaco_manipulation::GenerateGraspPoses>(
      "generate_grasp_poses", false);

  server_.start();
  ROS_INFO("Server Started!");
}

PickUpActionServer::~PickUpActionServer() {}

void PickUpActionServer::processGoal(
    const pick_up_action::PickUpGoalConstPtr& goal) {
  jaco_manipulation::PlanAndMoveArmGoal pam_goal;
  
  if (goal->type == pick_up_action::PickUpGoal::PICK_UP) {
    ROS_INFO("Picking up object.");

    // Prepare the GPG message.
    jaco_manipulation::GenerateGraspPoses gpg_message;
    gpg_message.request.object_location = goal->object_location;
    gpg_message.request.object_location.header.stamp = ros::Time::now();

    if (gpg_client_.call(gpg_message)) {
      // The main routine.
      ROS_INFO("Generated Grasp Poses... SUCCESS");
      ROS_INFO("Proceeding to grasp object.");

      pam_goal.goal_type = "open";

      bool grasp_success = false;
      pam_client_.waitForServer();
      ROS_INFO("[PickUpActionServer]: Step 1 >>> Opening gripper.");
      pam_client_.sendGoal(pam_goal);
      pam_client_.waitForResult();

      for (int round = 0; round < gpg_message.response.grasp_poses.size();
           round++) {
        pam_goal.target_pose = gpg_message.response.pregrasp_poses[round];
        // ROS_INFO("TARGET POSE IN %s FRAME...",
        // pam_goal.target_pose.header.frame_id.c_str());
        pam_goal.goal_type = "pose";
        pam_client_.waitForServer();
        ROS_INFO(
            "[PickUpActionServer]: Step 2 >>> Moving to pre-grasp pose...");
        pam_client_.sendGoal(pam_goal);
        pam_client_.waitForResult();

        if (pam_client_.getState() ==
            actionlib::SimpleClientGoalState::SUCCEEDED) {
          // Pre-grasp has succeeded now proceed for grasp
          pam_goal.goal_type = "pose";
          pam_goal.target_pose = gpg_message.response.grasp_poses[round];
          ROS_INFO("[PickUpActionServer]: Step 3 >>> Moving to grasp pose.");
          pam_client_.sendGoal(pam_goal);
          pam_client_.waitForResult();

          if (pam_client_.getState() ==
              actionlib::SimpleClientGoalState::SUCCEEDED) {
            pam_goal.goal_type = "close";
            pam_client_.waitForServer();
            ROS_INFO("[PickUpActionServer]: Step 4 >>> Closing gripper.");
            pam_client_.sendGoal(pam_goal);
            pam_client_.waitForResult();

            if (pam_client_.getState() ==
                actionlib::SimpleClientGoalState::SUCCEEDED) {
              grasp_success = true;
              break;
            }
          }

          else
            continue;
        }
      }

      if (grasp_success) {
        // ROS_INFO("Grasp success. Retracting arm now.");
      }

      ROS_INFO("\'Smooth as a whistle and it don't cost much\'.");
      ROS_INFO("The object is in safe hands! :-)");
      ROS_INFO("DONE.");
      server_.setSucceeded();
      return;

    } else {
      ROS_INFO(
          "Isn't it a pity that things should fail here? Grasp Pose Generator "
          "has failed.");
      ROS_INFO("Action aborted.");
      server_.setAborted();
      return;
    }
  }

  // Obvious else.
  else {
    ROS_INFO("putting down object.");

    jaco_manipulation::GenerateGraspPoses gpg_message;
    gpg_message.request.object_location = goal->object_location;
    gpg_message.request.object_location.header.stamp = ros::Time::now();

    if (!gpg_client_.call(gpg_message)) {
      ROS_INFO("GPG client failed. Contact chitt... if you can find him. ;-)");
      server_.setAborted();
      return;
    }

    ROS_INFO("Proceeding to drop object.");
    pam_goal.goal_type = "pose";
    pam_goal.target_pose = gpg_message.response.grasp_poses[0];
    pam_goal.target_pose.pose.position.z += 0.1;

    pam_client_.sendGoal(pam_goal);
    ROS_INFO("Sent goal.");
    pam_client_.waitForResult();

    if (pam_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO(
          "Something failed while trying to move to the pre-putdown pose. ");
      server_.setAborted();
      return;
    }

    pam_goal.target_pose = gpg_message.response.grasp_poses[0];

    pam_client_.sendGoal(pam_goal);
    ROS_INFO("Sent goal.");
    pam_client_.waitForResult();

    if (pam_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO(
          "Something failed when moving to put down point. Dropping object "
          "anyway.");
    }

    pam_goal.goal_type = "open";
    pam_client_.sendGoal(pam_goal);
    ROS_INFO("Sent goal.");
    pam_client_.waitForResult();

    if (pam_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Something failed while trying to open gripper.");
      server_.setAborted();
      return;
    }
  }
  
  pam_goal.goal_type = "home";

  ROS_INFO("[PickUpActionServer]: Step 5 >>> Moving to Home Pose.");
  pam_client_.sendGoal(pam_goal);
  pam_client_.waitForResult();

  if (pam_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Sucks. Can't retract arm. We should stop here. Sorry, pal.");
    server_.setAborted();
    return;
  }

  server_.setSucceeded();
}
}

int main(int argn, char* args[]) {
  ros::init(argn, args, "pick_up_action_server");
  pick_up_action::PickUpActionServer P_U_A_S;

  ros::spin();
}
