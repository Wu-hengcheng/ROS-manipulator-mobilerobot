//关节空间轨迹规划
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_planning");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up */
  sleep(2.0);
  
  // BEGIN_TUTORIAL
  // 
  // Setup
  // ^^^^^
  // 
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "arm";
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =  group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;



  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //	
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Planning frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link: %s", group.getEndEffectorLink().c_str());

// To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

// Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[1] = -0.314;  // radians
  joint_group_positions[2] = -1.5708;  // radians
 group.setJointValueTarget(joint_group_positions);



  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
group.setPlanningTime(10);
moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan); 
ROS_INFO("Visualizing plan 1 (pose goal) %s",success.val ? "":"FAILED");    

group.execute(my_plan);	



 // Sleep to give Rviz time to visualize the plan. 
  sleep(5.0);

	// END_TUTORIAL

  ros::shutdown();  

 return 0;
}
