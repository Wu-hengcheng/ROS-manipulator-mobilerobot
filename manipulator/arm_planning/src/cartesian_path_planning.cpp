//该节点实现机械臂直线插补运动，通过IterativeParabolicTimeParameterization进行重规划
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h> //topp算法


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

  // 读取机器人起始位置
moveit::core::RobotStatePtr current_state(group.getCurrentState());
 const robot_state::JointModelGroup* joint_model_group =  group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

std::vector<double > joint_group_position;
current_state -> copyJointGroupPositions(joint_model_group, joint_group_position);

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

 //获取终端link的名称
    std::string end_effector_link = group.getEndEffectorLink();

    // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //	
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Planning frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link: %s", group.getEndEffectorLink().c_str());

  //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    group.setPoseReferenceFrame(reference_frame);

//设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.setGoalPositionTolerance(0.005);
    group.setGoalOrientationTolerance(0.01);
 
    //设置允许的最大速度和加速度
    group.setMaxAccelerationScalingFactor(0.5);
    group.setMaxVelocityScalingFactor(0.5);

  // Cartesian Paths 笛卡尔路径规划
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations

   // 获取当前位姿数据为机械臂运动的起始位姿
    geometry_msgs::Pose start_pose = group.getCurrentPose(end_effector_link).pose;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose);

  geometry_msgs::Pose target_pose = start_pose;

  // 圆弧轨迹规划方法
  double centerA = target_pose.position.x;
  double centerB = target_pose.position.z;
  double radius = 0.2;
 
  for (double th = 0; th < 1.3963; th = th + 0.01)
  {
   
  target_pose.position.x = centerA - radius * sin(th);
   target_pose.position.z = centerB + radius * cos(th) - radius;
   waypoints.push_back(target_pose);
  }
  
  target_pose.position.z -= 0.2;
  target_pose.position.x -= 0.1;
  waypoints.push_back(target_pose); 


  target_pose.position.z -= 0.5;
  target_pose.position.x -= 0.2;
  waypoints.push_back(target_pose);

  target_pose.position.z -= 0.5;
  target_pose.position.x -= 0.6;
  waypoints.push_back(target_pose);

  target_pose.position.z -= 0.3;
  target_pose.position.x -= 0.2;
  waypoints.push_back(target_pose);

  target_pose.position.x -= 0.122;
  target_pose.position.z -= 0.11;
  target_pose.orientation.w = 1.0;
  waypoints.push_back(target_pose);
  //target_pose.position.y -= 0.2;
  //waypoints.push_back(target_pose);  // right

  //target_pose.position.z -= 0.4;
  //target_pose.position.y += 0.2;
  //target_pose.position.x -= 0.7;
  //waypoints.push_back(target_pose);  // up and left

 moveit_msgs::RobotTrajectory trajectory_2;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = 0.0;
  int  maxtries = 500;
  int  attempts = 0;
  while (fraction < 0.98 &&  attempts < maxtries)
  {
      fraction =group.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory_2);
      attempts++;

      if (attempts % 50 ==0)
      ROS_INFO("Still trying after %d attempts...",attempts);

  }

  if (fraction >= 0.98)
  {
//重规划
     moveit::planning_interface::MoveGroupInterface :: Plan  joinedplan;
     robot_trajectory::RobotTrajectory rt(group.getCurrentState() ->getRobotModel(), "arm");
     rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_2);
     trajectory_processing::IterativeParabolicTimeParameterization  iptp;
     bool Success = iptp.computeTimeStamps(rt);

    ROS_INFO("Computed time stamp %s", Success ? "SUCCEDED" : "FAILED");

     rt.getRobotTrajectoryMsg(trajectory_2);
     joinedplan.trajectory_ = trajectory_2;

     group.execute(joinedplan);

  }

   else
  {
      ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction ,maxtries);
  }
	// END_TUTORIAL

  ros::shutdown();  

 return 0;
}

