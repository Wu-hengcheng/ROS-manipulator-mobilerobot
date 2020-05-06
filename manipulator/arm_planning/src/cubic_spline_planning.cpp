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
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h> 

#include "cubicSpline.h" 
#include <stdio.h>

#define POINTS_COUNT 10
int main(int argc, char **argv)
{
  //ros::init(argc, argv, "arm_planning");
  ros::init(argc, argv, "cubic_spline_planning");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  void scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan &plan,double scale);//函数声明
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
    group.setMaxVelocityScalingFactor(0.9);

//机械臂运动到初始位置
  //  group.setNamedTarget("initial_pose");
    
   // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    //bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
  //  if (success) 
  //  group.execute(my_plan);

   // sleep(3.0);
  // cubic spline trajectory 三次样条曲线规划
  // ^^^^^^^^^^^^^^^
  
   // 获取当前位姿数据为机械臂运动的起始位姿
  geometry_msgs::Pose start_pose = group.getCurrentPose(end_effector_link).pose;

  std::vector<geometry_msgs::Pose> rwaypoints;//路径的反向数组
  //rwaypoints.push_back(start_pose);

  geometry_msgs::Pose target_pose = start_pose;

  double x_pos = target_pose.position.x;       //-0.636502399
  double z_pos = target_pose.position.z;       //1.062740791
  ROS_INFO("current x= %0.9f, z=%0.9f",x_pos, z_pos);
 
  double x_data[POINTS_COUNT] = {x_pos-1.224, x_pos-1.1, x_pos-0.96, x_pos-0.85,x_pos-0.7,x_pos-0.6,x_pos-0.4,x_pos-0.325, x_pos-0.1, x_pos};
  double z_data[POINTS_COUNT] = {z_pos-1.51276, z_pos-1.49, z_pos-1.2,z_pos-0.982, z_pos-0.91, z_pos-0.89,z_pos-0.82,z_pos-0.754, z_pos-0.2, z_pos};

  double x_out = x_pos-1.224-0.004;
  double z_out = z_pos-1.51276;
  double rate = 0.016339869281;

  cubicSpline spline;
  spline.loadData(x_data, z_data,POINTS_COUNT, 0, 0, cubicSpline::BoundType_First_Derivative);

  
  
  for (int i=0;i<=306;i++)
  {
     x_out = x_out + 0.004;
     spline.getYbyX(x_out,z_out);
     target_pose.position.x = x_out;
     target_pose.position.z = z_out;
     rwaypoints.push_back(target_pose);
  }

 //std::vector<geometry_msgs::Pose> waypoints;

// for (int i=rwaypoints.size()-1;i>=0;i--)
 //{
//	 waypoints.push_back(rwaypoints[i]);
// }
  reverse(rwaypoints.begin(),rwaypoints.end()); //得到正向路径点


  moveit_msgs::RobotTrajectory trajectory_2;
  const double jump_threshold = 0.0;
  const double eef_step = 0.002;
  double fraction = 0.0;
  int  maxtries = 5000;
  int  attempts = 0;
  //moveit::planning_interface::MoveGroupInterface :: Plan  plan;
  while (fraction < 0.98 &&  attempts < maxtries)
  {
      fraction =group.computeCartesianPath(rwaypoints,eef_step,jump_threshold,trajectory_2);
      attempts++;

      if (attempts % 50 ==0)
      ROS_INFO("Still trying after %d attempts...",attempts);

  }


  if (fraction >= 0.98)
  {
//重规划
     moveit::planning_interface::MoveGroupInterface :: Plan  plan;
     robot_trajectory::RobotTrajectory rt(group.getCurrentState() ->getRobotModel(), "arm");
     rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_2);
     trajectory_processing::TimeOptimalTrajectoryGeneration totg;
     bool Success = totg.computeTimeStamps(rt,1.0,1.0);
     // moveit::planning_interface::MoveItErrorCode Success = group.plan(plan);
     ROS_INFO("Computed time stamp %s", Success ? "SUCCEDED" : "FAILED");

     rt.getRobotTrajectoryMsg(trajectory_2);
     plan.trajectory_ = trajectory_2;

     scale_trajectory_speed(plan,1.5);
     group.execute(plan);

  }

   else
  {
      ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction ,maxtries);
  }

	// END_TUTORIAL

  ros::shutdown();  

 return 0;
}

      //轨迹重定义，规划速度和时间
   void scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan &plan,double scale)
   {
     int n_joints = plan.trajectory_.joint_trajectory.joint_names.size();
     
     for (int i=0;i<plan.trajectory_.joint_trajectory.points.size();i++)
      {
         plan.trajectory_.joint_trajectory.points[i].time_from_start *= 1/scale;

         for (int j=0;j<n_joints;j++)
          {
           plan.trajectory_.joint_trajectory.points[i].velocities[j] *= scale;
           plan.trajectory_.joint_trajectory.points[i].accelerations[j] *= scale*scale;
          }
      }
    }
