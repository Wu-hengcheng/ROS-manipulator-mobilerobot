用于gazebo和moveit联合仿真的配置文件。
1.moveit端配置
controllers.yaml:用于与gazebo中来自moveit的轨迹控制器进行通信。
manipulator_description_moveit_controller_manager.launch:启动轨迹控制器，加载controllers.yaml中定义的关节控制器
moveit_planning_execution.launch：与RVIZ一起启动moveit节点

2.gazebo端配置
trajectory_control.yaml:包含需要和gazebo一起加载的GazeboROS控制器列表
tarjectory_controller.launch:加载trajectory_control.yaml参数，

gazebo_joint_states.yaml:关节状态控制器参数
gazebo_states.launch:将关节控制器的配置参数加载到参数服务器中 ,运行robot_state_publisher节点，发布tf

bringup_moveit.launch:同时启动gazebo，轨迹控制器和moveit的接口

3.修改文件

   <1> 改写 joint_state_publisher 节点启动参数

　　　　<rosparam param="/source_list">[/move_group/fake_controller_joint_states]</rosparam>  改为  <rosparam param="/source_list">[/joint_states]</rosparam>；

　　<2> 改写 move_group.launch 文件的配置参数

　　　　<arg name="fake_execution" value="true"/>  改为  <arg name="fake_execution" value="false"/>
    <3>修改urdf文件参数
   添加transmission标签，hardwareInterface的参数应该与控制器一致。添加gazebo的ros_control插件。

4.安装软件包

sudo apt-get install ros-melodic-control*
sudo apt-get install ros-melodic-gazebo-ros*
sudo apt-get install ros-melodic-joint-trajectory*


5.仿真框架
/home/wuhc/下载/moveit和gazebo仿真框架.png
/home/wuhc/下载/moveit机器人控制框架.png
/home/wuhc/下载/gazebo+ros+ros_control控制框架.png


6.参考网址：
https://www.jianshu.com/p/2db9c75e150c
https://zhuanlan.zhihu.com/p/63229276
https://www.guyuehome.com/890
https://www.guyuehome.com/2839
github搜索seven_dof_arm



