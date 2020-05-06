/* ROS action server */
#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>
 
/* 三次样条插补 */
#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include "cubicSpline.h"
 
using namespace std;
 
/* action 服务端声明 */
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
 
/* 初始化输入输出速度加速度 */
double acc = 0, vel = 0;
double x_out = 0, y_out = 0;
 
/* 三次样条无参构造 */
cubicSpline::cubicSpline()
{
}
/* 析构 */
cubicSpline::~cubicSpline()
{
    releaseMem();
}
/* 初始化参数 */
void cubicSpline::initParam()
{
    x_sample_ = y_sample_ = M_ = NULL;
    sample_count_ = 0;
    bound1_ = bound2_ = 0;
}
/* 释放参数 */
void cubicSpline::releaseMem()
{
    delete x_sample_;
    delete y_sample_;
    delete M_;
 
    initParam();
}
/* 加载关节位置数组等信息 */
bool cubicSpline::loadData(double *x_data, double *y_data, int count, double bound1, double bound2, BoundType type)
{
    if ((NULL == x_data) || (NULL == y_data) || (count < 3) || (type > BoundType_Second_Derivative) || (type < BoundType_First_Derivative))
    {
        return false;
    }
 
    initParam();
 
    x_sample_ = new double[count];
    y_sample_ = new double[count];
    M_        = new double[count];
    sample_count_ = count;
 
    memcpy(x_sample_, x_data, sample_count_*sizeof(double));
    memcpy(y_sample_, y_data, sample_count_*sizeof(double));
 
    bound1_ = bound1;
    bound2_ = bound2;
 
    return spline(type);
}
/* 计算样条插值 */
bool cubicSpline::spline(BoundType type)
{
    if ((type < BoundType_First_Derivative) || (type > BoundType_Second_Derivative))
    {
        return false;
    }
 
    //  追赶法解方程求二阶偏导数
    double f1=bound1_, f2=bound2_;
 
    double *a=new double[sample_count_];                //  a:稀疏矩阵最下边一串数
    double *b=new double[sample_count_];                //  b:稀疏矩阵最中间一串数
    double *c=new double[sample_count_];                //  c:稀疏矩阵最上边一串数
    double *d=new double[sample_count_];
 
    double *f=new double[sample_count_];
 
    double *bt=new double[sample_count_];
    double *gm=new double[sample_count_];
 
    double *h=new double[sample_count_];
 
    for(int i=0;i<sample_count_;i++)
        b[i]=2;                                //  中间一串数为2
    for(int i=0;i<sample_count_-1;i++)
        h[i]=x_sample_[i+1]-x_sample_[i];      // 各段步长
    for(int i=1;i<sample_count_-1;i++)
        a[i]=h[i-1]/(h[i-1]+h[i]);
    a[sample_count_-1]=1;
 
    c[0]=1;
    for(int i=1;i<sample_count_-1;i++)
        c[i]=h[i]/(h[i-1]+h[i]);
 
    for(int i=0;i<sample_count_-1;i++)
        f[i]=(y_sample_[i+1]-y_sample_[i])/(x_sample_[i+1]-x_sample_[i]);
 
    for(int i=1;i<sample_count_-1;i++)
        d[i]=6*(f[i]-f[i-1])/(h[i-1]+h[i]);
 
    //  追赶法求解方程
    if(BoundType_First_Derivative == type)
    {
        d[0]=6*(f[0]-f1)/h[0];
        d[sample_count_-1]=6*(f2-f[sample_count_-2])/h[sample_count_-2];
 
        bt[0]=c[0]/b[0];
        for(int i=1;i<sample_count_-1;i++)
            bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);
 
        gm[0]=d[0]/b[0];
        for(int i=1;i<=sample_count_-1;i++)
            gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);
 
        M_[sample_count_-1]=gm[sample_count_-1];
        for(int i=sample_count_-2;i>=0;i--)
            M_[i]=gm[i]-bt[i]*M_[i+1];
    }
    else if(BoundType_Second_Derivative == type)
    {
        d[1]=d[1]-a[1]*f1;
        d[sample_count_-2]=d[sample_count_-2]-c[sample_count_-2]*f2;
 
        bt[1]=c[1]/b[1];
        for(int i=2;i<sample_count_-2;i++)
            bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);
 
        gm[1]=d[1]/b[1];
        for(int i=2;i<=sample_count_-2;i++)
            gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);
 
        M_[sample_count_-2]=gm[sample_count_-2];
        for(int i=sample_count_-3;i>=1;i--)
            M_[i]=gm[i]-bt[i]*M_[i+1];
 
        M_[0]=f1;
        M_[sample_count_-1]=f2;
    }
    else
        return false;
 
    delete a;
    delete b;
    delete c;
    delete d;
    delete gm;
    delete bt;
    delete f;
    delete h;
 
    return true;
}
/* 得到速度和加速度数组 */
bool cubicSpline::getYbyX(double &x_in, double &y_out)
{
    int klo,khi,k;
    klo=0;
    khi=sample_count_-1;
    double hh,bb,aa;
 
    //  二分法查找x所在区间段
    while(khi-klo>1)
    {
        k=(khi+klo)>>1;
        if(x_sample_[k]>x_in)
            khi=k;
        else
            klo=k;
    }
    hh=x_sample_[khi]-x_sample_[klo];
 
    aa=(x_sample_[khi]-x_in)/hh;
    bb=(x_in-x_sample_[klo])/hh;
 
    y_out=aa*y_sample_[klo]+bb*y_sample_[khi]+((aa*aa*aa-aa)*M_[klo]+(bb*bb*bb-bb)*M_[khi])*hh*hh/6.0;
 
    //////test
    acc = (M_[klo]*(x_sample_[khi]-x_in) + M_[khi]*(x_in - x_sample_[klo])) / hh;
    vel = M_[khi]*(x_in - x_sample_[klo]) * (x_in - x_sample_[klo]) / (2 * hh)
          - M_[klo]*(x_sample_[khi]-x_in) * (x_sample_[khi]-x_in) / (2 * hh)
          + (y_sample_[khi] - y_sample_[klo])/hh
          - hh*(M_[khi] - M_[klo])/6;
    printf("[---位置、速度、加速度---]");
    printf("%0.9f, %0.9f, %0.9f\n",y_out, vel, acc);
    //////test end
 
    return true;
}
 
 
/* 收到action的goal后调用的回调函数 */
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {
 
    /* move_group规划的路径包含的路点个数 */
    int point_num = goal->trajectory.points.size();
    ROS_INFO("%d",point_num);
 
    /* 各个关节位置 */
    double joint_1p[point_num];
    double joint_2p[point_num];
    double joint_3p[point_num];
    double joint_4p[point_num];
    double joint_5p[point_num];
    double joint_6p[point_num];

 
    /* 各个关节速度 */
    double joint_1v[point_num];
    double joint_2v[point_num];
    double joint_3v[point_num];
    double joint_4v[point_num];
    double joint_5v[point_num];
    double joint_6v[point_num];

    /* 各个关节加速度 */
    double joint_1a[point_num];
    double joint_2a[point_num];
    double joint_3a[point_num];
    double joint_4a[point_num];
    double joint_5a[point_num];
    double joint_6a[point_num];

    /* 时间数组 */
    double time_from_start[point_num];
 
    for (int i = 0; i < point_num; i++) {
        joint_1p[i] = goal->trajectory.points[i].positions[0];
        joint_2p[i] = goal->trajectory.points[i].positions[1];
        joint_3p[i] = goal->trajectory.points[i].positions[2];
        joint_4p[i] = goal->trajectory.points[i].positions[3];
        joint_5p[i] = goal->trajectory.points[i].positions[4];
        joint_6p[i] = goal->trajectory.points[i].positions[5]; 

        joint_1v[i] = goal->trajectory.points[i].velocities[0];
        joint_2v[i] = goal->trajectory.points[i].velocities[1];
        joint_3v[i] = goal->trajectory.points[i].velocities[2];
        joint_4v[i] = goal->trajectory.points[i].velocities[3];
        joint_5v[i] = goal->trajectory.points[i].velocities[4];
        joint_6v[i] = goal->trajectory.points[i].velocities[5];

        joint_1a[i] = goal->trajectory.points[i].accelerations[0];
        joint_2a[i] = goal->trajectory.points[i].accelerations[1];
        joint_3a[i] = goal->trajectory.points[i].accelerations[2];
        joint_4a[i] = goal->trajectory.points[i].accelerations[3];
        joint_5a[i] = goal->trajectory.points[i].accelerations[4];
        joint_6a[i] = goal->trajectory.points[i].accelerations[5];

 
        time_from_start[i] = goal->trajectory.points[i].time_from_start.toSec();
    }
 
    FILE *f;
    f=fopen("/home/wuhc/catkin_ws/src/arm_planning/src/joint_state_data.txt","w");
    for(int j=0;j<point_num;j++)
    {
        fprintf(f,"%f,",time_from_start[j]);//1
    }
    fprintf(f,"\n");
    for(int j=0;j<point_num;j++)
    {
        fprintf(f,"%f,",joint_1p[j]);//2
    }
    fprintf(f,"\n");
    for(int j=0;j<point_num;j++)
    {
        fprintf(f,"%f,",joint_1v[j]);//3
    }
    fprintf(f,"\n");
    for(int j=0;j<point_num;j++)
    {
        fprintf(f,"%f,",joint_1a[j]);//4
    }
    fprintf(f,"\n");
    fclose(f);
 
 
    // 实例化样条
    cubicSpline spline;
 
    // lumbar test
    spline.loadData(time_from_start, joint_1p, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    double rate = (time_from_start[point_num-1] - time_from_start[0])/(point_num*4);
    double p_joint_1[point_num*4];
    double v_joint_1[point_num*4];
    double a_joint_1[point_num*4];
    double time_from_start_[point_num*4];
    for (int k = 0; k <= point_num*4 ; k++) {
        spline.getYbyX(x_out, y_out);
        time_from_start_[k] = x_out;
        p_joint_1[k] = y_out;
        v_joint_1[k] = vel;
        a_joint_1[k] = acc;
        x_out += rate;
    }
/*
    // lumbar
    spline.loadData(time_from_start, p_lumbar, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    for (int k = 0; k < point_num ; k++) {
        x_out = time_from_start[k];
        spline.getYbyX(x_out, y_out);
        v_lumbar[k] = vel;
        a_lumbar[k] = acc;
    }
    // big_arm
    spline.loadData(time_from_start, p_big_arm, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    for (int k = 0; k <point_num ; k++) {
        x_out = time_from_start[k];
        spline.getYbyX(x_out, y_out);
        v_big_arm[k] = vel;
        a_big_arm[k] = acc;
    }
    // small_arm
    spline.loadData(time_from_start, p_small_arm, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    for (int k = 0; k <point_num ; k++) {
        x_out = time_from_start[k];
        spline.getYbyX(x_out, y_out);
        v_small_arm[k] = vel;
        a_small_arm[k] = acc;
    }
    // wrist
    spline.loadData(time_from_start, p_wrist, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    for (int k = 0; k <point_num ; k++) {
        x_out = time_from_start[k];
        spline.getYbyX(x_out, y_out);
        v_wrist[k] = vel;
        a_wrist[k] = acc;
    }
    // hand
    spline.loadData(time_from_start, p_hand, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    for (int k = 0; k <point_num ; k++) {
        x_out = time_from_start[k];
        spline.getYbyX(x_out, y_out);
        v_hand[k] = vel;
        a_hand[k] = acc;
    }
*/
    f=fopen("/home/wuhc/catkin_ws/src/arm_planning/src/joint_state_data","a");
    fprintf(f,"\n");
    for(int j=0;j<=point_num*4;j++)
    {
        fprintf(f,"%f,",time_from_start_[j]);//6
    }
    fprintf(f,"\n");
    for(int j=0;j<=point_num*4;j++)
    {
        fprintf(f,"%f,",p_joint_1[j]);//7
    }
    fprintf(f,"\n");
    for(int j=0;j<=point_num*4;j++)
    {
        fprintf(f,"%f,",v_joint_1[j]);//8
    }
    fprintf(f,"\n");
    for(int j=0;j<=point_num*4;j++)
    {
        fprintf(f,"%f,",a_joint_1[j]);//9
    }
    fprintf(f,"\n");
    fclose(f);
 
 
    //control_msgs::FollowJointTrajectoryFeedback feedback;
    //feedback = NULL;
    //as->publishFeedback(feedback);
    ROS_INFO("Recieve action successful, Now We get all joints P,V,A,T!");
    as->setSucceeded();
}
 
/* 主函数主要用于动作订阅和套接字通信 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "manipulator_server");
	ros::NodeHandle nh;
	// 定义一个服务器
	Server server(nh, "/manipulator_description/arm_controller/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
	// 服务器开始运行
	server.start();
	ros::spin();
 
}

