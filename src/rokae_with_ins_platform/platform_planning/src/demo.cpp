#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
// #include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

using namespace std;


int main(int argc, char **argv)
{
    /* code */
    ros::init(argc, argv, "demo"); //初始化

    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1); //多线程
    spinner.start(); //开启新的线程
 
    moveit::planning_interface::MoveGroupInterface arm("rokae");//初始化需要使用move group控制的机械臂中的rokae group
    moveit::planning_interface::MoveGroupInterface gripper("inspire");
    // arm.setPlanningTime(20.0);
    // arm.setNumPlanningAttempts(10);
    

    //获取终端LINK的名称
    const string end_effector_link = arm.getEndEffectorLink();

    //打印终端LINK的名称
    ROS_INFO("*************EndEffectorLink is [%s]",end_effector_link.c_str());


    //设置目标位置所使用的参考坐标系
    string reference_frame ="xMateCR7_base"; 
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后允许重新规划
    arm.allowReplanning(true);

    //设置位置（单位：米）姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.01);
    arm.setGoalOrientationTolerance(0.01);

    arm.setMaxAccelerationScalingFactor(0.8); //允许的最大速度和加速度
    arm.setMaxVelocityScalingFactor(0.8);

    //控制机械臂运动到up位置
    ROS_INFO("***********will excute first step********************");
    arm.setNamedTarget("home");
    arm.move();
    ROS_INFO("***********first step finished********************");
    sleep(2);

    cout<<"手指即将闭合"<<endl;
    gripper.setNamedTarget("close");
    gripper.move();
    cout<<"手指闭合完成"<<endl;


    

    // std::vector<geometry_msgs::Pose> waypoints;

    // // 设置机器人终端的目标位置
    // geometry_msgs::Pose target_pose = arm.getCurrentPose(end_effector_link).pose;


    // /*************************************************************绘制心形代码*******************************************/
    // target_pose.position.x -= 0.3;
    // waypoints.push_back(target_pose);

    // double centerA = target_pose.position.x;
    // double centerB = target_pose.position.z;
    // double radius = 0.1;

    // //绘制圆形
    // for(double th=0.0; th<6.28; th=th+0.01)
    // {
    //     target_pose.position.x = centerA + radius * cos(th);
    //     target_pose.position.z = centerB + radius * sin(th);
    //     waypoints.push_back(target_pose);
    // }

    // //绘制心形
    // for(double th=0.0; th<6.28; th=th+0.01)
    // {
    //     target_pose.position.x = centerA + radius * (2*cos(th)-cos(2*th));
    //     target_pose.position.z = centerB + radius * (2*sin(th)-sin(2*th));
    //     waypoints.push_back(target_pose);
    // }

    // //绘制爱心时，末端移回到原点
    // target_pose.position.x = centerA-0.05; 
    // target_pose.position.z = centerB;
    // waypoints.push_back(target_pose);


    
    //             //打印vector列表里点的具体位置
    //             for (size_t i = 0; i < waypoints.size(); i++)
    //             {
    //                 /* code */
    //                 ROS_INFO("position: x-%f y-%f z-%f orientation: x-%f y-%f z-%f w-%f",
    //                                                 waypoints[i].position.x,
    //                                                 waypoints[i].position.y,
    //                                                 waypoints[i].position.z,
    //                                                 waypoints[i].orientation.x,
    //                                                 waypoints[i].orientation.y,
    //                                                 waypoints[i].orientation.z,
    //                                                 waypoints[i].orientation.w);

    //             }
    //             // 笛卡尔空间下的路径规划
    //             moveit_msgs::RobotTrajectory trajectory;
    //             const double jump_threshold = 0.0;
    //             const double eef_step = 0.01;
    //             double fraction = 0.0;
    //             int maxtries = 100;   //最大尝试规划次数
    //             int attempts = 0;     //已经尝试规划次数
            
    //             while(fraction < 1.0 && attempts < maxtries)
    //             {
    //                 fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    //                 attempts++;
                    
    //                 if(attempts % 10 == 0)
    //                     ROS_INFO("Still trying after %d attempts...", attempts);
    //             }
                
    //             if(fraction == 1)
    //             {   
    //                 ROS_INFO("Path computed successfully. Moving the arm.");
    //                 sleep(3);
            
    //                 // 生成机械臂的运动规划数据
    //                 moveit::planning_interface::MoveGroupInterface::Plan plan;
    //                 plan.trajectory_ = trajectory;

    //                 ROS_INFO("***********will excute second step********************");
    //                 // 执行运动
    //                 arm.execute(plan);
    //                 ROS_INFO("***********second step finished********************");
    //                 sleep(10);
                    
    //             }
    //             else
    //             {
    //                 ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    //             }
    
    ros::shutdown();
 
    return 0;

}
