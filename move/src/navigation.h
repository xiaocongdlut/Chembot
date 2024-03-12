#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/UInt8MultiArray.h>
#include "std_msgs/UInt8.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <string.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <thread>
#include<ros/xmlrpc_manage>

using namespace std;

struct Pose
{
    struct Position
    {
        double x;
        double y;
    };
    double yaw;
    Position position;
    // ==重载
    bool operator==(const Pose &p)
    {
        return (p.position.x == position.x && p.position.y == position.y && p.yaw == yaw);
    }
};

class Navigation
{
public:
    Navigation();
    ~Navigation();
    // 处理odom回调函数
    void OdomCallback(const geometry_msgs::PoseStamped &msg);
    // 获取当前pose
    Pose GetPose();
    // move
    void Move(Pose target_pose);
    // rotate
    void Rotate(Pose target_pose);
    // 初始化pose
    void InitPose();
    // 处理error
    void CompensateError(Pose target_pose);
    // move to target_pose
    void MoveToTarget(Pose target_pos);
    // 处理mes回调函数
    void MesCallback(const std_msgs::UInt8MultiArrayConstPtr &mes_msg);
    // 反馈给mes的message
    void FeedbackToMes();
    // run
    void Run();
    void MoveToStation(Pose target_pos);
    void MoveToCompensatePoint(Pose target_pos);
    void MoveToMiddle(Pose target_pos);
    void MoveToCharge(vector<Pose> charge_route);
    void LeaveToOrigin(vector<Pose>leave_route);
    
private:
    ros::NodeHandle node_;
    ros::Subscriber odom_sub_;
    ros::Subscriber mes_msg_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher mes_msg_pub_;
    std::thread callbackThread_;
    // move speed
    geometry_msgs::Twist move_speed_;
    // initial pose
    Pose initial_pose_;
    Pose pose_;
    // threshold x y theta
    double threshold_x_, threshold_y_, threshold_theta_;
    // int loop_count_;
    // 存储工位位置
    vector<Pose> target_pose_;
    // 存储mes信息，指示到达哪个target_pose
    int mes_msg_;
    vector<Pose> charge_route_;
    vector<Pose> leave_route_;
     double threshold_distance_;
};
