#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <tf/tf.h>
#include <string.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/xmlrpc_manager.h>
using namespace std;

struct Pose // 存储位姿的结构体，包含(x,y)、yaw
{
    struct Position
    {
        double x;
        double y;
    };
    double yaw;
    Position position;

    bool operator==(const Pose &p) // 重载==
    {
        return (p.position.x == position.x && p.position.y == position.y && p.yaw == yaw);
    }
};

class Navigation
{
public:
    Navigation();                                                                      // 构造函数
    ~Navigation();                                                                     // 析构函数
    void OdomCallback(const geometry_msgs::PoseStamped &msg);                          // 处理odom的回调函数
    Pose GetPose();                                                                    // 获取当前位姿
    void Move(Pose target_pose);                                                       // x、y方向的移动
    void Rotate(Pose target_pose);                                                     // 绕z轴的旋转
    void InitPose();                                                                   // 初始化位姿
    void CompensateError(Pose target_pose);                                            // 处理位姿误差
    void MoveToTarget(Pose target_pos);                                                // 移动到目标点
    void MesCallback(const std_msgs::UInt8MultiArrayConstPtr &mes_msg);                // 处理mes的回调函数
    void FeedbackToMes();                                                              // 反馈给mes的message
    void MoveToIntermediatePoint(Pose target_pos);                                     // 回中间点
    void MoveToCompensationPoint(Pose target_pos);                                     // 在长距离情况下增加的中间位置补偿点
    void MoveToStation(Pose target_pos);                                               // 移动到某个工位
    void MoveToCharge(vector<Pose> charge_route);                                      // 移动至充电庄
    void MoveToTurningPoint(Pose target_pos);                                          // 在x轴方向向目标点前进
    void Run();                                                                        // 运行程序
    void PointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &pointCloud2_msg); //处理点云
    void LeaveToOrigin(vector<Pose> leave_route);                                      // 回到初始点
    void AngleCorrection(double theta);                                                //角度纠正
    bool CheckAMCL();                                                                  // 检验amcl是否开启

private:
    ros::NodeHandle node_;        // 节点句柄
    ros::Subscriber odom_sub_;    // 里程计信息订阅者
    ros::Subscriber mes_msg_sub_; // mes消息订阅者
    ros::Subscriber pointCloud2_sub_;
    ros::Publisher cmd_vel_pub_;                         // 速度消息发布者
    ros::Publisher mes_msg_pub_;                         // mes消息发布者
    std::thread callbackThread_;                         // 回调函数线程
    geometry_msgs::Twist move_speed_;                    // 移动速度
    Pose initial_pose_;                                  // 初始位姿
    Pose pose_;                                          // 存储当前位姿
    double threshold_x_, threshold_y_, threshold_theta_; // x、y、theta的误差阈值
    vector<Pose> target_pose_;                           // 存储工位位置信息
    int mes_msg_;                                        // 存储mes信息，指示到达哪个target_pose
    double threshold_distance_;                          // 距离阈值
    vector<Pose> charge_route_;                          // 充电路线
    vector<Pose> leave_route_;                           // 离开路线
    double threshold_obstacle_x;                         // 判断为障碍物的x方向阈值
    double threshold_obstacle_y;                         // 判断为障碍物的y方向阈值
    bool is_obstacle_;                                   // 判断是否为障碍物的阈值
};
