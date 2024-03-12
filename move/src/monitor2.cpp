#include <ros/ros.h>
#include <thread>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8MultiArray.h>

// 定义一个全局变量，用于记录上次收到速度消息的时间
ros::Time last_time;

// 定义一个回调函数，用于处理速度消息
void velocityCallback(const geometry_msgs::Twist &msg)
{
    // 如果速度消息不为零，说明机器人在运动，更新上次收到速度消息的时间
    if (msg.linear.x != 0.0 || msg.linear.y != 0.0 || msg.angular.z != 0.0)
    {
        last_time = ros::Time::now();
    }
}

void MesCallback(const std_msgs::UInt8MultiArrayConstPtr &mes_msg)
{
    if (mes_msg->data[0] == 1)
    {
       system("rosrun amcl amcl &");
       ROS_INFO("restart");
    }
    
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "robot_monitor");
    ros::NodeHandle nh;
   last_time=ros::Time::now();

    // 订阅速度话题
    ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 10, velocityCallback);

    // ros::Subscriber mes_sub = nh.subscribe("Mes_Command", 100, MesCallback);

    // std::thread callbackThread = std::thread([&]()
    //                                           {ros::MultiThreadedSpinner spinner(1);spinner.spin(); });

    // 定义一个定时器，每隔一秒检查一次机器人的运动状态
    ros::Timer timer = nh.createTimer(
        ros::Duration(1.0), [&](const ros::TimerEvent &event)
        {
            // 获取当前时间
            ros::Time now = ros::Time::now();

            // 如果距离上次收到速度消息的时间超过两分钟，说明机器人已经停止运动
            if (now - last_time > ros::Duration(30.0))
            {
                last_time=ros::Time::now();
                // 关闭robot_node节点
                system("rosnode kill /amcl");       
            } });

    // 进入循环，等待回调函数和定时器函数执行
    ros::spin();
    return 0;
}
