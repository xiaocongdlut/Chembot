#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>
#include <tf/transform_listener.h>

/***********************
*                      *
*    修改好的           * 
*                      *
/***********************


/************************************************
 * 获取当前机器人位置坐标
*/
typedef geometry_msgs::Pose2D Pose;


Pose get_pose()
{
    geometry_msgs::Pose2D pose_now;

    tf::StampedTransform transform;
    tf::TransformListener listener;
    try
    {
        listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        // ROS_ERROR("%s", ex.what());
    }
    
    pose_now.x = static_cast<double>(transform.getOrigin().x());
    pose_now.y = static_cast<double>(transform.getOrigin().y());
    pose_now.theta = tf::getYaw(transform.getRotation());     // 弧度
    ROS_INFO_STREAM("pos_now.x: "<<pose_now.x);
    ROS_INFO_STREAM("pos_now.y: "<<pose_now.y);
    ROS_INFO_STREAM("pos_now.theta: "<<pose_now.theta);

    return pose_now;
}


/***********************************************
 * 直线移动
*/
void move(const ros::Publisher& cmdVelPub, const geometry_msgs::Twist& initial_speed,
                                           float goal_distance_x, float goal_distance_y)
{
    double rate = 100;
    ros::Rate loopRate(rate);

    float linear_speed_x = initial_speed.linear.x;
    float linear_speed_y = initial_speed.linear.y;
    float linear_duration_x = abs(goal_distance_x) / linear_speed_x;
    float linear_duration_y = abs(goal_distance_y) / linear_speed_y;

    int ticks;
    geometry_msgs::Twist speed; 

    ROS_INFO("axis x running!");
    speed.linear.x = linear_speed_x * (goal_distance_x / abs(goal_distance_x));
    speed.linear.y = 0;
    speed.angular.z = 0;
    ticks = int(linear_duration_x * rate);

    for(int i = 0;i<ticks; i++)
    {
        cmdVelPub.publish(speed);
        loopRate.sleep();
    }

    
    ROS_INFO("axis y running!");
    speed.linear.x = 0;
    speed.linear.y = linear_speed_y * (goal_distance_y / abs(goal_distance_y));
    speed.angular.z = 0;
    ticks = int(linear_duration_y * rate);
    for (int i = 0; i < ticks; i++)
    {
        cmdVelPub.publish(speed);
        loopRate.sleep();
    }

    speed.linear.x = 0;
    speed.linear.y = 0;
    speed.angular.z = 0;
    cmdVelPub.publish(speed);
    ros::Duration(3.0).sleep();
}

/***********************************************
 * 旋转
*/
void rotate(const Pose& initial_pose, const ros::Publisher& cmdVelPub, float angular_z, float theta)
{
    double rate = 100;
    ros::Rate loopRate(rate);
    geometry_msgs::Twist speed; 

    ROS_INFO("rotation!");
    speed.linear.x = 0;
    speed.linear.y = 0;
    speed.angular.z = angular_z * (theta / abs(theta));

    Pose now_pose=get_pose();

    while(abs(now_pose.theta- initial_pose.theta)>0.03)
    {
        cmdVelPub.publish(speed);            
        ros::Duration(0.5).sleep();
        now_pose = get_pose();
    }

    speed.linear.x = 0;
    speed.linear.y = 0;
    speed.angular.z = 0;
    cmdVelPub.publish(speed);
    ros::Duration(3.0).sleep();
}

/*************************************************
 * 处理最终位置误差
*/

void process_error(const Pose& initial_pose, const Pose& goal_pose, const Pose& final_pose, const ros::Publisher& cmdVelPub, 
                    const geometry_msgs::Twist& initial_speed)
{

    double threshold_x = 0.05;
    double threshold_y = 0.05;
    double threshold_theta = 0.05;

    double delta_x = goal_pose.x - final_pose.x;
    double delta_y = goal_pose.y - final_pose.y;
    double delta_theta = goal_pose.theta - final_pose.theta;
    ROS_INFO_STREAM("delta_x: " << delta_x);
    ROS_INFO_STREAM("delta_y: " << delta_y);
    ROS_INFO_STREAM("delta_theta: " << delta_theta);

    if(abs(delta_theta) >= threshold_theta)
        rotate(initial_pose, cmdVelPub, initial_speed.angular.z, delta_theta);

    if(abs(delta_x) >= threshold_x || abs(delta_y) >= threshold_y)
    {
        move(cmdVelPub, initial_speed, delta_x, delta_y);   
    }
    else
    {
        ROS_INFO("over!");
    }
}

void move_demo( const ros::Publisher& cmdVelPub,const geometry_msgs::Twist& initial_speed,double xxx,double yyy){
    Pose goal_pose;                   // 目标位置
    goal_pose.x = xxx;
    goal_pose.y = yyy;
    
    ROS_INFO_STREAM("goal_pos.x: " <<  goal_pose.x);
    ROS_INFO_STREAM("goal_pos.y: " <<   goal_pose.y);

    Pose initial_pose = get_pose();    // 初始位置
    ROS_INFO_STREAM("initial_pose.x: " <<  initial_pose.x);
    ROS_INFO_STREAM("initial_pose.y: " <<   initial_pose.y);

    float goal_distance_x =  goal_pose.x -initial_pose.x;
    float goal_distance_y = goal_pose.y -initial_pose.y;
    ROS_INFO_STREAM("goal_distance_x: " <<  goal_distance_x);
    ROS_INFO_STREAM("goal_distance_y: " <<   goal_distance_y);

    move(cmdVelPub, initial_speed, goal_distance_x, goal_distance_y);

    Pose final_pose = get_pose();    // 停止位置
    ROS_INFO_STREAM("final_pose.x: " <<  final_pose.x);
    ROS_INFO_STREAM("final_pose.y: " <<   final_pose.y);
    
    process_error(initial_pose, goal_pose, final_pose, cmdVelPub, initial_speed);
}


/************************************************
 * 主程序
*/

int main(int argc, char** argv)
{

    ros::init(argc, argv, "MovetoGoal");
    std::string topic = "/cmd_vel";
    ros::NodeHandle node;
    ros::Publisher cmdVelPub = node.advertise<geometry_msgs::Twist>(topic, 1);

    geometry_msgs::Twist initial_speed; 
    initial_speed.linear.x = 0.15;
    initial_speed.linear.y = 0.15;
    initial_speed.angular.z = 0.2;

    move_demo(cmdVelPub,initial_speed,4.80,0.20);
    move_demo(cmdVelPub,initial_speed,0,0);
    
    return 0;
}
