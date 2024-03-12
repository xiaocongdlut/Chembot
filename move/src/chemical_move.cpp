#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <string.h>

ros::Publisher cmdVelPub;

void shutdown(int sig)
{
  cmdVelPub.publish(geometry_msgs::Twist());//ctrl+c
  ROS_INFO("goforward cpp ended!");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "GoForward");//int ros
  std::string topic = "/cmd_vel";
  ros::NodeHandle node;
  cmdVelPub = node.advertise<geometry_msgs::Twist>(topic, 1);
//=========================
  double rate = 10 ;
  ros::Rate loopRate(rate);
  float linear_speed     = 0.05;
  float goal_distance     = 0.5;
  float linear_duration = goal_distance / linear_speed;

  float angular_speed    = 1.0;
  float goal_angle    = M_PI;
  float angular_duration= goal_angle / angular_speed;

  int count = 0;
  int ticks;
//================================
  signal(SIGINT, shutdown);
  ROS_INFO("goforward cpp start...");
  geometry_msgs::Twist speed; 
//================================
  while (ros::ok())
  {
    speed.linear.x = linear_speed;
    ticks = int(linear_duration*rate);
    
    for(int i = 0;i<ticks; i++)
    {
        cmdVelPub.publish(speed);
        loopRate.sleep();
    }
    cmdVelPub.publish(geometry_msgs::Twist());
    ROS_INFO("axis y running!");


    speed.linear.x = 0;
    speed.linear.y = linear_speed;
    ticks = int(linear_duration*rate);
    for(int i = 0;i<ticks; i++)
    {
        cmdVelPub.publish(speed);
        loopRate.sleep();
    }

    speed.linear.x = 0;
    speed.linear.y = 0;
    cmdVelPub.publish(speed);
    loopRate.sleep();
    return 0;
  }

}
