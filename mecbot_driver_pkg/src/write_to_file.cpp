#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <fstream>

void odomCallback(const geometry_msgs::PoseStamped& odom_data)
{

    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_data.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // 创建一个YAML节点
    YAML::Node yaml_node;

    // 将odom数据写入YAML节点
    yaml_node["initial_pose_x"] = odom_data.pose.position.x;
    yaml_node["initial_pose_y"] = odom_data.pose.position.y;
    yaml_node["initial_pose_a"] = yaw;

    // 将YAML节点写入文件
    std::ofstream file("/home/mecbot/Mecbot_ws/src/mecbot_driver_pkg/launch/odom_cat.yaml");
    file << yaml_node;
    file.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_to_yaml");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/odom_cat", 10, odomCallback);

    // 进入ROS循环
    ros::spin();

    return 0;
}
