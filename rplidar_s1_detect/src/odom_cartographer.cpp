// 订阅 cartographer与amcl算法协同定位后地图坐标 map 与机器人根结点 base_link （或 base_footprint）之间的TF关系，
// 构建名为 odom_cartographer 的topic发布器，发布位置坐标，用于导航。

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_cartographer_tf_transform");

    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // 创建名为 odom_cartographer topic 的发布器
    ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("odom_cartographer", 10);

    ros::Rate rate(10.0);
    while (node.ok()) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            // 监听从 /map 到 base_link 的转换
            transformStamped = tfBuffer.lookupTransform("map", "base_link",
                ros::Time(0));
        }
        catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // 构建一个 PoseStamped 消息
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z;
        pose.pose.orientation.x = transformStamped.transform.rotation.x;
        pose.pose.orientation.y = transformStamped.transform.rotation.y;
        pose.pose.orientation.z = transformStamped.transform.rotation.z;
        pose.pose.orientation.w = transformStamped.transform.rotation.w;

        // 将 PoseStamped 消息发布到 odom_cartographer topic
        pub.publish(pose);

        rate.sleep();
    }
    return 0;
};
