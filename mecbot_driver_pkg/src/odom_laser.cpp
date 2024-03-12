#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

class LaserOdometry {
private:
    ros::Subscriber sub;  // Lidar扫描的ROS订阅器
    ros::Publisher pub;  // 里程计数据的ROS发布器
    tf::Transform last_transform;  // 上一个变换（从上次扫描以来的运动）
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud;  // 上一个点云
 //   tf::TransformBroadcaster tf_broadcaster;  // 用于发布变换信息

public:
    LaserOdometry() : last_transform(tf::Transform::getIdentity()), last_cloud(new pcl::PointCloud<pcl::PointXYZ>) {
        ros::NodeHandle nh;
        sub = nh.subscribe("/scan", 1000, &LaserOdometry::callback, this);  // 订阅Lidar扫描数据
        pub = nh.advertise<nav_msgs::Odometry>("/odom_laser", 50);  // 公开odom主题
    }

    void callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // 将扫描转换为点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (unsigned int i = 0; i < msg->ranges.size(); ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;
            pcl::PointXYZ point;
            point.x = msg->ranges[i] * cos(angle);
            point.y = msg->ranges[i] * sin(angle);
            point.z = 0;
            current_cloud->points.push_back(point);
        }

        // 使用ICP找到从上一个云到当前云的变换
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(current_cloud);
        icp.setInputTarget(last_cloud);
        pcl::PointCloud<pcl::PointXYZ> final;
        icp.align(final);

        // 从ICP结果中提取变换
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        tf::Matrix3x3 rotation(transformation(0, 0), transformation(0, 1), transformation(0, 2),
                               transformation(1, 0), transformation(1, 1), transformation(1, 2),
                               transformation(2, 0), transformation(2, 1), transformation(2, 2));
        tf::Vector3 translation(transformation(0, 3), transformation(1, 3), transformation(2, 3));
        tf::Transform current_transform(rotation, translation);

        // 累积变换并发布
        last_transform = last_transform * current_transform;

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        tf::poseTFToMsg(last_transform, odom.pose.pose);
        pub.publish(odom);

        // 保存当前点云以供下次使用
        last_cloud = current_cloud;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_odometry");
    LaserOdometry lo;
    ros::spin();  // 保持节点运行，直到它被关闭
    return 0;
}
