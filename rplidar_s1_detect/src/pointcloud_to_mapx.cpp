#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>

ros::Publisher pub;
std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener;
laser_geometry::LaserProjection projector;

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vec;

void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    // 将LaserScan转换为PointCloud
    sensor_msgs::PointCloud2 cloud_msg;
    projector.transformLaserScanToPointCloud("laser_link", *scan_msg, cloud_msg, *tfBuffer);

    // 定义转换后的点云
    sensor_msgs::PointCloud2 transformed_cloud;

    // 用tf2_ros库进行坐标系转换
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer->lookupTransform("map", "laser_link",
                                                    ros::Time(0),
                                                    ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }

    tf2::doTransform(cloud_msg, transformed_cloud, transformStamped);

    // 发布转换后的点云
    pub.publish(transformed_cloud);

    // 将转换后的点云保存到vector中
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_cloud, *pcl_cloud);
    cloud_vec.push_back(pcl_cloud);
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "transform_pointcloud");

    ros::NodeHandle nh;

    // 创建tf的监听器
    tfBuffer = std::make_shared<tf2_ros::Buffer>();
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    // 订阅LaserScan
    ros::Subscriber sub = nh.subscribe("/laserscan", 1, scan_cb);

    // 发布转换后的点云
    pub = nh.advertise<sensor_msgs::PointCloud2>("/transformed_pointcloud", 1);

    ros::spin();

    return 0;
}
