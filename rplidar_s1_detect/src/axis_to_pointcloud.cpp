#include"ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include"sensor_msgs/PointCloud.h"
#include"laser_geometry/laser_geometry.h"
laser_geometry::LaserProjection projector;
ros::Publisher scan_pub;
 
 
void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
sensor_msgs::PointCloud cloud;
projector.projectLaser(*scan_in,cloud);
scan_pub.publish(cloud);
}
 
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_scan_to_pointcloud");
    ros::NodeHandle nh;
    scan_pub=nh.advertise<sensor_msgs::PointCloud>("/scan/point_cloud",1000);
 
    ros::Subscriber sub = nh.subscribe("/scan", 1000, ScanCallback);
 
    ros::spin();
    return 0;
}
 
