#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <vector>
#include <cmath>

#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener;


using namespace std;

typedef struct
{
double x;
double y;
}Boundary;

vector<Boundary> boundary;
Boundary boundary_point;

// 地图数据
nav_msgs::OccupancyGrid mapData;

// tf监听器
tf::TransformListener* listener;

// LaserScan到PointCloud2的转换器
laser_geometry::LaserProjection projector;

// 用于发布点云的发布器
ros::Publisher cloudPub;
ros::Publisher cloudPub_new;

void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    //[1]获得转换后点云
    // 将LaserScan转换为PointCloud
    sensor_msgs::PointCloud2 cloud_msg;
    projector.transformLaserScanToPointCloud("imu", *scan_msg, cloud_msg, *tfBuffer);

    // 定义转换后的点云
    sensor_msgs::PointCloud2 transformed_cloud;

    // 用tf2_ros库进行坐标系转换
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer->lookupTransform("map", "imu",
                                                    ros::Time(0),
                                                    ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }

    tf2::doTransform(cloud_msg, transformed_cloud, transformStamped);

//[2]转换后点云pointcloud2转pointcloud
    // 把sensor_msgs/PointCloud2转化为pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);//边界点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZ>);//新障碍物点云
    pcl::fromROSMsg(transformed_cloud, *cloud);

     int count = 0;
    // 处理每一个点
    for (const auto& point : cloud->points)
    {
        // 通过tf来把点从雷达坐标系转化到地图坐标系
        tf::StampedTransform transform;
        try
        {
            listener->lookupTransform(mapData.header.frame_id, transformed_cloud.header.frame_id,
                                      ros::Time(0), transform);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }

        // 做坐标系转换
        tf::Vector3 pointInMapFrame = transform * tf::Vector3(point.x, point.y, point.z);

        // 如果这个点在地图范围内
    bool point_in_boundary=false;
	for(int i=0; i<boundary.size();i++){
		double ddd=sqrt(pow(point.x-boundary[i].x,2)+pow(point.y-boundary[i].y,2));
        	
		if (ddd<0.15){
			ROS_WARN("ddd %f)", ddd);
	                cloud_boundary->points.push_back(point);
                    point_in_boundary=true;
			break;
		}
	}

    if (point_in_boundary==false){
	                cloud_new->points.push_back(point);
    }

    count++;
    }

    ROS_INFO("sacn data num: %d", count);
    count = 0;

    // 将过滤后的点云发布出来
    sensor_msgs::PointCloud2 cloudMsgOut;
    pcl::toROSMsg(*cloud_boundary, cloudMsgOut);
    cloudMsgOut.header = transformed_cloud.header;
    cloudPub.publish(cloudMsgOut);

    sensor_msgs::PointCloud2 cloudMsgOut_new;
    pcl::toROSMsg(*cloud_new, cloudMsgOut_new);
    cloudMsgOut_new.header = transformed_cloud.header;
    cloudPub_new.publish(cloudMsgOut_new);

}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    mapData = *map;

    int width = map->info.width;
    int height = map->info.height;
    double resolution = map->info.resolution;  // 每个单元格的大小，单位是米
	ROS_WARN("resolution %f)", resolution);  // 0.05
    

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            // 获取该单元格的值
            int value = map->data[y * width + x];

            // 如果值大于50，我们认为这个单元格是障碍物
            if (value > 50)
            {
                // 计算该单元格在地图坐标系中的位置
		
                boundary_point.x = x * resolution + map->info.origin.position.x;
                boundary_point.y = y * resolution + map->info.origin.position.y;
		        boundary.push_back(boundary_point);

                ROS_INFO("Obstacle at (%f, %f)", boundary_point.x, boundary_point.y);
            }
        }
    }

		ROS_WARN("boundary.size() in mapCallback  %d", boundary.size());

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_detector");
    ros::NodeHandle nh;

    // 创建tf的监听器
    tfBuffer = std::make_shared<tf2_ros::Buffer>();
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    listener = new tf::TransformListener();
    cloudPub = nh.advertise<sensor_msgs::PointCloud2>("boundary_cloud", 1);
    cloudPub_new = nh.advertise<sensor_msgs::PointCloud2>("new_cloud", 1);

    ros::Subscriber mapSub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber sub = nh.subscribe("/scan", 1, scan_cb);

    ros::spin();

    delete listener;

    return 0;
}
