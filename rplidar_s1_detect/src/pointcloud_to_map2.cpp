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
using namespace std;



typedef struct
{
double x;
double y;
}Boundary;

vector<Boundary> boundary;

typedef struct
{
double x;
double y;
}Mappoint;

vector<Mappoint>mappoint;

// 地图数据
nav_msgs::OccupancyGrid mapData;

// tf监听器
tf::TransformListener* listener;

// LaserScan到PointCloud2的转换器
laser_geometry::LaserProjection projector;

// 用于发布点云的发布器
ros::Publisher cloudPub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg)
{
    double x =0;
    double y=0;
    // 把LaserScan转化为PointCloud2
    sensor_msgs::PointCloud2 cloudMsg;
    projector.transformLaserScanToPointCloud(scanMsg->header.frame_id, *scanMsg, cloudMsg, *listener);

    // 把sensor_msgs/PointCloud2转化为pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloudMsg, *cloud);

    // 处理每一个点
    for ( auto& point : cloud->points)
    {
        // 通过tf来把点从雷达坐标系转化到地图坐标系
        tf::StampedTransform transform;
        try
        {
            listener->lookupTransform("/map","/laser_link",
                                      ros::Time(0), transform);

        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }

        // 做坐标系转换
        tf::Vector3 pointInMapFrame = transform * tf::Vector3(point.x, point.y, point.z);

        // 把这个点对应到地图坐标系
        int map_x = (pointInMapFrame.x() - mapData.info.origin.position.x) / mapData.info.resolution;
        int map_y = (pointInMapFrame.y() - mapData.info.origin.position.y) / mapData.info.resolution;
              //位置
     
        point.x=map_x;
        point.y=map_y;  
       cloud_filtered->points.push_back(point);
       
       
        // ROS_INFO("Detected obstacle at (%f, %f)", map_x, map_y);

        
        // 如果这个点在地图范围内
	// for(int i=0; i<boundary.size();i++){
	// 	double ddd=sqrt(pow(point.x-boundary[i].x,2)+pow(point.y-boundary[i].y,2));
  //       ROS_WARN("ddd %f)", ddd);
	// 	if (ddd<1){
	//                 cloud_filtered->points.push_back(point);
	// 		break;
		//}
	//}





/*
        if (map_x >= 0 && map_x < mapData.info.width && map_y >= 0 && map_y < mapData.info.height)
        {
            // 在地图上找到对应的cell
            int8_t cellValue = mapData.data[map_y * mapData.info.width + map_x];

            // 如果在正负5cm范围内，则视为障碍物边界
            if (cellValue >= -5 && cellValue <= 5)
            {
                ROS_INFO("Detected obstacle at (%f, %f, %f)", point.x, point.y, point.z);
                cloud_filtered->points.push_back(point);
            }
        }
*/
    }

    // 将过滤后的点云发布出来
    sensor_msgs::PointCloud2 cloudMsgOut;
    pcl::toROSMsg(*cloud_filtered, cloudMsgOut);
    cloudMsgOut.header = cloudMsg.header;
    cloudPub.publish(cloudMsgOut);
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    int width = map->info.width;
    int height = map->info.height;
    double resolution = map->info.resolution;  // 每个单元格的大小，单位是米

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
		Boundary boundary_point;
                boundary_point.x = x * resolution + map->info.origin.position.x;
                boundary_point.y = y * resolution + map->info.origin.position.y;
		        boundary.push_back(boundary_point);

                ROS_INFO("Obstacle at (%f, %f)", boundary_point.x, boundary_point.y);
            }
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_detector");
    ros::NodeHandle nh;



    listener = new tf::TransformListener();
    cloudPub = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);

ros::Subscriber scanSub = nh.subscribe("/scan", 1, scanCallback);
    ros::Subscriber mapSub = nh.subscribe("/map", 1, mapCallback);

    ros::spin();

    delete listener;

    return 0;
}