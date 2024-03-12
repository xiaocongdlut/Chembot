#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

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
                double posX = x * resolution + map->info.origin.position.x;
                double posY = y * resolution + map->info.origin.position.y;

                ROS_INFO("Obstacle at (%f, %f)", posX, posY);
            }
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_extractor");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/map", 1, mapCallback);

    ros::spin();

    return 0;
}
