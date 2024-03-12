#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>

void writeToFile(const geometry_msgs::PoseStamped& odom_data) {
    std::ofstream file;
    file.open("odom.txt");
    file << "Position:\n";
    file << "x: " << odom_data.pose.position.x << "\n";
    file << "y: " << odom_data.pose.position.y << "\n";
    file << "z: " << odom_data.pose.position.z << "\n";
    file << "Orientation:\n";
    file << "x: " << odom_data.pose.orientation.x << "\n";
    file << "y: " << odom_data.pose.orientation.y << "\n";
    file << "z: " << odom_data.pose.orientation.z << "\n";
    file << "w: " << odom_data.pose.orientation.w << "\n";
    file.close();
}

void odomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    writeToFile(*msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_listener2");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("odom_cat", 10, odomCallback);

    ros::spin(); 

    return 0;
}
