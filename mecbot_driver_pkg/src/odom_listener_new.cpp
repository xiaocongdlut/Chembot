#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <fstream>

// 这个函数将odom的数据写入文件
void writeToFile(const geometry_msgs::PoseStamped& odom_data) {
    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_data.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    std::ofstream file;
    file.open("/home/dut-coc/Mecbot_ws/src/mecbot_driver_pkg/launch/odom_cat.txt");
    file << odom_data.pose.position.x << "\n";
    file << odom_data.pose.position.y << "\n";
    file << yaw << "\n";
    file.close();
}

void odomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    writeToFile(*msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_listener");
    ros::NodeHandle nh;

    // After setting the initial pose, we can start subscribing to the odom topic to update the state
    ros::Subscriber sub = nh.subscribe("odom_cat", 10, odomCallback);
    ros::spin(); // Let ROS take over from here


    return 0;
}
