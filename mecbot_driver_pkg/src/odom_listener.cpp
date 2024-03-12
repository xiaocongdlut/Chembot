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
    file.open("/home/mecbot/Mecbot_ws/src/mecbot_driver_pkg/launch/odom_cat.txt");
    file << odom_data.pose.position.x << "\n";
    file << odom_data.pose.position.y << "\n";
    file << yaw << "\n";
    file.close();
}

// 这个函数将读取odom的数据
geometry_msgs::PoseStamped readFromFile() {
    std::ifstream file;
    file.open("/home/dut-coc/Mecbot_ws/odom.txt");

    geometry_msgs::PoseStamped odom_data;
    double yaw;

    std::string ignore;
    file >> ignore >> ignore >> odom_data.pose.position.x;
    file >> ignore >> ignore >> odom_data.pose.position.y;
    file >> ignore >> ignore >> ignore >> yaw;

    // Convert yaw to quaternion for PoseStamped
    tf::Quaternion quat = tf::createQuaternionFromYaw(yaw);
    tf::quaternionTFToMsg(quat, odom_data.pose.orientation);

    return odom_data;
}

void odomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    writeToFile(*msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_listener");
    ros::NodeHandle nh;

    // If this is the first run, we don't have a saved state, so we subscribe to the odom topic
    if (!std::ifstream("odom.txt")) {
        ros::Subscriber sub = nh.subscribe("odom_cat", 10, odomCallback);
        ros::spin(); // Let ROS run until the node is shut down
    } else {
        // If this is not the first run, we read the saved state and use it as the initial pose for AMCL
        geometry_msgs::PoseStamped initial_pose = readFromFile();
        nh.setParam("/amcl/initial_pose_x", initial_pose.pose.position.x);
        nh.setParam("/amcl/initial_pose_y", initial_pose.pose.position.y);

        tf::Quaternion quat;
        tf::quaternionMsgToTF(initial_pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        nh.setParam("/amcl/initial_pose_a", yaw);

        // After setting the initial pose, we can start subscribing to the odom topic to update the state
        ros::Subscriber sub = nh.subscribe("odom_cat", 10, odomCallback);
        ros::spin(); // Let ROS take over from here
    }

    return 0;
}
