#include "navigation.h"

Navigation::Navigation()
{
    move_speed_.linear.x = 0.15;
    move_speed_.linear.y = 0.15;
    move_speed_.angular.z = 0.2;

    threshold_x_ = 0.03;
    threshold_y_ = 0.03;
    threshold_theta_ = 0.02;
    // loop_count_ = 0;

    Pose target_pose_one;
    target_pose_one.position.x = 0;
    target_pose_one.position.y = 0;
    target_pose_one.yaw = 0;

    Pose target_pose_two;
    target_pose_two.position.x = 4.625;
    target_pose_two.position.y = 0.56;
    target_pose_two.yaw = 0;

    Pose target_pose_three;
    target_pose_three.position.x = -0.285;
    target_pose_three.position.y = 0.895;
    target_pose_three.yaw = 0;

    Pose target_pose_four;
    target_pose_four.position.x = 2;
    target_pose_four.position.y = 0.895;
    target_pose_four.yaw = 0;

    Pose target_pose_five;
    target_pose_five.position.x = 4.625;
    target_pose_five.position.y = -0.64;
    target_pose_five.yaw = 0;

    target_pose_.push_back(target_pose_one);
    target_pose_.push_back(target_pose_two);
    target_pose_.push_back(target_pose_three);
    target_pose_.push_back(target_pose_four);
    target_pose_.push_back(target_pose_five);

    initial_pose_ = target_pose_one;

    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    mes_msg_pub_ = node_.advertise<std_msgs::String>("/mes1", 1);

    mes_msg_sub_ = node_.subscribe("/mes", 1, &Navigation::MesCallback, this);
    odom_sub_ = node_.subscribe("/odom_cat", 1, &Navigation::OdomCallback, this);
}

Navigation::~Navigation()
{
}

void Navigation::OdomCallback(const geometry_msgs::PoseStamped &msg)
{
    // ROS_INFO("Callback");
    pose_.position.x = msg.pose.position.x;
    pose_.position.y = msg.pose.position.y;
    double qx = msg.pose.orientation.x;
    double qy = msg.pose.orientation.y;
    double qz = msg.pose.orientation.z;
    double qw = msg.pose.orientation.w;
    double roll, pitch, yaw;
    tf::Quaternion quat(qx, qy, qz, qw);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    pose_.yaw = yaw;
    ROS_INFO_STREAM("pose_.position.x:\t" << pose_.position.x << "\n");
    ROS_INFO_STREAM("pose_.position.y:\t" << pose_.position.y << "\n");
    ROS_INFO_STREAM("pose_.yaw:\t" << pose_.yaw << "\n");
}

Pose Navigation::GetPose()
{
    // ros::spinOnce();
    ROS_INFO_STREAM("pose_.position.x:\t" << pose_.position.x << "\n");
    return pose_;
}

void Navigation::Move(Pose target_pose)
{
    int rate = 100;
    ros::Rate loopRate(100);
    Pose current_pose = GetPose();
    ROS_INFO_STREAM("current_pose.position.x:\t" << current_pose.position.x << "\n");
    ROS_INFO_STREAM("current_pose.position.y:\t" << current_pose.position.y << "\n");
    ROS_INFO_STREAM("current_pose.yaw:\t" << current_pose.yaw << "\n");
    float delta_x = target_pose.position.x - current_pose.position.x;
    float delta_y = target_pose.position.y - current_pose.position.y;
    float linear_duration_x = abs(delta_x) / move_speed_.linear.x;
    float linear_duration_y = abs(delta_y) / move_speed_.linear.y;
    geometry_msgs::Twist speed;
    ROS_INFO("direction x is running!\n");
    speed.linear.x = move_speed_.linear.x * (delta_x / abs(delta_x));
    speed.linear.y = 0;
    speed.angular.z = 0;
    int ticks = int(linear_duration_x * rate);
    for (int i = 0; i < ticks; i++)
    {
        cmd_vel_pub_.publish(speed);
        loopRate.sleep();
    }
    ROS_INFO("direction y is running!\n");
    speed.linear.x = 0;
    speed.linear.y = move_speed_.linear.y * (delta_y / abs(delta_y));
    speed.angular.z = 0;
    ticks = int(linear_duration_y * rate);
    for (int i = 0; i < ticks; i++)
    {
        cmd_vel_pub_.publish(speed);
        loopRate.sleep();
    }
    speed.linear.x = 0;
    speed.linear.y = 0;
    speed.angular.z = 0;
    cmd_vel_pub_.publish(speed);
    ros::Duration(3.0).sleep();
}

void Navigation::Rotate()
{
    Pose current_pose = GetPose();
    float delta_theta = initial_pose_.yaw - current_pose.yaw;
    geometry_msgs::Twist speed;
    ROS_INFO("rotation!\n");
    speed.linear.x = 0;
    speed.linear.y = 0;
    speed.angular.z = move_speed_.angular.z * (delta_theta / abs(delta_theta));

    while (abs(current_pose.yaw - initial_pose_.yaw) > threshold_theta_)
    {
        delta_theta = initial_pose_.yaw - current_pose.yaw;
        speed.angular.z = move_speed_.angular.z * (delta_theta / abs(delta_theta));
        cmd_vel_pub_.publish(speed);
        ros::Duration(0.5).sleep();
        current_pose = GetPose();
    }
    speed.linear.x = 0;
    speed.linear.y = 0;
    speed.angular.z = 0;
    cmd_vel_pub_.publish(speed);
    ros::Duration(3.0).sleep();
}

void Navigation::InitPose()
{
    Rotate();
    Move(initial_pose_);
    FeedbackToMes();
}

void Navigation::CompensateError(Pose target_pose)
{
    ROS_INFO("CompensateError");
    Pose current_pose = GetPose();
    ROS_INFO_STREAM("current_pose:\t" << current_pose.position.x << "\n");
    double delta_x = target_pose.position.x - current_pose.position.x;
    double delta_y = target_pose.position.y - current_pose.position.y;
    double delta_theta = target_pose.yaw - current_pose.yaw;

    ROS_INFO_STREAM("delta_theta:\t" << delta_theta << "\n");
    // if (abs(delta_theta) >= threshold_theta_)
    //     Rotate();
    while (abs(delta_x) >= threshold_x_ || abs(delta_y) >= threshold_y_ ||
           abs(delta_theta) >= threshold_theta_)
    {
        if (abs(delta_theta) >= threshold_theta_)
            Rotate();
        Move(target_pose);
        current_pose = GetPose();
        delta_x = target_pose.position.x - current_pose.position.x;
        delta_y = target_pose.position.y - current_pose.position.y;
        delta_theta = target_pose.yaw - current_pose.yaw;
    }
}

void Navigation::FeedbackToMes()
{
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(10);
    while (ros::Time::now() - start_time < ros::Duration(2.0))
    {
        std_msgs::String msg;
        msg.data = "OK!";
        mes_msg_pub_.publish(msg);
        rate.sleep();
    }
}

void Navigation::MoveToTarget(Pose target_pos)
{
    Move(target_pos);
    CompensateError(target_pos);
    FeedbackToMes();
}

void Navigation::MesCallback(const std_msgs::Char::ConstPtr &mes_msg)
{
    mes_msg_ = mes_msg->data;
}

void Navigation::Run()
{
    ros::AsyncSpinner spinner(2);
    spinner.start();
    while (ros::ok())
    {
        MoveToTarget(target_pose_[1]);
        MoveToTarget(target_pose_[2]);
        MoveToTarget(target_pose_[3]);
        MoveToTarget(target_pose_[4]);
        MoveToTarget(target_pose_[0]);
    }
    spinner.stop();
}
