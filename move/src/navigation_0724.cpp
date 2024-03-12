#include "navigation.h"

Navigation::Navigation()
{
    is_obstacle_ = 1;

    move_speed_.linear.x = 0.15;
    move_speed_.linear.y = 0.15;
    move_speed_.angular.z = 0.2;

    threshold_x_ = 0.03;
    threshold_y_ = 0.03;
    threshold_theta_ = 0.02;

    mes_msg_ = 0;

    threshold_distance_ = 3;

    threshold_obstacle_x = 1;
    threshold_obstacle_y = 0.6;

    Pose target_pose_zero; // initial pose
    target_pose_zero.position.x = 0;
    target_pose_zero.position.y = 0;
    target_pose_zero.yaw = 0;

    Pose target_pose_one; // target one 称量工位
    target_pose_one.position.x = 4.78;
    target_pose_one.position.y = 0.63;
    target_pose_one.yaw = 0;

    Pose target_pose_two; // target two 移液工位
    target_pose_two.position.x = -0.34;
    target_pose_two.position.y = 0.86;
    target_pose_two.yaw = 0;

    Pose target_pose_three; // target three 旋盖机工位
    target_pose_three.position.x = 1.65;
    target_pose_three.position.y = 0.78;
    target_pose_three.yaw = 0;

    Pose target_pose_four; // target four 气质联用
    target_pose_four.position.x = 4.68;
    target_pose_four.position.y = -0.95;
    target_pose_four.yaw = M_PI;

    Pose target_pose_nine; // target nine 通风橱
    target_pose_nine.position.x = 4.625;
    target_pose_nine.position.y = -0.84;
    target_pose_nine.yaw = 0;

    Pose target_pose_ten; // target ten 液质联用
    target_pose_ten.position.x = 4.625;
    target_pose_ten.position.y = -0.84;
    target_pose_ten.yaw = M_PI;

    target_pose_.push_back(target_pose_zero);
    target_pose_.push_back(target_pose_one);
    target_pose_.push_back(target_pose_two);
    target_pose_.push_back(target_pose_three);
    target_pose_.push_back(target_pose_four);
    target_pose_.push_back(target_pose_nine);
    target_pose_.push_back(target_pose_ten);

    initial_pose_ = target_pose_zero;

    Pose route_point_zero;
    route_point_zero.position.x = 0;
    route_point_zero.position.y = 0;
    route_point_zero.yaw = 0;

    Pose route_point_one;
    route_point_one.position.x = -1.20;
    route_point_one.position.y = 0;
    route_point_one.yaw = -M_PI / 2;

    Pose route_point_two;
    route_point_two.position.x = -1.24;
    route_point_two.position.y = -1.92;
    route_point_two.yaw = -M_PI / 2;

    Pose route_point_three;
    route_point_three.position.x = -1.24;
    route_point_three.position.y = -3.65;
    route_point_three.yaw = -M_PI / 2;

    Pose route_point_four;
    route_point_four.position.x = -0.277;
    route_point_four.position.y = -3.65;
    route_point_four.yaw = M_PI;

    Pose route_point_five;
    route_point_five.position.x = -1.36;
    route_point_five.position.y = -3.60;
    route_point_five.yaw = M_PI;

    Pose route_point_six;
    route_point_six.position.x = -1.36;
    route_point_six.position.y = -2.77;
    route_point_six.yaw = M_PI;

    Pose route_point_seven;
    route_point_seven.position.x = -2.0;
    route_point_seven.position.y = -2.77;
    route_point_seven.yaw = M_PI;






    Pose leave_point_zero;
    leave_point_zero.position.x = -1.36;
    leave_point_zero.position.y = -2.77;
    leave_point_zero.yaw = M_PI;

    Pose leave_point_one;
    leave_point_one.position.x = -1.36;
    leave_point_one.position.y = -3.60;
    leave_point_one.yaw = M_PI;

    Pose leave_point_two;
    leave_point_two.position.x = -1.24;
    leave_point_two.position.y = -3.60;
    leave_point_two.yaw = M_PI / 2;

    Pose leave_point_three;
    leave_point_three.position.x = -1.24;
    leave_point_three.position.y = -1.92;
    leave_point_three.yaw = M_PI / 2;

    Pose leave_point_four;
    leave_point_four.position.x = 0;
    leave_point_four.position.y = 0;
    leave_point_four.yaw = 0;


    leave_route_.push_back(leave_point_zero);
    leave_route_.push_back(leave_point_one);
    leave_route_.push_back(leave_point_two);
    leave_route_.push_back(leave_point_three);
    leave_route_.push_back(leave_point_four);

    charge_route_.push_back(route_point_zero);
    charge_route_.push_back(route_point_one);
    charge_route_.push_back(route_point_two);
    charge_route_.push_back(route_point_three);
    charge_route_.push_back(route_point_four);
    charge_route_.push_back(route_point_five);
    charge_route_.push_back(route_point_six);
    charge_route_.push_back(route_point_seven);

    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    mes_msg_pub_ = node_.advertise<std_msgs::UInt8>("/chessis_feedback_to_mes", 100);

    mes_msg_sub_ = node_.subscribe("Mes_Command", 100, &Navigation::MesCallback, this);
    odom_sub_ = node_.subscribe("/odom_cat", 100, &Navigation::OdomCallback, this);
    pointCloud2_sub_ = node_.subscribe("/point_cloud2", 100, &Navigation::PointCloud2Callback, this);

    callbackThread_ = std::thread([&]()
                                  {ros::MultiThreadedSpinner spinner(3);spinner.spin(); }); // 开两个线程，用来处理两个回调函数
}

Navigation::~Navigation()
{
    ros::shutdown();
}

void Navigation::OdomCallback(const geometry_msgs::PoseStamped &msg)
{
    pose_.position.x = msg.pose.position.x;
    pose_.position.y = msg.pose.position.y;
    double qx = msg.pose.orientation.x;
    double qy = msg.pose.orientation.y;
    double qz = msg.pose.orientation.z;
    double qw = msg.pose.orientation.w;
    double roll, pitch, yaw;
    tf::Quaternion quat(qx, qy, qz, qw);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 将四元数转化为偏航角yaw
    pose_.yaw = yaw;
}

void Navigation::PointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &pointCloud2_msg)
{
    Pose robot_position = GetPose();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*pointCloud2_msg, cloud);
    for (int i = 0; i < cloud.points.size(); i++)
    {
        if (abs(cloud.points[i].x - robot_position.position.x) <= threshold_obstacle_x &&
            abs(cloud.points[i].y - robot_position.position.y) <= threshold_obstacle_y)
        {
            move_speed_.linear.x = 0;
            move_speed_.linear.y = 0;
            move_speed_.angular.z = 0;
            is_obstacle_ = 1;
            return;
        }
    }
    move_speed_.linear.x = 0.15;
    move_speed_.linear.y = 0.15;
    move_speed_.angular.z = 0.2;
    is_obstacle_ = 0;
}

Pose Navigation::GetPose()
{
    return pose_; // 返回当前位姿
}

void Navigation::Move(Pose target_pose)
{
    int rate = 200;
    ros::Rate loopRate(rate); // 速度发布频率，哪个频率最合适需要尝试

    Pose current_pose = GetPose();
    float delta_x = target_pose.position.x - current_pose.position.x;
    float delta_y = target_pose.position.y - current_pose.position.y;
    float linear_duration_x = abs(delta_x) / move_speed_.linear.x;
    float linear_duration_y = abs(delta_y) / move_speed_.linear.y;

    geometry_msgs::Twist speed;
    if (abs(current_pose.yaw - 0) < 0.2 || abs(abs(current_pose.yaw) - M_PI) < 0.2) // 朝向为0或者pi的情况
    {
        int flag = 1; // 用于判断车的移动方向
        if (abs(abs(current_pose.yaw) - M_PI) < 0.2)
            flag = -1;
        cout << "x方向正在移动!" << endl;
        speed.linear.x = flag * move_speed_.linear.x * (delta_x / abs(delta_x));
        speed.linear.y = 0;
        speed.angular.z = 0;
        int ticks = int(linear_duration_x * rate);
        for (int i = 0; i < ticks; i++)
        {
            while (is_obstacle_)
            {
                ROS_INFO("There are obstacles around !");
                ros::Duration(0.1).sleep();
            }
            cmd_vel_pub_.publish(speed);
            loopRate.sleep(); // 200Hz，每秒发200次
        }
        cout << "y方向正在移动!" << endl;
        speed.linear.x = 0;
        speed.linear.y = flag * move_speed_.linear.y * (delta_y / abs(delta_y));
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
        ros::Duration(1.0).sleep(); // 上版为3s，最新为1s
    }
    else // 朝向为pi/2或者-pi/2的情况
    {
        int flag = 1;
        if (abs(current_pose.yaw - M_PI / 2) < 0.2)
            flag = -1;
        cout << "x方向正在移动!" << endl;
        speed.linear.x = flag * move_speed_.linear.x * (delta_y / abs(delta_y)) * (-1);
        speed.linear.y = 0;
        speed.angular.z = 0;
        int ticks = int(linear_duration_y * rate);
        for (int i = 0; i < ticks; i++)
        {
            while (is_obstacle_)
            {
                ROS_INFO("There are obstacles around !");
                ros::Duration(0.1).sleep();
            }
            cmd_vel_pub_.publish(speed);
            loopRate.sleep(); // 200Hz，每秒发200次
        }
        cout << "y方向正在移动!" << endl;
        speed.linear.x = 0;
        speed.linear.y = flag * move_speed_.linear.y * (delta_x / abs(delta_x));
        speed.angular.z = 0;
        ticks = int(linear_duration_x * rate);
        for (int i = 0; i < ticks; i++)
        {
            cmd_vel_pub_.publish(speed);
            loopRate.sleep();
        }

        speed.linear.x = 0;
        speed.linear.y = 0;
        speed.angular.z = 0;
        cmd_vel_pub_.publish(speed);
        ros::Duration(1.0).sleep(); // 上版为3s，最新为1s
    }
}

void Navigation::Rotate(Pose target_pose)
{
    cout << "正在进行旋转!" << endl;
    if (target_pose.yaw == 0 || target_pose.yaw == M_PI) // 朝向为0或者pi的情况
    {
        int flag1 = 1; // 用于处理yaw为0、pi处的正负
        int flag2 = 1; // 用于处理发布绕z轴旋转速度的正负
        Pose current_pose = GetPose();
        if (current_pose.yaw < 0)
            flag1 = -1;
        float delta_theta = flag1 * target_pose.yaw - current_pose.yaw;
        if (delta_theta < 0)
            flag2 = -1;
        geometry_msgs::Twist speed;
        speed.linear.x = 0;
        speed.linear.y = 0;
        speed.angular.z = move_speed_.angular.z * flag2;
        while (abs(delta_theta) > threshold_theta_)
        {
            speed.angular.z = move_speed_.angular.z * flag2;
            cmd_vel_pub_.publish(speed);
            ros::Duration(0.1).sleep(); // 上版为0.05s，最新为0.1s
            current_pose = GetPose();
            if (current_pose.yaw < 0)
                flag1 = -1;
            else
                flag1 = 1;
            delta_theta = flag1 * target_pose.yaw - current_pose.yaw;
            if (delta_theta < 0)
                flag2 = -1;
            else
                flag2 = 1;
        }
        speed.linear.x = 0;
        speed.linear.y = 0;
        speed.angular.z = 0;
        cmd_vel_pub_.publish(speed);
        ros::Duration(1.0).sleep(); // 上版为3s，最新为1s
    }
    else // 朝向为pi/2或者-pi/2的情况
    {
        int flag = 1;
        Pose current_pose = GetPose();
        float delta_theta = target_pose.yaw - current_pose.yaw;
        if (delta_theta < 0)
            flag = -1;
        geometry_msgs::Twist speed;
        speed.linear.x = 0;
        speed.linear.y = 0;
        speed.angular.z = flag * move_speed_.angular.z;
        while (abs(delta_theta) > threshold_theta_)
        {
            speed.angular.z = flag * move_speed_.angular.z;
            cmd_vel_pub_.publish(speed);
            ros::Duration(0.1).sleep(); // 上版为0.05s，最新为0.1s
            current_pose = GetPose();
            delta_theta = target_pose.yaw - current_pose.yaw;
            if (delta_theta < 0)
                flag = -1;
            else
                flag = 1;
        }
        speed.linear.x = 0;
        speed.linear.y = 0;
        speed.angular.z = 0;
        cmd_vel_pub_.publish(speed);
        ros::Duration(1.0).sleep(); // 上版为3s，最新为1s
    }
}

void Navigation::InitPose()
{
    Rotate(initial_pose_);
    Move(initial_pose_);
    CompensateError(initial_pose_);
    FeedbackToMes();
}

void Navigation::CompensateError(Pose target_pose)
{
    cout << "正在进行误差补偿!" << endl;
    int flag1 = 1;
    Pose current_pose = GetPose();
    double delta_x = target_pose.position.x - current_pose.position.x;
    double delta_y = target_pose.position.y - current_pose.position.y;
    if (target_pose.yaw == M_PI && current_pose.yaw < 0)
        flag1 = -1;
    double delta_theta = flag1 * target_pose.yaw - current_pose.yaw;

    while (abs(delta_x) > threshold_x_ || abs(delta_y) > threshold_y_ ||
           abs(delta_theta) > threshold_theta_)
    {
        Rotate(target_pose);
        Move(target_pose);
        current_pose = GetPose();
        delta_x = target_pose.position.x - current_pose.position.x;
        delta_y = target_pose.position.y - current_pose.position.y;
        if (target_pose.yaw == M_PI && current_pose.yaw < 0)
            flag1 = -1;
        else
            flag1 = 1;
        delta_theta = flag1 * target_pose.yaw - current_pose.yaw;
    }
    cout << "执行结果如下:" << endl;
    ROS_INFO_STREAM("delta_x:\t" << delta_x << endl);
    ROS_INFO_STREAM("delta_y:\t" << delta_y << endl);
    ROS_INFO_STREAM("delta_theta:\t" << delta_theta << endl);
}

void Navigation::FeedbackToMes()
{
    // 发布数字给mes（只发布一次）
    std_msgs::UInt8 msg;
    msg.data = 255;
    mes_msg_pub_.publish(msg);
    mes_msg_ = 0; // 在将信息反馈回mes后及时将该变量赋值为0，防止再次进入该处程序
}

void Navigation::MoveToIntermediatePoint(Pose target_pos)
{
    // [1] 在当前位置平移回中间线
    // [2] 如果目标点偏航角与当前不一致，则进行掉头，反之，则保持当前朝向。

    Pose current_pose = GetPose();
    Pose intermediate_pose;
    intermediate_pose.position.x = current_pose.position.x;
    intermediate_pose.position.y = 0;
    intermediate_pose.yaw = target_pos.yaw;
    MoveToTarget(intermediate_pose);
}

void Navigation::MoveToTurningPoint(Pose target_pos)
{
    Pose turning_pose;
    turning_pose.position.x = target_pos.position.x;
    turning_pose.position.y = 0;
    turning_pose.yaw = target_pos.yaw;
    MoveToTarget(turning_pose);
}

void Navigation::MoveToCompensationPoint(Pose target_pos)
{
    Pose current_pose = GetPose();
    Pose compensate_pose;
    compensate_pose.position.x = current_pose.position.x +
                                 (target_pos.position.x - current_pose.position.x) / 2; // 补偿点设为中间点
    compensate_pose.position.y = 0;
    compensate_pose.yaw = target_pos.yaw;
    MoveToTarget(compensate_pose);
}

void Navigation::MoveToTarget(Pose target_pos)
{
    Move(target_pos);
    CompensateError(target_pos);

    /*     // 重载==用于寻找下标
            int index;
            auto it = find(target_pose_.begin(), target_pose_.end(), target_pos);
            if (it != target_pose_.end())
            {
                index = distance(target_pose_.begin(), it);
            }
            if (index != 0 && index != 6)
                FeedbackToMes();  */
}

void Navigation::MoveToStation(Pose target_pos)
{
    MoveToIntermediatePoint(target_pos);
    Pose current_pose = GetPose();
    if (abs(target_pos.position.x - current_pose.position.x) >= threshold_distance_)
        MoveToCompensationPoint(target_pos);
    MoveToTurningPoint(target_pos);
    MoveToTarget(target_pos);
    FeedbackToMes();
}

void Navigation::MesCallback(const std_msgs::UInt8MultiArrayConstPtr &mes_msg)
{
    if (mes_msg->data[0] == 1)
        mes_msg_ = mes_msg->data[1];
}

void Navigation::Run()
{
    while (ros::ok())
    {

        switch (mes_msg_)
        {
        case 1:
            cout << "机器人正在向称量工位移动：" << endl;
            MoveToStation(target_pose_[1]);
            break;
        case 2:
            cout << "机器人正在向移液工位移动：" << endl;
            MoveToStation(target_pose_[2]);
            break;
        case 3:
            cout << "机器人正在向拧瓶工位移动：" << endl;
            MoveToStation(target_pose_[3]);
            break;
        case 4:
            cout << "机器人正在向**工位移动：" << endl;
            MoveToStation(target_pose_[4]);
            break;

        case 11:
            cout << "机器人进入充电桩：" << endl;
            MoveToCharge(charge_route_);
            break;
        case 12:
            cout << "机器人离开充电桩：" << endl;
            LeaveToOrigin(leave_route_);
            break;
        default:
            break;
        }
        ros::Duration(0.5).sleep();
    }
}

void Navigation::MoveToCharge(vector<Pose> charge_route)
{
    MoveToIntermediatePoint(charge_route[0]);
    
    for (int i = 0; i < charge_route.size(); i++)
    {
        MoveToTarget(charge_route[i]);
    }
    FeedbackToMes();
}


void Navigation::LeaveToOrigin(vector<Pose>leave_route)
{
    for (int i = 0; i < leave_route.size(); i++)
    {
        MoveToTarget(leave_route[i]);
    }
    FeedbackToMes();
}