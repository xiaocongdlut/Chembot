#include "navigation.h"

Navigation::Navigation()
{
    move_speed_.linear.x = 0.15;
    move_speed_.linear.y = 0.15;
    move_speed_.angular.z = 0.2;

    threshold_x_ = 0.03;
    threshold_y_ = 0.03;
    threshold_theta_ = 0.02;

    mes_msg_ = 0;

    threshold_distance_ = 3;

    // initial pose
    Pose target_pose_zero;
    target_pose_zero.position.x = 0;
    target_pose_zero.position.y = 0;
    target_pose_zero.yaw = 0;

    // target one 称量工位
    Pose target_pose_one;
    target_pose_one.position.x = 4.78;
    target_pose_one.position.y = 0.63;
    target_pose_one.yaw = 0;

    // target two 移液工位
    Pose target_pose_two;
    target_pose_two.position.x = -0.34;
    target_pose_two.position.y = 0.86;
    target_pose_two.yaw = 0;

    // target three 拧瓶工位
    Pose target_pose_three;
    target_pose_three.position.x = 1.65;
    target_pose_three.position.y = 0.78;
    target_pose_three.yaw = 0;

    // target four
    Pose target_pose_four;
    target_pose_four.position.x = 4.68;
    target_pose_four.position.y = -0.95;
    target_pose_four.yaw = M_PI;

    // target five
    Pose target_pose_five;
    target_pose_five.position.x = 4.625;
    target_pose_five.position.y = -0.84;
    target_pose_five.yaw = 0;

    target_pose_.push_back(target_pose_zero);
    target_pose_.push_back(target_pose_one);
    target_pose_.push_back(target_pose_two);
    target_pose_.push_back(target_pose_three);
    target_pose_.push_back(target_pose_four);
    target_pose_.push_back(target_pose_five);

    initial_pose_ = target_pose_zero;

    Pose route_point_zero;
    route_point_zero.position.x = 1;
    route_point_zero.position.y = 1;
    route_point_zero.yaw = -M_PI / 2;

    Pose route_point_one;
    route_point_one.position.x = 1;
    route_point_one.position.y = 1;
    route_point_one.yaw = -M_PI / 2;

    Pose route_point_two;
    route_point_two.position.x = 1;
    route_point_two.position.y = 1;
    route_point_two.yaw = -M_PI / 2;

    Pose route_point_three;
    route_point_three.position.x = 1;
    route_point_three.position.y = 1;
    route_point_three.yaw = -M_PI / 2;

    Pose route_point_four;
    route_point_four.position.x = 1;
    route_point_four.position.y = 1;
    route_point_four.yaw = 0;

    Pose route_point_five;
    route_point_five.position.x = 1;
    route_point_five.position.y = 1;
    route_point_five.yaw = 0;

    Pose route_point_six;
    route_point_six.position.x = 1;
    route_point_six.position.y = 1;
    route_point_six.yaw = 0;

    Pose route_point_seven;
    route_point_seven.position.x = 1;
    route_point_seven.position.y = 1;
    route_point_seven.yaw = 0;

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

    callbackThread_ = std::thread([&]()
                                  {ros::MultiThreadedSpinner spinner(2);spinner.spin(); });
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
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    pose_.yaw = yaw;
}

Pose Navigation::GetPose()
{
    return pose_;
}

void Navigation::Move(Pose target_pose)
{
    int rate = 200;
    ros::Rate loopRate(rate);
    Pose current_pose = GetPose();
    float delta_x = target_pose.position.x - current_pose.position.x;
    float delta_y = target_pose.position.y - current_pose.position.y;
    float linear_duration_x = abs(delta_x) / move_speed_.linear.x;
    float linear_duration_y = abs(delta_y) / move_speed_.linear.y;
    geometry_msgs::Twist speed;
    if (abs(current_pose.yaw - 0) < 0.2 || abs(abs(current_pose.yaw) - M_PI) < 0.2)
    {
        int flag = 1;
        if (abs(abs(current_pose.yaw) - M_PI) < 0.2)
            flag = -1;
        cout << "x方向正在移动!" << endl;
        speed.linear.x = flag * move_speed_.linear.x * (delta_x / abs(delta_x));
        speed.linear.y = 0;
        speed.angular.z = 0;
        int ticks = int(linear_duration_x * rate);
        for (int i = 0; i < ticks; i++)
        {
            cmd_vel_pub_.publish(speed);
            loopRate.sleep();
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
}

void Navigation::Rotate(Pose target_pose)
{
    cout << "正在进行旋转!" << endl;
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
        ros::Duration(0.1).sleep(); // 上版为0.01s，最新为0.05s
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
    if (current_pose.yaw < 0)
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
        if (current_pose.yaw < 0)
            flag1 = -1;
        else
            flag1 = 1;
        delta_theta = flag1 * target_pose.yaw - current_pose.yaw;
    }
}

void Navigation::FeedbackToMes()
{
    // 发布数字（只发布一次）
    std_msgs::UInt8 msg;
    msg.data = 254;
    mes_msg_pub_.publish(msg);
    mes_msg_ = 0;
}

void Navigation::MoveToMiddle(Pose target_pos)
{
    Pose current_pose = GetPose();
    Pose middle_pose;
    middle_pose.position.x = current_pose.position.x;
    middle_pose.position.y = 0;
    middle_pose.yaw = target_pos.yaw;
    MoveToTarget(middle_pose);
}

void Navigation::MoveToCompensatePoint(Pose target_pos)
{
    Pose current_pose = GetPose();
    Pose compensate_pose;
    compensate_pose.position.x += (target_pos.position.x - current_pose.position.x) / 2;
    compensate_pose.position.y = 0;
    compensate_pose.yaw = target_pos.yaw;
    MoveToTarget(compensate_pose);
}

void Navigation::MoveToTarget(Pose target_pos)
{
    Move(target_pos);
    CompensateError(target_pos);

/*     重载==用于寻找下标
    int index;
    auto it = find(target_pose_.begin(), target_pose_.end(), target_pos);
    if (it != target_pose_.end())
    {
        index = distance(target_pose_.begin(), it);
    }
    if (index != 0 && index != 6)
        FeedbackToMes(); */
}

void Navigation::MoveToStation(Pose target_pos)
{
    MoveToMiddle(target_pos);
    Pose current_pose = GetPose();
    if (abs(target_pos.position.x - current_pose.position.x) >= threshold_distance_)
        MoveToCompensatePoint(target_pos);
    // Pose current_pose = GetPose();
    Pose turn_pose;
    turn_pose.position.x = target_pos.position.x;
    turn_pose.position.y = 0;
    turn_pose.yaw = target_pos.yaw;
    MoveToTarget(turn_pose);
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
        default:
            break;
        }
        ros::Duration(0.5).sleep();
    }
}

void Navigation::MoveToCharge(vector<Pose> charge_route)
{
    for (int i = 0; i < charge_route.size(); i++)
    {
        MoveToTarget(charge_route[i]);
    }
}
