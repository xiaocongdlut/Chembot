#include "navigation_0708.h"

Navigation::Navigation() // 构造函数
{
    // 底盘xy方向的移动速度 z轴的旋转速度
    move_speed_.linear.x = 0.15;
    move_speed_.linear.y = 0.15;
    move_speed_.angular.z = 0.10;
    // 角度和距离的阈值
    threshold_x_ = 0.03;
    threshold_y_ = 0.03;
    threshold_theta_ = 0.02;
    // 用于存储来自于mes的信息，决定底盘去哪个工位
    mes_msg_ = 0;
    // 目标点与当前点距离超过3，就在中间设置一个中间点
    threshold_distance_ = 3;

    Pose target_pose_zero; // initial pose
    target_pose_zero.position.x = 0;
    target_pose_zero.position.y = 0;
    target_pose_zero.yaw = 0;

    Pose target_pose_one;              // target one 称量工位
    target_pose_one.position.x = 4.76; // 4.80
    target_pose_one.position.y = 0.79; // 0.72
    target_pose_one.yaw = 0;

    Pose target_pose_two; // target two 移液工位
    target_pose_two.position.x = -0.43;
    target_pose_two.position.y = 0.92;
    target_pose_two.yaw = 0;

    Pose target_pose_three; // target three 旋盖机工位
    target_pose_three.position.x = 1.64;
    target_pose_three.position.y = 0.86;
    target_pose_three.yaw = 0;

    Pose target_pose_four; // target four 气质联用
    target_pose_four.position.x = 0.97;
    target_pose_four.position.y = -0.84;
    target_pose_four.yaw = M_PI;

    Pose target_pose_five; // target five 安捷伦
    target_pose_five.position.x = 3.58;
    target_pose_five.position.y = -0.86;
    target_pose_five.yaw = M_PI;

    Pose target_pose_six; // target six
    target_pose_six.position.x = 0;
    target_pose_six.position.y = 0;
    target_pose_six.yaw = 0;

    Pose target_pose_seven; // target seven 通风橱垃圾存储
    target_pose_seven.position.x = 5.73;
    target_pose_seven.position.y = 0.73;
    target_pose_seven.yaw = 0;

    Pose target_pose_eight; // target eight 物料仓
    target_pose_eight.position.x = 0.43;
    target_pose_eight.position.y = 0.89;
    target_pose_eight.yaw = 0;
    Pose target_pose_nine;              // target nine 通风橱
    target_pose_nine.position.x = 5.15; // 5.21
    target_pose_nine.position.y = 0.76;
    target_pose_nine.yaw = 0;

    Pose target_pose_ten; // target ten 液质联用
    target_pose_ten.position.x = 4.68;
    target_pose_ten.position.y = -0.86;
    target_pose_ten.yaw = M_PI;

    target_pose_.push_back(target_pose_zero);
    target_pose_.push_back(target_pose_one);
    target_pose_.push_back(target_pose_two);
    target_pose_.push_back(target_pose_three);
    target_pose_.push_back(target_pose_four);
    target_pose_.push_back(target_pose_five);
    target_pose_.push_back(target_pose_six);
    target_pose_.push_back(target_pose_seven);
    target_pose_.push_back(target_pose_eight);
    target_pose_.push_back(target_pose_nine);
    target_pose_.push_back(target_pose_ten);

    initial_pose_ = target_pose_zero;

    // 去充电桩的路线
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
    route_point_five.position.x = -1.42;
    route_point_five.position.y = -3.60;
    route_point_five.yaw = M_PI;

    Pose route_point_six;
    route_point_six.position.x = -1.42;
    route_point_six.position.y = -2.87;
    route_point_six.yaw = M_PI;

    Pose route_point_seven;
    route_point_seven.position.x = -1.825;
    route_point_seven.position.y = -2.87;
    route_point_seven.yaw = M_PI;
    // 离开充电桩回到初始点
    Pose leave_point_zero;
    leave_point_zero.position.x = -1.40;
    leave_point_zero.position.y = -2.87;
    leave_point_zero.yaw = M_PI;

    Pose leave_point_one;
    leave_point_one.position.x = -1.40;
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

    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);            // 定义/cmd_vel的话题
    mes_msg_pub_ = node_.advertise<std_msgs::UInt8>("/chessis_feedback_to_mes", 100); // 定义/chessis_feedback_to_mes话题

    mes_msg_sub_ = node_.subscribe("Mes_Command", 100, &Navigation::MesCallback, this); // 订阅mes部分的话题Mes_Command
    odom_sub_ = node_.subscribe("/odom_cat", 100, &Navigation::OdomCallback, this);     // 订阅odom部分的话题odom_cat

    callbackThread_ = std::thread([&]()
                                  {ros::MultiThreadedSpinner spinner(2);spinner.spin(); }); // 开两个线程，用来处理两个回调函数
}

Navigation::~Navigation()// 析构函数
{
    ros::shutdown(); // 关闭节点
}

void Navigation::OdomCallback(const geometry_msgs::PoseStamped &msg) // 回调函数 处理odom，转化成位置信息
{
    pose_.position.x = msg.pose.position.x;
    pose_.position.y = msg.pose.position.y;
    double qx = msg.pose.orientation.x;
    double qy = msg.pose.orientation.y;
    double qz = msg.pose.orientation.z;
    double qw = msg.pose.orientation.w;
    double roll, pitch, yaw;
    tf::Quaternion quat(qx, qy, qz, qw);          // 四元数
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 将四元数转化为偏航角yaw
    pose_.yaw = yaw;
}

Pose Navigation::GetPose() // 得到机器人当前位置
{
    return pose_; // 返回当前位姿
}

void Navigation::Move(Pose target_pose)// 移动，其中不包括位置误差处理
{
    int rate = 200;
    ros::Rate loopRate(rate); // 速度发布频率，哪个频率最合适需要尝试

    Pose current_pose = GetPose();
    // delta_x、delta_y的大小
    float delta_x = target_pose.position.x - current_pose.position.x;
    float delta_y = target_pose.position.y - current_pose.position.y;
    // 完成需要的时间
    float linear_duration_x = abs(delta_x) / move_speed_.linear.x;
    float linear_duration_y = abs(delta_y) / move_speed_.linear.y;

    geometry_msgs::Twist speed;
    if (abs(current_pose.yaw - 0) < 0.2 || abs(abs(current_pose.yaw) - M_PI) < 0.2) // 朝向为0或者pi的情况
    {
        int flag = 1; // 用于判断车的移动方向

        if (abs(abs(current_pose.yaw) - M_PI) < 0.2) //判断当前朝向是否在pi附近，如果在取flag为-1
            flag = -1;

        cout << "x方向正在移动!" << endl;
        speed.linear.x = flag * move_speed_.linear.x * (delta_x / abs(delta_x)); // 速度更新方式：1.flag 2.delta的正负，这两部分影响最后速度的正负，对于y方向同样如此
        speed.linear.y = 0;
        speed.angular.z = 0;
        int ticks = int(linear_duration_x * rate);
        for (int i = 0; i < ticks; i++)
        {
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
    else // 朝向为pi/2或者-pi/2的情况，与朝向为0或者pi的情况不同
    {
        int flag = 1;                               // 用于判断车的移动方向
        if (abs(current_pose.yaw - M_PI / 2) < 0.2) //判断当前朝向是否在pi/2附近，如果在取flag为-1
            flag = -1;
        cout << "x方向正在移动!" << endl;
        /*速度更新方式：1.flag 2.delta的正负 3.（-1）（特殊）（考虑实际情况）
        这三部分影响最后速度的正负*/
        speed.linear.x = flag * move_speed_.linear.x * (delta_y / abs(delta_y)) * (-1);
        speed.linear.y = 0;
        speed.angular.z = 0;
        int ticks = int(linear_duration_y * rate);
        for (int i = 0; i < ticks; i++)
        {
            cmd_vel_pub_.publish(speed);
            loopRate.sleep(); // 200Hz，每秒发200次
        }
        cout << "y方向正在移动!" << endl;
        speed.linear.x = 0;
        /*速度更新方式：1.flag 2.delta的正负
        这两部分影响最后速度的正负*/
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

void Navigation::AngleCorrection(double theta) //用于处理机器人在某个位置角度一直偏，采用旋转固定角度来修正
{
    int rate = 200;
    ros::Rate loopRate(rate);
    double delta_theta = theta;
    int flag = 1;
    if (delta_theta < 0)
        flag = -1;
    geometry_msgs::Twist speed;
    speed.linear.x = 0;
    speed.linear.y = 0;
    speed.angular.z = flag * 0.1;
    float time = abs(delta_theta) / 0.1;
    int ticks = int(time * rate);
    for (int i = 0; i < ticks; i++)
    {
        cmd_vel_pub_.publish(speed);
        loopRate.sleep();
    }
    speed.linear.x = 0;
    speed.linear.y = 0;
    speed.angular.z = 0;
    cmd_vel_pub_.publish(speed);
    ros::Duration(1.0).sleep();
}

void Navigation::Rotate(Pose target_pose)// 旋转，其中包括对角度误处理
{
    cout << "正在进行旋转!" << endl;
    if (target_pose.yaw == 0 || target_pose.yaw == M_PI) // 朝向为0或者pi的情况
    {
        int flag1 = 1; // 用于处理yaw为0、pi处的正负
        int flag2 = 1; // 用于处理发布绕z轴旋转速度的正负
        Pose current_pose = GetPose();

        /*针对于底盘在pi处存在的问题：比如当前为179度，转过去时就是+180度，如果处于-179度，转过去就是-180度
        因此要考虑当前所处于位置角度的正负问题*/
        if (current_pose.yaw < 0)
            flag1 = -1;
        /*举个例子：目标点为pi，当前点为-179度，如果不加这个flag1，那计算公式就为delta=pi-（-179）=359度，显然不对；
        加上以后，变成了delta=-pi-（-179）=-1度，这种显然是对的*/
        float delta_theta = flag1 * target_pose.yaw - current_pose.yaw;

        if (delta_theta < 0) // 这个flag则是根据delta的正负来处理旋转速度方向的正负
            flag2 = -1;
        geometry_msgs::Twist speed;
        speed.linear.x = 0;
        speed.linear.y = 0;
        /*进行一个变速，delta比较大的时候，旋转的速度是规定旋转速度的5倍；
        delta为30度的时候，旋转的速度变慢，变回规定的速度，用于减小惯性带来的误差*/
        if (abs(delta_theta) < M_PI / 6)
            speed.angular.z = flag2 * move_speed_.angular.z;
        else
            speed.angular.z = flag2 * move_speed_.angular.z * 5;

        while (abs(delta_theta) > threshold_theta_)// 用于处理角度误差，大于角度阈值，就一直旋转调整，直到满足要求为止
        {
            if (abs(delta_theta) < M_PI / 6)
                speed.angular.z = flag2 * move_speed_.angular.z;
            else
                speed.angular.z = flag2 * move_speed_.angular.z * 5;
            // speed.angular.z = move_speed_.angular.z * flag2;
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
        // 满足要求之后，让底盘停止
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
        if ((abs(abs(current_pose.yaw) - M_PI) < 0.2 && target_pose.yaw == M_PI / 2 && current_pose.yaw < 0) ||
            (abs(abs(current_pose.yaw) - M_PI) < 0.2 && target_pose.yaw == -M_PI / 2 && current_pose.yaw > 0))
            current_pose.yaw = -current_pose.yaw;

        float delta_theta = target_pose.yaw - current_pose.yaw;

        if (delta_theta < 0)
            flag = -1;
        geometry_msgs::Twist speed;
        speed.linear.x = 0;
        speed.linear.y = 0;
        if (abs(delta_theta) < M_PI / 6)
            speed.angular.z = flag * move_speed_.angular.z;
        else
            speed.angular.z = flag * move_speed_.angular.z * 5;
        while (abs(delta_theta) > threshold_theta_)
        {
            if (abs(delta_theta) < M_PI / 6)
                speed.angular.z = flag * move_speed_.angular.z;
            else
                speed.angular.z = flag * move_speed_.angular.z * 5;

            cmd_vel_pub_.publish(speed);
            ros::Duration(0.1).sleep(); // 上版为0.05s，最新为0.1s
            current_pose = GetPose();
            if ((abs(abs(current_pose.yaw) - M_PI) < 0.2 && target_pose.yaw == M_PI / 2 && current_pose.yaw < 0) ||
                (abs(abs(current_pose.yaw) - M_PI) < 0.2 && target_pose.yaw == -M_PI / 2 && current_pose.yaw > 0))
                current_pose.yaw = -current_pose.yaw;
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

void Navigation::InitPose()// 回到初始点
{
    Rotate(initial_pose_);// 旋转
    Move(initial_pose_);// 移动
    CompensateError(initial_pose_);// 处理位置和角度误差
    FeedbackToMes();// 反馈信息给mes
}

void Navigation::CompensateError(Pose target_pose)//误差补偿
{
    cout << "正在进行误差补偿!" << endl;
    int flag1 = 1;
    Pose current_pose = GetPose();




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

bool Navigation::CheckAMCL()//判断amcl是否开启
{
    std::string node_to_check = "amcl";// 指定要找的节点名字“amcl”
    ros::V_string nodes;
    ros::master::getNodes(nodes);// 获取所有的节点名
    for (const std::string &node : nodes)// 在所有节点中寻找amcl，然后找到了返回true，找不到返回false
    {
        if (node == "/" + node_to_check)
        {
            return true;
        }
    }
    return false;
}

void Navigation::FeedbackToMes()// 如果到达了指定点并满足要求，就向mes返回255
{
    // 发布数字给mes（只发布一次）
    std_msgs::UInt8 msg;
    msg.data = 255;
    mes_msg_pub_.publish(msg);
    mes_msg_ = 0; // 在将信息反馈回mes后及时将该变量赋值为0，防止再次进入该处程序
}

void Navigation::MoveToIntermediatePoint(Pose target_pos)// 回到中间线
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

void Navigation::MoveToTurningPoint(Pose target_pos)//移动到拐点，作用：在进入目标点的之间，先进行一次误差补偿，方便底盘稳定进入
{
    Pose turning_pose;
    turning_pose.position.x = target_pos.position.x;
    turning_pose.position.y = 0;
    turning_pose.yaw = target_pos.yaw;
    MoveToTarget(turning_pose);
}

void Navigation::MoveToCompensationPoint(Pose target_pos)// 距离过长时，移动到设置的中间点
{
    Pose current_pose = GetPose();
    Pose compensate_pose;
    compensate_pose.position.x = current_pose.position.x +
                                 (target_pos.position.x - current_pose.position.x) / 2; // 补偿点设为中间点
    compensate_pose.position.y = 0;
    // compensate_pose.yaw = target_pos.yaw;
    // MoveToTarget(compensate_pose);

    // zhuan 45du, zai zhuan hui qu
    compensate_pose.yaw = target_pos.yaw;
    MoveToTarget(compensate_pose);

    // compensate_pose.yaw = abs(target_pos.yaw)-M_PI/4;
    // Rotate(compensate_pose);

    // compensate_pose.yaw = target_pos.yaw;
    // Rotate(compensate_pose);
}

void Navigation::MoveToTarget(Pose target_pos)// 移动到指定的点，并且包含在指定点的误差处理
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

void Navigation::MoveToStation(Pose target_pos)// 移动到目标点，包含所有处理
{
    MoveToIntermediatePoint(target_pos);// 回到中间线
    Pose current_pose = GetPose();
    if (abs(target_pos.position.x - current_pose.position.x) >= threshold_distance_)// 判断是不是需要有中间补偿点
        MoveToCompensationPoint(target_pos);
    MoveToTurningPoint(target_pos);// 拐点
    MoveToTarget(target_pos);// 移动到目标点
    if (mes_msg_ == 7 || mes_msg_ == 9)// 这两个角度修正过程对应的是指定的那两个偏的位置，根据实际情况，选择不同的角度
    {
        AngleCorrection(0.15);
    }
    if (mes_msg_ == 1)
    {
        AngleCorrection(0.10);
    }
    FeedbackToMes();
}

void Navigation::MesCallback(const std_msgs::UInt8MultiArrayConstPtr &mes_msg)// 接受mes的反馈信息
{
    if (mes_msg->data[0] == 1)
        mes_msg_ = mes_msg->data[1];
}

void Navigation::Run()// 运动总程序，根据mes_msg_的值选择移动到指定的工位
{
    int count = 0;
    while (ros::ok())
    {

        switch (mes_msg_)
        {
        case 1:
            while (!CheckAMCL())// 在移动之前，判断amcl是否关闭
            {
                count++;
                cout << "restart number" << count << endl;
                system("rosrun amcl amcl &");// 如果amcl未开，就是用终端命令进行重启，知道amcl处于打开的状态
                ros::Duration(10.0).sleep();
            }
            count = 0;
            cout << "机器人正在向称量工位移动：" << endl;
            MoveToStation(target_pose_[1]);
            break;

        case 2:
            while (!CheckAMCL())
            {
                count++;
                cout << "restart number" << count << endl;
                system("rosrun amcl amcl &");
                ros::Duration(10.0).sleep();
            }
            count = 0;
            cout << "机器人正在向移液工位移动：" << endl;
            MoveToStation(target_pose_[2]);
            break;
        case 3:
            while (!CheckAMCL())
            {
                count++;
                cout << "restart number" << count << endl;
                system("rosrun amcl amcl &");
                ros::Duration(10.0).sleep();
            }
            count = 0;
            cout << "机器人正在向旋盖机工位移动：" << endl;
            MoveToStation(target_pose_[3]);
            break;
        case 4:
            while (!CheckAMCL())
            {
                count++;
                cout << "restart number" << count << endl;
                system("rosrun amcl amcl &");
                ros::Duration(10.0).sleep();
            }
            count = 0;
            cout << "机器人正在向气质联用工位移动：" << endl;
            MoveToStation(target_pose_[4]);
            break;

        case 5:
            while (!CheckAMCL())
            {
                count++;
                cout << "restart number" << count << endl;
                system("rosrun amcl amcl &");
                ros::Duration(10.0).sleep();
            }
            count = 0;
            cout << "机器人正在向 anjielun 工位移动：" << endl;
            MoveToStation(target_pose_[5]);
            break;

        case 6:
            while (!CheckAMCL())
            {
                count++;
                cout << "restart number" << count << endl;
                system("rosrun amcl amcl &");
                ros::Duration(10.0).sleep();
            }
            count = 0;
            cout << "机器人正在向   工位移动：" << endl;
            MoveToStation(target_pose_[6]);
            break;

        case 7:
            while (!CheckAMCL())
            {
                count++;
                cout << "restart number" << count << endl;
                system("rosrun amcl amcl &");
                ros::Duration(10.0).sleep();
            }
            count = 0;
            cout << "机器人正在向la ji cun fang 工位移动：" << endl;
            MoveToStation(target_pose_[7]);
            break;

        case 8:
            while (!CheckAMCL())
            {
                count++;
                cout << "restart number" << count << endl;
                system("rosrun amcl amcl &");
                ros::Duration(10.0).sleep();
            }
            count = 0;
            cout << "机器人正在向 cang 工位移动：" << endl;
            MoveToStation(target_pose_[8]);
            break;

        case 9:
            while (!CheckAMCL())
            {
                count++;
                cout << "restart number" << count << endl;
                system("rosrun amcl amcl &");
                ros::Duration(10.0).sleep();
            }
            count = 0;
            cout << "机器人正在通风橱移动：" << endl;
            MoveToStation(target_pose_[9]);
            break;
        case 10:
            while (!CheckAMCL())
            {
                count++;
                cout << "restart number" << count << endl;
                system("rosrun amcl amcl &");
                ros::Duration(10.0).sleep();
            }
            count = 0;
            cout << "机器人正在向液质联用工位移动：" << endl;
            MoveToStation(target_pose_[10]);
            break;
        case 11:
            while (!CheckAMCL())
            {
                count++;
                cout << "restart number" << count << endl;
                system("rosrun amcl amcl &");
                ros::Duration(10.0).sleep();
            }
            count = 0;
            cout << "机器人进入充电桩：" << endl;
            MoveToCharge(charge_route_);
            break;
        case 12:
            while (!CheckAMCL())
            {
                count++;
                cout << "restart number" << count << endl;
                system("rosrun amcl amcl &");
                ros::Duration(10.0).sleep();
            }
            count = 0;
            cout << "机器人离开充电桩：" << endl;
            LeaveToOrigin(leave_route_);
            break;
        case 13:
            while (!CheckAMCL())
            {
                count++;
                cout << "restart number" << count << endl;
                system("rosrun amcl amcl &");
                ros::Duration(10.0).sleep();
            }
            count = 0;
            cout << "hui ling dian:" << endl;
            MoveToStation(target_pose_[0]);
            break;
        default:
            break;
        }
        // MoveToStation(target_pose_[1]);
        // MoveToStation(target_pose_[2]);
        // MoveToStation(target_pose_[3]);
        // MoveToStation(target_pose_[5]);
        // MoveToStation(target_pose_[6]);
        ros::Duration(0.5).sleep();
    }
}

void Navigation::MoveToCharge(vector<Pose> charge_route)// 移动到充电桩
{
    MoveToIntermediatePoint(charge_route[0]);

    for (int i = 0; i < charge_route.size(); i++)// 循环充电路线点
    {
        if (i == (charge_route.size() - 2))
        {
            move_speed_.linear.x = 0.05;
            move_speed_.linear.y = 0.05;
            move_speed_.angular.z = 0.10;
        }
        MoveToTarget(charge_route[i]);
    }
    AngleCorrection(-0.12);//到达无线充电处，将地盘位置摆正，进行确定角度补偿
    move_speed_.linear.x = 0.15;
    move_speed_.linear.y = 0.15;
    move_speed_.angular.z = 0.10;
    FeedbackToMes();
}

void Navigation::LeaveToOrigin(vector<Pose> leave_route)// 离开充电桩，回到初始点
{
    for (int i = 0; i < leave_route.size(); i++)// 循环遍历路线上的点位
    {
        MoveToTarget(leave_route[i]);
    }
    FeedbackToMes();
}
