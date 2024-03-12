#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h> //部分工控机不带这个库文件，需要自己下载安装一下
#include <math.h>
#include <algorithm>
#include <mutex>
#include <stdio.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include <std_msgs/String.h>

using namespace std;
serial::Serial ser; //声明串口对象
/*
//Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pos_covariance_cmd_vel[36]   = {
                                             1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e-1 };

const double odom_twist_covariance_cmd_vel[36]   = {
                                             1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e-1 };

const double odom_pose_covariance_imu[36]   = {
                                            1e6,    0,    0,   0,   0,    0, 
										      0, 1e6,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e-1 };

const double odom_twist_covariance_imu[36]  = {
                                            1e6,    0,    0,   0,   0,    0, 
										      0, 1e6,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e-3,   0,    0,
										      0,    0,    0,   0, 1e-3,    0,
										      0,    0,    0,   0,   0,  1e-3 };
										      
double odom_x = 0.0;
double odom_y = 0.0;
double odom_theta = 0.0;
double odom_dt = 0.0;
double odom_vx=0.0;
double odom_vy=0.0;
double odom_vth=0.0;
ros::Time odom_last_time;

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    odom_vx = msg->linear.x;
    odom_vy = msg->linear.y;
    odom_vth = msg->angular.z;

    ros::Time odom_current_time = ros::Time::now();
    odom_dt = (odom_current_time - odom_last_time).toSec();

    double odom_delta_x = (odom_vx * cos(odom_theta) - odom_vy*sin(odom_theta))* odom_dt;
    double odom_delta_y = (odom_vx * sin(odom_theta) + odom_vy*cos(odom_theta))* odom_dt;
    double odom_delta_theta = odom_vth * odom_dt;

    odom_x += odom_delta_x;
    odom_y += odom_delta_y;
    odom_theta += odom_delta_theta;

    odom_last_time = odom_current_time;
}

void publishTransform()
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(odom_x, odom_y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, odom_theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
}

void publishOdometry(ros::Publisher& pub)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = odom_x;
    odom.pose.pose.position.y = odom_y;
    odom.pose.pose.position.z = 0.0;

    tf::Quaternion q;
    q.setRPY(0, 0, odom_theta);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    //There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
    //这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
    if(odom_vth!= 0){
              //If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
      //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance_imu, sizeof(odom_pose_covariance_imu));
      memcpy(&odom.twist.covariance, odom_twist_covariance_imu, sizeof(odom_twist_covariance_imu));       
   //ROS_INFO("ODOM USE IMU");
    }
    //odom_vx== 0&&odom_vy== 0&&

    else{
      //If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
      //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
      memcpy(&odom.pose.covariance, odom_pos_covariance_cmd_vel, sizeof(odom_pos_covariance_cmd_vel));
      memcpy(&odom.twist.covariance, odom_twist_covariance_cmd_vel, sizeof(odom_twist_covariance_cmd_vel));
   // ROS_INFO("ODOM USE cmd_vel");
    }
   
    pub.publish(odom);
}
*/
class chassis_driver{
    public:
        ros::NodeHandle n; //创建节点句柄
        ros::Subscriber speedSub;
        ros::Timer timer;

        float max_motor_speed=3000;
        float timeout=1.0;
        // 线速度和角速度
        float linearVelocity_x = 0.0;  //Vx，由speedCb函数赋值
        float linearVelocity_y = 0.0;  //Vy，由speedCb函数赋值
        float angularVelocity = 0.0;  //Wz，由speedCb函数赋值
        double last_time = -1.0;
        std::mutex velMtx; // 锁linearVelocity和angularVelocity

        /****************************************
        chassis_driver()
        函数功能：//构造函数，创建结构体时自动执行
        入口参数：无
        出口参数：无
        ****************************************/
        chassis_driver(){
            //1.检测串口是否已经打开，并给出提示信息
            if (ser.isOpen()){
                ROS_INFO("chassis serial port initialized");
                //2.进行电机初始化
                //2.1 去使能
                motor_enable_set(0x01, 0);
                motor_enable_set(0x02, 0);
                motor_enable_set(0x03, 0);
                motor_enable_set(0x04, 0);

                //2.2 进速度模式
                motor_mode_set(0x01, 1);
                motor_mode_set(0x02, 1);
                motor_mode_set(0x03, 1);
                motor_mode_set(0x04, 1);
            
                //2.3 设定速度为0
                motor_speed_set(0x01, 0);
                motor_speed_set(0x02, 0);
                motor_speed_set(0x03, 0);
                motor_speed_set(0x04, 0);
            
                //2.4上使能
                motor_enable_set(0x01, 1);
                motor_enable_set(0x02, 1);
                motor_enable_set(0x03, 1);
                motor_enable_set(0x04, 1);

                ROS_INFO("chassis setup OK !!!");
                ros::Duration(2).sleep();
            }
            else{
                ROS_ERROR("chassis port is not open");
                ros::shutdown();
            }
            //3.订阅名为/cmd_vel的topic，注册回调函数speedCb，用于进行运动学速度解算
            speedSub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &chassis_driver::speedCb, this);
            //4.回调control函数，用于机器人的实际控制
            timer = n.createTimer(ros::Duration(0.02), &chassis_driver::control, this);
        }

        /****************************************
        ~chassis_driver()
        函数功能：//析构函数，结束结构体时自动执行
        入口参数：无
        出口参数：无
        ****************************************/
        ~chassis_driver(){
            try{
                //1. 去使能
                motor_enable_set(0x01, 0);
                motor_enable_set(0x02, 0);
                motor_enable_set(0x03, 0);
                motor_enable_set(0x04, 0);
                //2. 去速度
                motor_speed_set(0x01, 0);
                motor_speed_set(0x02, 0);
                motor_speed_set(0x03, 0);
                motor_speed_set(0x04, 0);
            }
            catch (serial::IOException& e){
                ROS_ERROR_STREAM("Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
            }
            ser.close(); //Close the serial port //关闭串口  
            ROS_INFO_STREAM("Shutting down"); //Prompt message //提示信息
        }

        //【 1. 驱动函数 】
        /****************************************
        void speedCb(const geometry_msgs::Twist::ConstPtr &msg)
        函数功能：变量转换，将/cmd_vel中的geometry_msgs::Twist::ConstPtr 变量转成 float
        入口参数：/cmd_vel
        出口参数：无
        ****************************************/
        void speedCb(const geometry_msgs::Twist::ConstPtr &msg){
            velMtx.lock();
            linearVelocity_x = msg->linear.x;
            linearVelocity_y = msg->linear.y;
            angularVelocity = msg->angular.z;
            last_time = ros::Time::now().toSec();
            velMtx.unlock();
        }

        /****************************************
        void getSpeeds_diff(float linear, float angular, int *l, int *r)
        函数功能：差速底盘运动学函数，根据线速度 Vx 和角速度 Wz ，计算左轮速度 Vl 和 右轮速度 Vr
        入口参数：线速度 Vx 和角速度 Wz ，指针：左轮速度 Vl 和 右轮速度 Vr
        出口参数：无
        ****************************************/
        void getSpeeds_diff(float linear, float angular, int *l, int *r){
            float ticks_per_meter= 3082;     // 一圈多少个码，角速度 w= 60*N*v/(Pi*d) = 3082 v  (待完善)
            float wheel_separation=0.629;    //左右轮中心距，单位m

            float tickRate = linear * ticks_per_meter;
            float diffTicks = 1.0 * angular * wheel_separation * ticks_per_meter;
            *l = tickRate - diffTicks;
            *r = tickRate + diffTicks;
        }

        /****************************************
        void getSpeeds_mec(float linear, float angular, int *l, int *r)
        函数功能：底盘运动学函数，根据线速度 Vx  ,Vy和角速度 Wz ，计算左前轮速度 Vlf、左后轮速度 Vlb、右前轮速度 Vrf、右后轮速度 Vrb
        入口参数： Vx Vy Wz ，指针：Vlf Vlb Vrf Vrb
        出口参数：无
        ****************************************/
        /*
            2-3
            1-4
        */
        void getSpeeds_mec(float vx, float vy,  float wz, int *vlf, int *vlb, int *vrf, int *vrb){
            float r=0.1;                //单个麦伦半径，单位：m
            float lx=0.3;               //前后轮距离的一半，单位：m
            float ly=0.25;              //左右轴距的一半，单位：m
            float k=477.7;//60/2pi*50
            //*vlb=((vx+vy)-wz*(lx+ly))/r*k;
            //*vlf=((vx-vy)-wz*(lx+ly))/r*k;
            //*vrf=((vx+vy)+wz*(lx+ly))/r*k;
            //*vrb=((vx-vy)+wz*(lx+ly))/r*k;
            *vlb=((10*vx+10*vy)-wz*0.5)*k;
            *vlf=((10*vx-10*vy)-wz*0.5)*k;
            *vrf=((10*vx+10*vy)+wz*0.5)*k;
            *vrb=((10*vx-10*vy)+wz*0.5)*k;
        } 

        /****************************************
        void controlMotor_diff()
        函数功能：//*** 注释内容需明确 ***
        入口参数：//*** 注释内容需明确 ***
        出口参数：无
        ****************************************/
        void controlMotor_diff(){
            int left, right;
            velMtx.lock();
            if (ros::Time::now().toSec() - last_time > timeout){
                left = 0, right = 0;
            }
            else{
                getSpeeds_diff(linearVelocity_x, angularVelocity, &left, &right);
                if (std::max(abs(left), abs(right)) > max_motor_speed){
                    float factor = max_motor_speed / std::max(abs(left), abs(right));
                    left *= factor;
                    right *= factor;
                }
            }
            velMtx.unlock();
           
            motor_speed_set(0x01, left);
            motor_speed_set(0x02, -right);
            motor_speed_set(0x03, left);
            motor_speed_set(0x04, -right);

	        ROS_INFO("SET Motor Target Speed: %d , %d , %d , %d",left,-right,left,-right);
            ros::Duration(0.05).sleep();
        }
          
        /****************************************
        void controlMotor_mec()
        函数功能：//*** 注释内容需明确 ***
        入口参数：//*** 注释内容需明确 ***
        出口参数：无

        l2   r3
        l1   r4
        电机1为左后轮
        电机2为左前轮
        电机3为右前轮
        电机4为右后轮
        轮子为x型排布
        ****************************************/
        void controlMotor_mec(){
            int l1,l2, r3,r4;
            velMtx.lock();
            if (ros::Time::now().toSec() - last_time > timeout) {
                l1=0;
                l2=0;
                r3=0;
                r4=0;
            }
            else{
                getSpeeds_mec(linearVelocity_x, -linearVelocity_y,-angularVelocity, &l2, &l1, &r3, &r4);
                if (std::max(std::max(abs(l2), abs(l1)),std::max(abs(r3),abs(r4))) > max_motor_speed){
                    float factor = max_motor_speed /std::max(std::max(abs(l2), abs(l1)),std::max(abs(r3),abs(r4)));
                    l1 *= factor;
                    l2 *= factor;
                    r3 *= factor;
                    r4 *= factor;
                }
            }
            velMtx.unlock();

            if ((l1>-10)&&(l1<10)){
                motor_speed_set(0x01, 0);
                motor_speed_set(0x02, 0);
                motor_speed_set(0x03, 0);
                motor_speed_set(0x04, 0);
            }
            else if((l1>0)&&(-r3<0)){
                motor_speed_set(0x01, l1);
                motor_speed_set(0x02, l2);
                motor_speed_set(0x03, -r3-1);
                motor_speed_set(0x04, -r4-1);
            }
            else if((l1<0)&&(-r3>0)){
                motor_speed_set(0x01, l1-1);
                motor_speed_set(0x02, l2-1);
                motor_speed_set(0x03, -r3);
                motor_speed_set(0x04, -r4);
            }
            else{
                motor_speed_set(0x01, l1);
                motor_speed_set(0x02, l2);
                motor_speed_set(0x03, -r3);
                motor_speed_set(0x04, -r4);
            }
	        ROS_INFO("SET Motor Target Speed: %d , %d , %d , %d",l1,l2,-r3,-r4);
            ros::Duration(0.05).sleep();
        }

        /****************************************
        void control(const ros::TimerEvent &unused_timer_event)
        函数功能：//*** 注释内容需明确 ***
        入口参数：//*** 注释内容需明确 ***
        出口参数：无
        ****************************************/
        void control(const ros::TimerEvent &unused_timer_event){
            //controlMotor_diff();
            controlMotor_mec();
        }



    //【 2. 功能函数 】
    /****************************************
    void CRC_check(uint8_t* stream,int len)
    函数功能：用于进行目标数组的CRC校验
    入口参数：目标数组stream,数组长度 len
    出口参数：无

    CRC校验步骤：
    （1）预置1个16位的寄存器为十六进制FFFF（即全为1），称此寄存器为CRC寄存器；
    （2）把第一个8位二进制数据（既通讯信息帧的第一个字节）与16位的CRC寄存器的低8位相异或，把结果放于CRC寄存器，高八位数据不变；
    （3）把CRC寄存器的内容右移一位（朝低位）用0填补最高位，并检查右移后的移出位；
    （4）如果移出位为0：重复第3步（再次右移一位）；如果移出位为1，CRC寄存器与多项式（对于16位CRC：1010 0000 0000 0001）进行异或；
    （5）重复步骤3和4，直到右移8次，这样整个8位数据全部进行了处理；
    （6）重复步骤2到步骤5，进行通讯信息帧下一个字节的处理；
    （7）将该通讯信息帧所有字节按上述步骤计算完成后，得到的16位CRC寄存器的高、低字节进行交换；
    （8）最后得到的CRC寄存器内容即为：CRC码。
    以上计算步骤中的多项式A001（1010 0000 0000 0001）是8005按位颠倒后的结果。
    ****************************************/
    void CRC_check(uint8_t* stream,int len) {
        stream[len-2] = 0xff; //首先把CRC位设为ff，避免出错
        stream[len-1] = 0xff; //首先把CRC位设为ff，避免出错
        //ROS_INFO("CRC_len:%d",len);  //检验stream长度是否正确
        uint16_t CRC_HIGH = 0xFF; //CRC寄存器高八位设为ff
        uint8_t CRC_LOW = 0xFF;   //CRC寄存器低八位设为ff
        uint16_t CRC = (CRC_HIGH<<8)+CRC_LOW ;  //16位CRC寄存器根据高八位低八位叠加原理，此处值为：ffff
        for (int i = 0; i < len-2; i++){  //从stream首位开始，一直校验到CRC前一位
            CRC_LOW= stream[i] ^ CRC_LOW;   //将数据的第一个8-bit字符与16位CRC寄存器低八位进行异或，并把结果存入CRC寄存器
            CRC = (CRC_HIGH<<8)+CRC_LOW ;   //把结果存入CRC寄存器
            for (int j = 0; j < 8; j++)
            {
                uint8_t check = (CRC_LOW << 7);
                if (check == 0x80){
                    CRC = (CRC >> 1);
                    CRC = CRC ^ 0xa001;
                }
                else if (check == 0){
                    CRC=(CRC>>1);
                }
            CRC_HIGH = (CRC >> 8);
            CRC_LOW = CRC - (CRC_HIGH << 8);
            }
        }
        stream[len-2] = CRC_LOW;
        stream[len-1] = CRC_HIGH;
        //ROS_INFO("CRC:%x %x",CRC_LOW,CRC_HIGH);  //检验stream长度是否正确
    }

    /****************************************
    void motor_enable_set(uint8_t ID, int enable)
    函数功能：用于设定给定ID号电机的使能状态【0：去使能】【1：上使能】
    入口参数：电机ID号，使能命令
    出口参数：无
    用法：motor_mode_set(0x01, F);
    ****************************************/
    void motor_enable_set(uint8_t ID, int enable){
        uint8_t target_enable[] = {ID,0x06,0x00,0xc9,0x00,0x00,0xff,0xff};// 初始化电机模式切换数组，第0位为ID，1位为06写命令，2-3位为功能码3100，4-5位为模式hex值，6-7为CRC检验位
        if(enable== 1){
            target_enable[4]=0x00;
            target_enable[5]=0x01;
        }
        else{
            target_enable[4]=0x00;
            target_enable[5]=0x00;
        }
        CRC_check(target_enable, sizeof(target_enable) / sizeof(target_enable[0]));//进行CRC校验
        //for (int i = 0; i < sizeof(target_enable) / sizeof(target_enable[0]); i++) ROS_INFO("target_enable%d:%x",i,target_enable[i]); 
        ser.write(target_enable, sizeof(target_enable) / sizeof(uint8_t));
        ros::Duration(0.05).sleep();
        ROS_INFO("SET Motor %d Enable: %c",ID,enable);
        size_t len_receive_buff = ser.available();//获取应答帧长度
        uint8_t  receive_buff[len_receive_buff]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
        len_receive_buff = ser.read(receive_buff, len_receive_buff);//接收应答帧
        ros::Duration(0.05).sleep();
    }

    /****************************************
    void motor_mode_set(uint8_t ID, int mode)
    函数功能：用于设定给定ID号电机的操作模式【1：位置模式】【-3：立即速度】【3：速度模式】【4：力矩模式】
    入口参数：电机ID号，模式代号
    出口参数：无
    用法：motor_mode_set(0x01, -3);
    ****************************************/
    void motor_mode_set(uint8_t ID, int mode){
        uint8_t target_mode[] = {ID,0x06,0x00,0x64,0x00,0x00,0xff,0xff};// 初始化电机模式切换数组，第0位为ID，1位为06写命令，2-3位为功能码3500，4-5位为模式hex值，6-7为CRC检验位
        if(mode == 1){
            target_mode[4]=0x00;
            target_mode[5]=0x01;
        }
        else{
            target_mode[4]=0x00;
            target_mode[5]=0x00;
        }
        CRC_check(target_mode, sizeof(target_mode) / sizeof(target_mode[0]));//进行CRC校验
        //for (int i = 0; i < sizeof(target_mode) / sizeof(target_mode[0]); i++) ROS_INFO("target_mode%d:%x",i,target_mode[i]); 
        ser.write(target_mode, sizeof(target_mode) / sizeof(uint8_t));
        ros::Duration(0.05).sleep();
        ROS_INFO("SET Motor %d Mode: %d",ID,mode);
        size_t len_receive_buff = ser.available();//获取应答帧长度
        uint8_t  receive_buff[len_receive_buff]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
        len_receive_buff = ser.read(receive_buff, len_receive_buff);//接收应答帧
        ros::Duration(0.05).sleep();
    }

    /****************************************
    void motor_speed_set(uint8_t ID，uint64_t rpm)
    函数功能：用于设定给定ID号电机的rpm速度
    入口参数：电机ID号
    出口参数：无
    用法：motor_speed_set(0x01，500);
    ****************************************/
    void motor_speed_set(uint8_t ID, int64_t rpm){
        uint8_t target_speed[] = {ID,0x06,0x01,0x91,0x00,0x00,0xff,0xff};// 初始化电机目标速度数组，第0位为ID，1-5位为固定值，6-9位为速度hex值，10-11为CRC检验位

        target_speed[4]=rpm>>8;
        target_speed[5]=rpm-target_speed[4];

        CRC_check(target_speed, sizeof(target_speed) / sizeof(target_speed[0]));
        //for (int i = 0; i < sizeof(target_speed) / sizeof(target_speed[0]); i++) ROS_INFO("target_speed%d:%x",i,target_speed[i]); 
        ser.write(target_speed, sizeof(target_speed) / sizeof(uint8_t));
        ros::Duration(0.02).sleep();
        //ROS_INFO("SET Motor %d Target Speed: %d",ID,rpm);
        size_t len_receive_buff = ser.available();//获取应答帧长度
        if(len_receive_buff!=8){
            ROS_WARN("Motor %d len_receive_buff: %d",ID,len_receive_buff);
        }
        uint8_t  receive_buff[len_receive_buff]={0x00};
        len_receive_buff = ser.read(receive_buff, len_receive_buff);//接收应答帧
        //ros::Duration(0.05).sleep();
    }

    /****************************************
    int motor_speed_callback(uint8_t ID)
    函数功能：用于获取给定ID号电机的rpm速度
    入口参数：电机ID号
    出口参数：电机速度rpm
    用法：int motor_1_speed = motor_speed_callback(0x01);
    ****************************************/
    int motor_speed_callback_Actual(uint8_t ID){
	    uint8_t speed_callback[]={ID,0x03,0x05,0x16,0x00,0x01,0xff,0xff}; //根据ID，定义请求帧
	    CRC_check(speed_callback,sizeof(speed_callback) / sizeof(speed_callback[0])); //进行请求帧CRC校验
	    //for (int i = 0; i < sizeof(speed_callback) / sizeof(speed_callback[0]); i++) ROS_INFO("speed_callback%d:%x",i,speed_callback[i]); //输出并检验请求帧
	    ser.write(speed_callback, sizeof(speed_callback)/sizeof(uint8_t)); //发送请求帧
	    ros::Duration(0.02).sleep();//延迟
 	    size_t len_speed_receive = ser.available();//获取应答帧长度
	    //ROS_INFO("len_speed_receive:%d",len_speed_receive);//输出并检验应答帧长度
        int64_t rpm_motor=0;//定义电机速度反馈变量
	    uint8_t speed_receive[len_speed_receive]={};//根据应答帧长度建立应答帧接收数组
        if(len_speed_receive!=0){//应答帧长度不为0，则接收应答帧
			len_speed_receive = ser.read(speed_receive, len_speed_receive);//接收应答帧
			//for (int i = 0; i < sizeof(speed_receive) / sizeof(speed_receive[0]); i++) ROS_INFO("speed_receive%d:%x",i,speed_receive[i]); //输出并检验应答帧
			
            	//根据speed_receive数组，计算rpm_motor
            	rpm_motor=speed_receive[3]*256+speed_receive[4];
		if(rpm_motor>10000){
			rpm_motor=rpm_motor-65536;
		}
    		//ROS_INFO("Motor %d Speed Callback: %d",ID,rpm_motor);//输出并检验rpm
		}
	    //ros::Duration(0.05).sleep();
	    return rpm_motor;
    }   


    int motor_speed_callback_Target(uint8_t ID){
	    uint8_t speed_callback[]={ID,0x03,0x05,0x15,0x00,0x01,0xff,0xff}; //根据ID，定义请求帧
	    CRC_check(speed_callback,sizeof(speed_callback) / sizeof(speed_callback[0])); //进行请求帧CRC校验
	    //for (int i = 0; i < sizeof(speed_callback) / sizeof(speed_callback[0]); i++) ROS_INFO("speed_callback%d:%x",i,speed_callback[i]); //输出并检验请求帧
	    ser.write(speed_callback, sizeof(speed_callback)/sizeof(uint8_t)); //发送请求帧
	    ros::Duration(0.02).sleep();//延迟
 	    size_t len_speed_receive = ser.available();//获取应答帧长度
	    //ROS_INFO("len_speed_receive:%d",len_speed_receive);//输出并检验应答帧长度
        int64_t rpm_motor=0;//定义电机速度反馈变量
	    uint8_t speed_receive[len_speed_receive]={};//根据应答帧长度建立应答帧接收数组
        if(len_speed_receive!=0){//应答帧长度不为0，则接收应答帧
			len_speed_receive = ser.read(speed_receive, len_speed_receive);//接收应答帧
			//for (int i = 0; i < sizeof(speed_receive) / sizeof(speed_receive[0]); i++) ROS_INFO("speed_receive%d:%x",i,speed_receive[i]); //输出并检验应答帧
			
            	//根据speed_receive数组，计算rpm_motor
            	rpm_motor=speed_receive[3]*256+speed_receive[4];
		if(rpm_motor>10000){
			rpm_motor=rpm_motor-65536;
		}
    		//ROS_INFO("Motor %d Speed Callback: %d",ID,rpm_motor);//输出并检验rpm
		}
	    //ros::Duration(0.05).sleep();
	    return rpm_motor;
    }  

    void monitor_function(){
        int rpm_motor_A1=motor_speed_callback_Actual(0x01);
        int rpm_motor_A2=motor_speed_callback_Actual(0x02);
        int rpm_motor_A3=motor_speed_callback_Actual(0x03);
        int rpm_motor_A4=motor_speed_callback_Actual(0x04);
        ROS_INFO("--- Motor Actual Speed: %d , %d , %d , %d",rpm_motor_A1,rpm_motor_A2,rpm_motor_A3,rpm_motor_A4);
        int rpm_motor_T1=motor_speed_callback_Target(0x01);
        int rpm_motor_T2=motor_speed_callback_Target(0x02);
        int rpm_motor_T3=motor_speed_callback_Target(0x03);
        int rpm_motor_T4=motor_speed_callback_Target(0x04);
        ROS_INFO("--- Motor Target Speed: %d , %d , %d , %d",rpm_motor_T1,rpm_motor_T2,rpm_motor_T3,rpm_motor_T4);
    }

};

int main(int argc, char **argv){
    try{
        //设置串口属性，并打开串口
        ser.setPort("/dev/SERIAL1"); //串口chassis_485
        ser.setBaudrate(115200);     //设置波特率
        serial::Timeout to = serial::Timeout::simpleTimeout(3000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e){
        ROS_ERROR("Unable to open the chassis port ");
        return -1;
    }
    ros::init(argc, argv, "chassis_driver");

    chassis_driver mec_Robot;//创建机器人结构体

    ros::NodeHandle nh;
    //ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, twistCallback);
    //ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Publisher beat_pub = nh.advertise<std_msgs::String>("beat", 1000);//创建一个名为keyboard_cmd_pub的Publisher，发布名为keyboard_cmd的topic，消息类型为std_msgs::String，暂存队列长度为1000

   // odom_last_time = ros::Time::now();

    ros::Rate rate(20); // 10 Hz
    while (ros::ok())
    {
        //publishTransform();
        //publishOdometry(odom_pub);
        std_msgs::String beat;//初始化std_msgs::String类型的消息变量beat
        beat.data="ok";
        beat_pub.publish(beat);//发布消息cmd
        beat.data="";
        mec_Robot.monitor_function();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}