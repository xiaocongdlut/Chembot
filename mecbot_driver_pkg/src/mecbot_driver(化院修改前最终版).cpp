#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h> //部分工控机不带这个库文件，需要自己下载安装一下
#include <math.h>
#include <algorithm>
#include <mutex>

using namespace std;
serial::Serial ser; //声明串口对象

class chassis_driver
{
public:
    ros::NodeHandle n; //创建节点句柄
    ros::Subscriber speedSub;
    ros::Timer timer;
    float ticks_per_meter = 0.0;
    float wheel_separation = 0.0;
    float max_motor_speed = 0.0;
    float timeout = 0.0;
    // 线速度和角速度
    float linearVelocity_x = 0.0;  //Vx
    float linearVelocity_y = 0.0;  //Vy
    float angularVelocity = 0.0;  //Wz

    double last_time = -1.0;

    // bool enable_off = true;

    int32_t motor_1_initial_position = 0;
    int32_t motor_2_initial_position = 0;
    int32_t motor_3_initial_position = 0;
    int32_t motor_4_initial_position = 0;

    int32_t motor_1_position = 0;
    int32_t motor_2_position = 0;
    int32_t motor_3_position = 0;
    int32_t motor_4_position = 0;

    std::mutex velMtx; // 锁linearVelocity和angularVelocity
    chassis_driver()
    {
        //检测串口是否已经打开，并给出提示信息
        if (ser.isOpen())
        {
            ROS_INFO("chassis serial port initialized");

            /**进行电机初始化-需完善 **/
            
            //1. 去使能
            motor_enable_set(0x01, 0);
            motor_enable_set(0x02, 0);
            motor_enable_set(0x03, 0);
            motor_enable_set(0x04, 0);

            //2. 进速度模式
            motor_mode_set(0x01, 1);
            motor_mode_set(0x02, 1);
            motor_mode_set(0x03, 1);
            motor_mode_set(0x04, 1);
            
            //3. 设定速度为0
            motor_speed_set(0x01, 0);
            motor_speed_set(0x02, 0);
            motor_speed_set(0x03, 0);
            motor_speed_set(0x04, 0);
            
            //4. 上使能
            motor_enable_set(0x01, 1);
            motor_enable_set(0x02, 1);
            motor_enable_set(0x03, 1);
            motor_enable_set(0x04, 1);

            ROS_INFO("chassis setup OK !!!");
            ros::Duration(2).sleep();
       
       
       /**测试 **/
/*
          motor_speed_set(0x01, 100);
          ros::Duration(2).sleep();
          motor_speed_set(0x01, -100);
          ros::Duration(2).sleep();

          motor_speed_set(0x02, 500);
          ros::Duration(2).sleep();
          motor_speed_set(0x02, -500);
          ros::Duration(2).sleep();
*/
        }
        else
        {
            ROS_ERROR("chassis port is not open");
            ros::shutdown();
        }
        speedSub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &chassis_driver::speedCb, this);//*** 注释内容需明确 ***
        timer = n.createTimer(ros::Duration(0.02), &chassis_driver::control, this);//*** 注释内容需明确 ***
        readParam();
    }


        //【 1. 驱动函数 】

    /****************************************
    void readParam()
    函数功能：//*** 注释内容需明确 ***
    入口参数：//*** 注释内容需明确 ***
    出口参数：无
    ****************************************/
    void readParam()
    {
       // n.getParam("chassis_driver/ticks_per_meter", ticks_per_meter);
       // n.getParam("chassis_driver/wheelSeparation", wheel_separation);
       // n.getParam("chassis_driver/maxMotorSpeed", max_motor_speed);
       // n.getParam("chassis_driver/timeout", timeout);

        ticks_per_meter= 3082;       // 角速度 w= 60*N*v/(Pi*d) = 3082 v
        wheel_separation=0.629;    //左右轮中心距
        max_motor_speed=3000;
        timeout=1.0;

        //std::cout << "ticks_per_meter: " << ticks_per_meter << std::endl
        //<< "wheel_separation: " << wheel_separation << std::endl
        //<< "max_motor_speed: " << max_motor_speed << std::endl
        //<< "timeout: " << timeout << std::endl;
    }

    /****************************************
    void speedCb(const geometry_msgs::Twist::ConstPtr &msg)
    函数功能：//*** 注释内容需明确 ***
    入口参数：//*** 注释内容需明确 ***
    出口参数：无
    ****************************************/
    void speedCb(const geometry_msgs::Twist::ConstPtr &msg)
    {
        velMtx.lock();
        linearVelocity_x = msg->linear.x;
        linearVelocity_y = msg->linear.y;
        angularVelocity = msg->angular.z;
        last_time = ros::Time::now().toSec();
        velMtx.unlock();
    }

    /****************************************
    void getSpeeds_diff(float linear, float angular, int *l, int *r)
    函数功能：底盘运动学函数，根据线速度 Vx 和角速度 Wz ，计算左轮速度 Vl 和 右轮速度 Vr
    入口参数：线速度 Vx 和角速度 Wz ，指针：左轮速度 Vl 和 右轮速度 Vr
    出口参数：无
    ****************************************/
    void getSpeeds_diff(float linear, float angular, int *l, int *r)
    {
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

    void getSpeeds_mec(float vx, float vy,  float wz, int *vlf, int *vlb, int *vrf, int *vrb)
    {
        float r=0.1;     //  单个麦伦半径，单位：m
        float lx=0.3;             //   前后轮距离的一半，单位：m
        float ly=0.25;             //   左右轴距的一半，单位：m
        float k=477.7;//60/2pi*50
//        *vlb=((vx+vy)-wz*(lx+ly))/r*k;
 //       *vlf=((vx-vy)-wz*(lx+ly))/r*k;
  //      *vrf=((vx+vy)+wz*(lx+ly))/r*k;
    //    *vrb=((vx-vy)+wz*(lx+ly))/r*k;
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
    void controlMotor_diff()
    {
        int left, right;
        velMtx.lock();
        if (ros::Time::now().toSec() - last_time > timeout)
        {
            left = 0, right = 0;
        }
        else
        {
            getSpeeds_diff(linearVelocity_x, angularVelocity, &left, &right);
            if (std::max(abs(left), abs(right)) > max_motor_speed)
            {
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
            if (std::max(std::max(abs(l2), abs(l1)),std::max(abs(r3),abs(r4))) > max_motor_speed)
             {
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
    void control(const ros::TimerEvent &unused_timer_event)
    {
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
        uint8_t  receive_buff[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
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
        uint8_t  receive_buff[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
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
        uint8_t  receive_buff[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
        len_receive_buff = ser.read(receive_buff, len_receive_buff);//接收应答帧
        //ros::Duration(0.05).sleep();
    }
};

int main(int argc, char **argv)
{
    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/SERIAL1"); //串口
        ser.setBaudrate(115200);         //设置波特率
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("Unable to open the chassis port ");
        return -1;
    }
    ros::init(argc, argv, "chassis_driver");
    chassis_driver cd;
    ros::spin();
    return 0;
}
