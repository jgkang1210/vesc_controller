// 2018.10.12 Created by Juhwan Kim, commands added by Seokyong Song
// 2018.10.15 Motions tested in Jangil bay
// 2019.01.21 PID control implemented
// 2019.01.30 Sensor data logging added, ID control added, Error standard < ~ 5 cm 5 deg, direction change: 1.6 s
// 2019.01.31 Heave Yaw control stop problem solved, Flipper collision solved, Deadzone implemented
// 2019.02.12 Mode change added
// 2019.02.25 One flipper test added

#include "hero_msgs/hero_flipper_state.h"
#include "hero_msgs/hero_flipper_state.h"
#include "ros/ros.h"                         // ROS Default Header File
#include "ros_tutorials_topic/MsgTutorial.h" // MsgTutorial Message File Header. The header file is automatically created when building the package.
#include "ros_opencv/hero_opencv.h"

#include <errno.h> /* ERROR Number Definitions          */
#include <fcntl.h> /* File Control Definitions          */
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions */
#include <cmath>

using namespace std;

#define CALI_1 458
#define CALI_2 494
#define CALI_3 515
#define CALI_4 488
#define CALI_5 357
#define CALI_6 577

#define PI 3.141592
#define DensitySeawater 1020
#define GravityConstant 9.80665

void set_motor_brake(int fd, unsigned char ID);
void set_motor_release(int fd, unsigned char ID);
void set_motor_pos(int fd, unsigned char ID, float degree);
void motor_flush(int fd);

float motor_cali(unsigned char ID, float degree);
void surge_forward(int fd);
void surge_backward(int fd);
void heave_down(int fd);
void heave_up(int fd);
void yaw_right(int fd);
void yaw_left(int fd);

void flipper_ready(int fd, float target_pos_left, float target_pos_right);

float motor_pos[6] = {0.0,};
int flipper_amplitude = 33;
int flipper_iter_times = 10;
float flipper_period = 0.75;
int reverse_direction = 0;

float Depth = 0;
float Roll = 0;
float Pitch = 0;
float Yaw = 0;

int ID_data = 0;
float Distance = 0;
float Tran[3] = {0,};
float Rotat[3] = {0,};

int LoggingFlag = 0;
int DeadzoneFlag = 1;
int Mode14Flag = 0;

ofstream SensorLog;
int ControlParam = 0;

ros::Time begin, current;
double secs_start, secs_finish, time_passed;

int open_serial(char *dev_name, int baud, int vtime, int vmin)
{
    int fd;
    struct termios newtio;
    // 시리얼포트를 연다.
    // fd = open(dev_name, O_RDWR | O_NOCTTY);
    fd = open(dev_name, O_WRONLY | O_NOCTTY);

    if (fd < 0)
    {
        // 화일 열기 실패
        printf("Device OPEN FAIL %s\n", dev_name);
        return -1;
    }
    // 시리얼 포트 환경을 설정한다.
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_iflag = IGNPAR; // non-parity
    // newtio.c_iflag = 0;
    newtio.c_oflag = 0;
    newtio.c_cflag = CS8 | CLOCAL | CREAD; // NO-rts/cts
    switch (baud)
    {
    case 115200:
        newtio.c_cflag |= B115200;
        break;
    case 57600:
        newtio.c_cflag |= B57600;
        break;
    case 38400:
        newtio.c_cflag |= B38400;
        break;
    case 19200:
        newtio.c_cflag |= B19200;
        break;
    case 9600:
        newtio.c_cflag |= B9600;
        break;
    case 4800:
        newtio.c_cflag |= B4800;
        break;
    case 2400:
        newtio.c_cflag |= B2400;
        break;
    default:
        newtio.c_cflag |= B115200;
        break;
    }
    // set input mode (non-canonical, no echo,.....)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = vtime; // timeout 0.1초 단위
    newtio.c_cc[VMIN] = vmin;   // 최소 n 문자 받을 때까진 대기
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    return fd;
}

void close_serial(int fd) { close(fd); }

void msgCallback_state(const hero_msgs::hero_flipper_state::ConstPtr &msg)
{
    Depth = ((msg->Depth)-1013) / (DensitySeawater*GravityConstant/100);
    Roll = (msg->Roll) * 180 / PI;
    Pitch = (msg->Pitch) * 180 / PI;
    Yaw = (msg->Yaw) * 180 / PI;

    if (LoggingFlag) {
        if (ControlParam == 2) {
            current = ros::Time::now();
            secs_finish = current.toSec();
            time_passed = secs_finish - secs_start;
            
            SensorLog << time_passed << "\t" << Depth << "\r\n";
        }
        else if (ControlParam == 3) {
            current = ros::Time::now();
            secs_finish = current.toSec();
            time_passed = secs_finish - secs_start;

            SensorLog << time_passed << "\t" << Yaw << "\r\n";
        } 
    }
    //ROS_INFO("Depth msg = %f", msg->Depth); // Prints the 'stamp.sec' message
    //ROS_INFO("Roll msg = %f", msg->Roll);   // Prints the 'stamp.nsec' message
    //ROS_INFO("Pitch msg = %f", msg->Pitch); // Prints the 'data' message
    //ROS_INFO("Yaw msg = %f", msg->Yaw);     // Prints the 'data' message
}

void msgCallback_opencv(const ros_opencv::hero_opencv::ConstPtr &msg)
{
    ID_data = msg->ID;
    Distance = msg->DISTANCE;
    Tran[0] = msg->TRAN[0];
    Tran[1] = msg->TRAN[1];
    Tran[2] = msg->TRAN[2];
    Rotat[0] = msg->ROTA[0];
    Rotat[1] = msg->ROTA[1];
    Rotat[2] = msg->ROTA[2];

    if (LoggingFlag) {
        if (ControlParam == 1) {
            current = ros::Time::now();
            secs_finish = current.toSec();
            time_passed = secs_finish - secs_start;

            SensorLog << time_passed << "\t" << Distance << "\r\n";
        }
    }
    //ROS_INFO("ID msg = %d", msg->ID); // Prints the 'stamp.sec' message
    //ROS_INFO("DISTANCE msg = %f", msg->DISTANCE);   // Prints the 'stamp.nsec' message
    //ROS_INFO("Tran msg = [%f %f %f]", msg->TRAN[0],msg->TRAN[1],msg->TRAN[2]); // Prints the 'data' message
    //ROS_INFO("Rotat msg = [%f %f %f]", msg->ROTA[0],msg->ROTA[1],msg->ROTA[2]);     // Prints the 'data' message
}

int main(int argc, char **argv) // Node Main Function
{
    cout << "Ver.2019.02.13 Start!" << endl;
    cout << "i o: amplitude 2 deg up&down" << endl;
    cout << "k l: iterative times up&down" << endl;
    cout << "[ ]: period 0.1 s up&down" << endl;
    cout << "8 9 0: surge(fwd) heave(down) yaw(right)" << endl;
    cout << "7: negative direction" << endl;
    cout << "= !: experiment commands" << endl;

    ros::init(argc, argv, "topic_publisher"); // Initializes Node Name
    ros::NodeHandle
        nh; // Node handle declaration for communication with ROS system
    ros::Rate loop_rate(100);   //Hz
    ros::Rate wait_rate(10);   //Hz
    ros::Rate ready_rate(1);   //Hz

    ros::Subscriber sub_state =
        nh.subscribe("/hero_flipper/state", 100, msgCallback_state);

        ros::Subscriber sub_ros_opencv =
        nh.subscribe("/ros_opencv_msg", 100, msgCallback_opencv);

    float degree = 0;
    int fd;           // 시리얼포트 파일핸들
    char cc, buf[50]; // 데이타 버퍼
    int rdcnt;

    char pc_input;

    // 시리얼 포트를 연다
    // 시리얼포트를 1초동안 대기하거나 32바이트 이상의 데이타가 들어오면
    // 깨어나도록 설정한다.
    fd = open_serial("/dev/ttyTHS1", 115200, 0, 0);
    if (fd < 0)
        return -2;

    while (ros::ok())
    {
        pc_input = getchar();

        if (pc_input == '/')
        {
            for (int i = 0; i < 6; i++){
                motor_pos[i] = 0.0;
                //set_motor_pos(fd, i+1, 0.0);
                
            }
            flipper_ready(fd, 0.0, 0.0);
        }
        /*
        else if(pc_input == '#')
        {
            if(Mode14Flag == 0){
                Mode14Flag = 1;
                cout << "Mode14 applied\r\n";
            }
            else {
                Mode14Flag = 0;
                cout << "Mode14Flag not applied\r\n";
            }
        }
        else if(pc_input == '$') // rotate motor 1, else 0
        {
            for (int i = 0; i < 6; i++){
                motor_pos[i] = 0.0;
            }
            flipper_ready(fd, 0.0, 0.0);
            set_motor_release(fd, 1);
            set_motor_release(fd, 3);
            set_motor_release(fd, 4);
            set_motor_release(fd, 5);
            set_motor_release(fd, 6);

            for (int j = 0; j < flipper_iter_times; j++){
                float deg = 0.0;

                for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
                    deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

                    motor_pos[1] = deg;
                    set_motor_pos(fd, 2, motor_pos[1]);
                    loop_rate.sleep();
                }
            }
        }
        else if(pc_input == '%') // rotate motor 1&4, else 0
        {
            for (int i = 0; i < 6; i++){
                motor_pos[i] = 0.0;                
            }
            flipper_ready(fd, 0.0, 0.0);
            set_motor_release(fd, 1);
            set_motor_release(fd, 3);
            set_motor_release(fd, 4);
            set_motor_release(fd, 6);

            for (int j = 0; j < flipper_iter_times; j++){
                float deg = 0.0;

                for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
                    deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

                    motor_pos[1] = deg;
                    motor_pos[4] = -1*deg;
                    set_motor_pos(fd, 2, motor_pos[1]);
                    set_motor_pos(fd, 5, motor_pos[4]);
                    loop_rate.sleep();
                }
            }
        }
        else if(pc_input == '^') // rotate motor 0,2,3,5, else 0
        {
            for (int i = 0; i < 6; i++){
                motor_pos[i] = 0.0;                
            }
            flipper_ready(fd, 0.0, 0.0);
            set_motor_release(fd, 2);
            set_motor_release(fd, 5);

            for (int j = 0; j < flipper_iter_times; j++){
                float deg = 0.0;
                
                for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
                    deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

                    motor_pos[0] = deg;
                    set_motor_pos(fd, 1, motor_pos[0]);
                    motor_pos[2] = -1*deg;
                    set_motor_pos(fd, 3, motor_pos[2]);
                    motor_pos[3] = deg;
                    set_motor_pos(fd, 4, motor_pos[3]);
                    motor_pos[5] = -1*deg;
                    set_motor_pos(fd, 6, motor_pos[5]);

                    loop_rate.sleep();
                }
            }            
        }
        else if(pc_input == '&') // rotate motor 0,2,3,5, else release
        {
            for (int i = 0; i < 6; i++){
                motor_pos[i] = 0.0;                
            }
            flipper_ready(fd, 0.0, 0.0);
            set_motor_release(fd, 2);
            set_motor_release(fd, 5);

            for (int j = 0; j < flipper_iter_times; j++){
                float deg = 0.0;
                
                for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
                    deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

                    motor_pos[0] = deg;
                    set_motor_pos(fd, 1, motor_pos[0]);
                    motor_pos[2] = -1*deg;
                    set_motor_pos(fd, 3, motor_pos[2]);
                    motor_pos[3] = deg;
                    set_motor_pos(fd, 4, motor_pos[3]);
                    motor_pos[5] = -1*deg;
                    set_motor_pos(fd, 6, motor_pos[5]);

                    loop_rate.sleep();
                }
            }
        }
        else if(pc_input == '*') // rotate motor 0,1,2,3,5
        {
            for (int i = 0; i < 6; i++){
                motor_pos[i] = 0.0;                
            }
            flipper_ready(fd, 0.0, 0.0);
            set_motor_release(fd, 5);

            for (int j = 0; j < flipper_iter_times; j++){
                float deg = 0.0;
                
                for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
                    deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

                    motor_pos[0] = deg;
                    set_motor_pos(fd, 1, motor_pos[0]);
                    motor_pos[1] = deg;
                    set_motor_pos(fd, 2, motor_pos[1]);
                    motor_pos[2] = -1*deg;
                    set_motor_pos(fd, 3, motor_pos[2]);
                    motor_pos[3] = deg;
                    set_motor_pos(fd, 4, motor_pos[3]);
                    motor_pos[5] = -1*deg;
                    set_motor_pos(fd, 6, motor_pos[5]);

                    loop_rate.sleep();
                }
            }
        }
        else if(pc_input == '(') // rotate motor 0,1,2,3,4,5
        {
            for (int i = 0; i < 6; i++){
                motor_pos[i] = 0.0;
            }
            flipper_ready(fd, 0.0, 0.0);

            for (int j = 0; j < flipper_iter_times; j++){
                float deg = 0.0;
                
                for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
                    deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

                    motor_pos[0] = deg;
                    set_motor_pos(fd, 1, motor_pos[0]);
                    motor_pos[1] = deg;
                    set_motor_pos(fd, 2, motor_pos[1]);
                    motor_pos[2] = -1*deg;
                    set_motor_pos(fd, 3, motor_pos[2]);
                    motor_pos[3] = deg;
                    set_motor_pos(fd, 4, motor_pos[3]);
                    motor_pos[4] = -1*deg;
                    set_motor_pos(fd, 5, motor_pos[4]);
                    motor_pos[5] = -1*deg;
                    set_motor_pos(fd, 6, motor_pos[5]);

                    loop_rate.sleep();
                }
            }
        }
        */

        else if(pc_input == 'q')
        {
            motor_pos[0]++;
            set_motor_pos(fd, 1, motor_pos[0]);
        }        
        else if(pc_input == 'w')
        {
            motor_pos[1]++;
            set_motor_pos(fd, 2, motor_pos[1]);
        }
        else if(pc_input == 'e')
        {
            motor_pos[2]++;
            set_motor_pos(fd, 3, motor_pos[2]);
        }
        else if(pc_input == 'r')
        {
            motor_pos[3]++;
            set_motor_pos(fd, 4, motor_pos[3]);
        }
        else if(pc_input == 't')
        {
            motor_pos[4]++;
            set_motor_pos(fd, 5, motor_pos[4]);
        }
        else if (pc_input == 'y')
        {
            motor_pos[5]++;
            set_motor_pos(fd, 6, motor_pos[5]);
        }
        else if(pc_input == 'a')
        {
            motor_pos[0]--;
            set_motor_pos(fd, 1, motor_pos[0]);
        }
        else if(pc_input == 's')
        {
            motor_pos[1]--;
            set_motor_pos(fd, 2, motor_pos[1]);
        }
        else if(pc_input == 'd')
        {
            motor_pos[2]--;
            set_motor_pos(fd, 3, motor_pos[2]);
        }
        else if(pc_input == 'f')
        {
            motor_pos[3]--;
            set_motor_pos(fd, 4, motor_pos[3]);
        }
        else if(pc_input == 'g')
        {
            motor_pos[4]--;
            set_motor_pos(fd, 5, motor_pos[4]);
        }
        else if (pc_input =='h')
        {
            motor_pos[5]--;
            set_motor_pos(fd, 6, motor_pos[5]);
        }
        else if(pc_input == 'z')
        {
            set_motor_release(fd, 1);
        }
        else if(pc_input == 'x')
        {
            set_motor_release(fd, 2);
        }
        else if(pc_input == 'c')
        {
            set_motor_release(fd, 3);
        }
        else if(pc_input == 'v')
        {
            set_motor_release(fd, 4);
        }
        else if(pc_input == 'b')
        {
            set_motor_release(fd, 5);
        }
        else if(pc_input == 'n')
        {
            set_motor_release(fd, 6);
        }
        else if (pc_input == 'm')
        {
            for (int i = 0; i < 6; i++)
            {
                set_motor_release(fd, i+1);
            }
        }
        else if(pc_input == 'i')
        {
            flipper_amplitude += 2;
        }
        else if(pc_input == 'o')
        {
            flipper_amplitude -= 2;
        }
        else if(pc_input == 'k')
        {
            flipper_iter_times += 1;
            //cont_on = 1;
            //target_motor_pos[0] = 45;
        }
        else if(pc_input == 'l')
        {
            flipper_iter_times -= 1;
            //cont_on = 0;
            //target_motor_pos[0] = 315;
        }
        else if(pc_input =='[')
        {
            flipper_period += 0.1;
        }
        else if(pc_input ==']')
        {
            flipper_period -= 0.1;
        }
        
        // One flipper test
        else if(pc_input == 'Q')
        {
            for (int j = 0; j < 10; j++){
                    float deg = 0.0;

                    for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
                         deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

                        motor_pos[0] = -1*deg;
                        set_motor_pos(fd, 1, motor_pos[0]);
                        loop_rate.sleep();
                    }
            }
        }
        else if(pc_input == 'W')
        {
            for (int j = 0; j < 10; j++){
                    float deg = 0.0;

                    for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
                         deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

                        motor_pos[1] = -1*deg;
                        set_motor_pos(fd, 2, motor_pos[1]);
                        loop_rate.sleep();
                    }
            }
        }
        else if(pc_input == 'E')
        {
            for (int j = 0; j < 10; j++){
                    float deg = 0.0;

                    for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
                         deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

                        motor_pos[2] = -1*deg;
                        set_motor_pos(fd, 3, motor_pos[2]);
                        loop_rate.sleep();
                    }
            }
        }
        else if(pc_input == 'R')
        {
            for (int j = 0; j < 10; j++){
                    float deg = 0.0;

                    for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
                         deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

                        motor_pos[3] = deg;
                        set_motor_pos(fd, 4, motor_pos[3]);
                        loop_rate.sleep();
                    }
            }
        }
        else if(pc_input == 'T')
        {
            for (int j = 0; j < 10; j++){
                    float deg = 0.0;

                    for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
                         deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

                        motor_pos[4] = deg;
                        set_motor_pos(fd, 5, motor_pos[4]);
                        loop_rate.sleep();
                    }
            }
        }
        else if(pc_input == 'Y')
        {
            for (int j = 0; j < 10; j++){
                    float deg = 0.0;

                    for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
                         deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

                        motor_pos[5] = deg;
                        set_motor_pos(fd, 6, motor_pos[5]);
                        loop_rate.sleep();
                    }
            }
        }

        // Surge, Heave, Yaw test
        else if(pc_input == '7')
        {
            if(reverse_direction == 0){
                reverse_direction = 1;
            }
            else {
                reverse_direction = 0;
            }
        }
        else if (pc_input == '8')
        {
            if (reverse_direction == 0){
                flipper_ready(fd, 0.0, 0.0);
                surge_forward(fd);
            }
            else {
                flipper_ready(fd, 180.0, 180.0);
                surge_backward(fd);	
            }
        }	  
	    else if (pc_input == '9')
        {		  
            if (reverse_direction == 0){
                flipper_ready(fd, 270.0, 90.0);
                heave_down(fd);
            }
            else {
                flipper_ready(fd, 90.0, 270.0);
                heave_up(fd);
            }        
        }	  
        else if (pc_input == '0')
        {
            if (reverse_direction == 0){
                flipper_ready(fd, 180.0, 0.0);
                yaw_right(fd);
            }
            else {
                flipper_ready(fd, 0.0, 180.0);
                yaw_left(fd);	
            }   	  
        }
        else if(pc_input=='.')
        {
            ros::spinOnce();
        }
        else if(pc_input=='=')
        {
            int DisplayCounts = 0;
            while(DisplayCounts == 10){
                 printf("Dist: %.2f, Depth: %.2f, Roll: %.2f , Pitch: %.2f , Yaw: %.2f\r\n", Distance, Depth, Roll, Pitch, Yaw);
                 ros::spinOnce();

                 wait_rate.sleep();

                 DisplayCounts++;                 
            }                   
           
        }
        else if(pc_input=='@')
        {
            if(DeadzoneFlag == 0){
                DeadzoneFlag = 1;
                cout << "Dead zone applied normally\r\n";
            }
            else {
                DeadzoneFlag = 0;
                cout << "Dead zone not applied normally\r\n";
            }
        }
        else if(pc_input=='!') // PID loop
        {
            ControlParam = 0;
            float TargetParam = 0.0;
            float errArray[10] = {};
            float errSum = 0.0; float errAbsSum = 0.0; float errDiff = 0.0;
            float ForceOutput = 0.0;
            float ForceOutput_last = 0.0;
            float K_p = 0.0; float K_i = 0.0; float K_d = 0.0;
            float A = 1.0342 * flipper_amplitude - 14.819; // constant used in (Force -> Period) function for current flippers
            float B = 0.0209 * flipper_amplitude - 3.0456; // constant used in (Force -> Period) function for current flippers
            int PIDContinue = 1;
            int PIDcount = 0;
            flipper_iter_times = 1;
            char Logname[100];
            int NameIndex;
            char *DeadzoneStr;

            printf("Dist: %.2f, Depth: %.2f, Roll: %.2f , Pitch: %.2f , Yaw: %.2f\r\n", Distance, Depth, Roll, Pitch, Yaw);
            ros::spinOnce();
                
            cout << "Parameter (Distance=1, Heave=2, Yaw=3): ";
            cin >> ControlParam;

            cout << "Target Parameter: ";
            cin >> TargetParam;

            cout << "Input K_p(Proportional gain) K_i(Integral gain) K_d(derivative gain): ";
            cin >>  K_p >> K_i >> K_d;

            if (DeadzoneFlag)
                DeadzoneStr = "_dead.txt";
            else
                DeadzoneStr = "_nondead.txt";               
                
            if (ControlParam == 1){
                NameIndex = sprintf(Logname, "/home/nvidia/catkin_ws/surge_tar%d_P%d_I%d_D%d_A%d", (int)(TargetParam*100), (int)K_p, (int)K_i, (int)K_d, flipper_amplitude);
                if (Distance-TargetParam >= 0)
                    flipper_ready(fd, 0.0, 0.0);
                else
                    flipper_ready(fd, 180.0, 180.0);
            }                
            else if (ControlParam == 2){
                NameIndex = sprintf(Logname, "/home/nvidia/catkin_ws/heave_tar%d_P%d_I%d_D%d_A%d", (int)(TargetParam*100), (int)K_p, (int)K_i, (int)K_d, flipper_amplitude);
                if (TargetParam-Depth >= 0)
                    flipper_ready(fd, 270.0, 90.0);
                else
                    flipper_ready(fd, 90.0, 270.0);
            }
            else if (ControlParam == 3){
                NameIndex = sprintf(Logname, "/home/nvidia/catkin_ws/yaw_tar%d_P%d_I%d_D%d_A%d", (int)TargetParam, (int)K_p, (int)K_i, (int)K_d, flipper_amplitude);
                if (TargetParam-Yaw >= 0)
                    flipper_ready(fd, 0.0, 180.0);
                else
                    flipper_ready(fd, 180.0, 0.0);
            }
            NameIndex += sprintf(Logname+NameIndex, "%s", DeadzoneStr);

            SensorLog.open(Logname);
            ready_rate.sleep();
            begin = ros::Time::now();
            secs_start = begin.toSec();

            while(PIDContinue){
                
                printf("Dist: %.2f, Depth: %.2f, Roll: %.2f , Pitch: %.2f , Yaw: %.2f\r\n", Distance, Depth, Roll, Pitch, Yaw);
                ros::spinOnce();
                LoggingFlag = 1;

                switch (ControlParam){
                    case 1: // Surge control with distance data
                        for (int i=0; i<9; i++){
                            errArray[i] = errArray[i+1];
                            errSum += errArray[i+1];
                            errAbsSum += abs(errArray[i+1]);
                        }
                        errArray[9] = Distance - TargetParam;
                        errSum += errArray[9];
                        errAbsSum += abs(errArray[9]);
                        errDiff = errArray[9] - errArray[8]; 

                        if ((DeadzoneFlag==0) || (abs(errArray[9])>0.05)){
                            ForceOutput = K_p*errArray[9] + K_i*errSum + K_d*errDiff;

                            if (ForceOutput >= 0){
                                if (ForceOutput > 6*A*pow(0.5, B))
                                    flipper_period = 0.5;
                                //else if (ForceOutput < 6*A*pow(2.0, B))
                                //    flipper_period = 2.0;                            
                                else
                                    flipper_period = pow((ForceOutput / (6*A)), (1/B));
                                
                                if (ForceOutput*ForceOutput_last < 0)
                                    flipper_ready(fd, 0.0, 0.0);
                                    
                                surge_forward(fd);
                            }

                            // Amplitude is determined by user's command.                        
                            else {
                                if (ForceOutput < (-6)*A*pow(0.5, B))
                                    flipper_period = 0.5;
                                //else if (ForceOutput > (-6)*A*pow(2.0, B))
                                //    flipper_period = 2.0;
                                else
                                    flipper_period = pow(((-1)*ForceOutput / (6*A)), (1/B));
                                
                                if (ForceOutput*ForceOutput_last < 0)
                                    flipper_ready(fd, 180.0, 180.0);
                                
                                surge_backward(fd);
                            }
                            ForceOutput_last = ForceOutput;
                        }
                        else {
                            for (int i = 0; i < 6; i++)
                                set_motor_release(fd, i+1);

                            for (int j = 0; j < 5; j++)
                                wait_rate.sleep();  
                        }
                        
                        PIDcount++;                        
                        if ((errAbsSum/10 < 0.05) && (PIDcount > 15) && (abs(errArray[9])<0.05))
                            PIDContinue = 0;                       
                        
                        errSum = 0;
                        errAbsSum = 0;

                        break;

                    case 2: // Heave control with depth data
                        for (int i=0; i<9; i++){
                            errArray[i] = errArray[i+1];
                            errSum += errArray[i+1];
                            errAbsSum += abs(errArray[i+1]);                            
                        }
                        errArray[9] = TargetParam - Depth;
                        errSum += errArray[9];
                        errAbsSum += abs(errArray[9]);
                        errDiff = errArray[9] - errArray[8];

                        if ((DeadzoneFlag==0) || (abs(errArray[9])>0.05)){
                            ForceOutput = K_p*errArray[9] + K_i*errSum + K_d*errDiff;     

                            // Amplitude is determined by user's command.
                            if (ForceOutput >= 0){
                                if (ForceOutput > 6*A*pow(0.5, B))
                                    flipper_period = 0.5;
                                //else if (ForceOutput < 6*A*pow(2.0, B))
                                //    flipper_period = 2.0;      
                                else
                                    flipper_period = pow((ForceOutput / (6*A)), (1/B));

                                if (ForceOutput*ForceOutput_last < 0)
                                    flipper_ready(fd, 270.0, 90.0);

                                heave_down(fd);
                            }
                            else {
                                if (ForceOutput < (-6)*A*pow(0.5, B))
                                    flipper_period = 0.5;
                                //else if (ForceOutput > (-6)*A*pow(2.0, B))
                                //    flipper_period = 2.0;
                                else
                                    flipper_period = pow(((-1)*ForceOutput / (6*A)), (1/B));

                                if (ForceOutput*ForceOutput_last < 0)
                                    flipper_ready(fd, 90.0, 270.0);
                                    
                                heave_up(fd);
                            }
                            ForceOutput_last = ForceOutput;
                        }
                        else {
                            for (int i = 0; i < 6; i++)
                                set_motor_release(fd, i+1);

                            for (int j = 0; j < 5; j++)
                                wait_rate.sleep(); 
                        }

                        PIDcount++;                        
                        if ((errAbsSum/10 < 0.05) && (PIDcount > 15) && (abs(errArray[9])<0.05))
                            PIDContinue = 0;
                        
                        errSum = 0;
                        errAbsSum = 0;   

                        break;
                    
                    case 3: // Yaw control with IMU data
                        for (int i=0; i<9; i++){
                            errArray[i] = errArray[i+1];
                            errSum += errArray[i+1];
                            errAbsSum += abs(errArray[i+1]);                          
                        }
                        errArray[9] = TargetParam - Yaw;
                        errSum += errArray[9];
                        errAbsSum += abs(errArray[9]);
                        errDiff = errArray[9] - errArray[8];

                        if ((DeadzoneFlag==0) || (abs(errArray[9])>5)){
                            ForceOutput = K_p*errArray[9] + K_i*errSum + K_d*errDiff;     

                            // Amplitude is determined by user's command.
                            if (ForceOutput >= 0){
                                if (ForceOutput > 6*A*pow(0.5, B))
                                    flipper_period = 0.5;
                                //else if (ForceOutput < 6*A*pow(2.0, B))
                                //    flipper_period = 2.0;  
                                else
                                    flipper_period = pow((ForceOutput / (6*A)), (1/B));

                                if (ForceOutput*ForceOutput_last < 0)
                                    flipper_ready(fd, 0.0, 180.0);
                            
                                yaw_left(fd); // error: opposite
                            }
                            else {
                                if (ForceOutput < (-6)*A*pow(0.5, B))
                                    flipper_period = 0.5;
                                //else if (ForceOutput > (-6)*A*pow(2.0, B))
                                //    flipper_period = 2.0;
                                else
                                    flipper_period = pow(((-1)*ForceOutput / (6*A)), (1/B));

                                if (ForceOutput*ForceOutput_last < 0)
                                    flipper_ready(fd, 180.0, 0.0);

                                yaw_right(fd); // error: opposite
                            }
                            ForceOutput_last = ForceOutput;
                        }
                        else {
                            for (int i = 0; i < 6; i++)
                                set_motor_release(fd, i+1);
                                
                            for (int j = 0; j < 5; j++)
                                wait_rate.sleep(); 
                        }
                        
                        PIDcount++;                        
                        if ((errAbsSum/10 < 5) && (PIDcount > 15))
                            PIDContinue = 0; 
                                                
                        errSum = 0;
                        errAbsSum = 0;

                        break;
                } // Switch (ControlParam) statement
                printf("err: %.2f, force output: %.2f, period: %.2f\r\n", errArray[9], ForceOutput, flipper_period);

            } // while (PIDConinue) statement
            //ros::spinOnce();
            for (int i = 0; i < 6; i++)
                set_motor_release(fd, i+1);

            LoggingFlag = 0;
            SensorLog.close();
        } // else if(pc_input=='!') statement

        printf("amplitude: %d, %d times, period: %.2f\r\n",flipper_amplitude,flipper_iter_times,flipper_period);
        printf("%.1f %.1f %.1f %.1f %.1f %.1f\r\n", motor_pos[0], motor_pos[1], motor_pos[2], motor_pos[3], motor_pos[4], motor_pos[5]);
        printf("direction config: %d\r\n", reverse_direction);
        
    }
    ros::spin(); // Increase count variable by one
    close_serial(fd);
    return 0;
}

void set_motor_pos(int fd, unsigned char ID, float degree)
{
    degree = motor_cali(ID, degree);
    unsigned char outbuff[8];
    union {
        float real;
        uint32_t base;
    } u_Pos;
    u_Pos.real = degree;
    outbuff[4] = (u_Pos.base >> (8 * 0)) & 0xFF;
    outbuff[5] = (u_Pos.base >> (8 * 1)) & 0xFF;
    outbuff[6] = (u_Pos.base >> (8 * 2)) & 0xFF;
    outbuff[7] = (u_Pos.base >> (8 * 3)) & 0xFF;

    outbuff[0] = 0xff;
    outbuff[1] = 0xff;
    outbuff[2] = ID;
    outbuff[3] = 0xfe;

    write(fd, outbuff, 8);
}
void set_motor_release(int fd, unsigned char ID)
{
    unsigned char outbuff[8];
    outbuff[0] = 0xff;
    outbuff[1] = 0xff;
    outbuff[2] = ID;
    outbuff[3] = 203;
    outbuff[4] = 0xfe;
    outbuff[5] = 0xfe;
    outbuff[6] = 0xfe;
    outbuff[7] = 0xfe;
    write(fd, outbuff, 8);
}
void motor_flush(int fd)
{
    unsigned char outbuff[8];
    outbuff[0] = 0xfc;
    outbuff[1] = 0xfc;
    outbuff[2] = 0xfc;
    outbuff[3] = 0xfc;
    outbuff[4] = 0xfc;
    outbuff[5] = 0xfc;
    outbuff[6] = 0xfc;
    outbuff[7] = 0xfc;
    write(fd, outbuff, 8);
}
void set_motor_brake(int fd, unsigned char ID)
{
    unsigned char outbuff[8];
    outbuff[0] = 0xff;
    outbuff[1] = 0xff;
    outbuff[2] = ID;
    outbuff[3] = 202;
    outbuff[4] = 0xfe;
    outbuff[5] = 0xfe;
    outbuff[6] = 0xfe;
    outbuff[7] = 0xfe;
    write(fd, outbuff, 8);
}
float motor_cali(unsigned char ID, float degree)
{
    if(ID==1)
    {
        degree += CALI_1;
        if(degree>360)
        degree -= 360;
    }
    else if(ID==2)
    {
        if (Mode14Flag)
            degree += CALI_2 + 180;
        else
            degree += CALI_2;

        if(degree>360)
        degree -= 360;
    }
    else if(ID==3)
    {
        degree += CALI_3;
        if(degree>360)
        degree -= 360;
    }
    else if(ID==4)
    {
        degree += CALI_4;
        if(degree>360)
        degree -= 360;
    }
    else if(ID==5)
    {
        if (Mode14Flag)
            degree += CALI_5 + 180;
        else
            degree += CALI_5;

        if(degree>360)
        degree -= 360;
    }
    else if(ID==6)
    {
        degree += CALI_6;
        if(degree>360)
        degree -= 360;
    }
    
    return degree;
}
void surge_forward(int fd){
    ros::Rate loop_rate(100);   //Hz

    for (int j = 0; j < flipper_iter_times; j++){
        float deg = 0.0;

        for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
            deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

            motor_pos[0] = -1*deg;
            set_motor_pos(fd, 1, motor_pos[0]);
            motor_pos[1] = deg;
            set_motor_pos(fd, 2, motor_pos[1]);
            motor_pos[2] = -1*deg;
            set_motor_pos(fd, 3, motor_pos[2]);
            motor_pos[3] = deg;
            set_motor_pos(fd, 4, motor_pos[3]);
            motor_pos[4] = -1*deg;
            set_motor_pos(fd, 5, motor_pos[4]);
            motor_pos[5] = deg;
            set_motor_pos(fd, 6, motor_pos[5]);

            loop_rate.sleep();
        }
    }

    for (int i = 0; i < 6; i++)
    {
        //set_motor_release(fd, i+1);
    }
}
void surge_backward(int fd){
    ros::Rate loop_rate(100);   //Hz

    for (int j = 0; j < flipper_iter_times; j++){
        float deg = 0.0;

        for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
            deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

            motor_pos[0] = 180+deg;
            set_motor_pos(fd, 1, motor_pos[0]);
            motor_pos[1] = 180-deg;
            set_motor_pos(fd, 2, motor_pos[1]);
            motor_pos[2] = 180+deg;
            set_motor_pos(fd, 3, motor_pos[2]);
            motor_pos[3] = 180-deg;
            set_motor_pos(fd, 4, motor_pos[3]);
            motor_pos[4] = 180+deg;
            set_motor_pos(fd, 5, motor_pos[4]);
            motor_pos[5] = 180-deg;
            set_motor_pos(fd, 6, motor_pos[5]);

            loop_rate.sleep();
        }
    }

    for (int i = 0; i < 6; i++)
    {
        //set_motor_release(fd, i+1);
    }
}
void heave_down(int fd){
    ros::Rate loop_rate(100);   //Hz

    for (int j = 0; j < flipper_iter_times; j++){
        float deg = 0.0;

        for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
            deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

            motor_pos[0] = 270+deg;
            set_motor_pos(fd, 1, motor_pos[0]);
            motor_pos[1] = 270-deg;
            set_motor_pos(fd, 2, motor_pos[1]);
            motor_pos[2] = 270+deg;
            set_motor_pos(fd, 3, motor_pos[2]);
            motor_pos[3] = 90-deg;
            set_motor_pos(fd, 4, motor_pos[3]);
            motor_pos[4] = 90+deg;
            set_motor_pos(fd, 5, motor_pos[4]);
            motor_pos[5] = 90-deg;
            set_motor_pos(fd, 6, motor_pos[5]);

            loop_rate.sleep();
        }
    }

    for (int i = 0; i < 6; i++)
    {
        //set_motor_release(fd, i+1);
    }
}
void heave_up(int fd){
    ros::Rate loop_rate(100);   //Hz

    for (int j = 0; j < flipper_iter_times; j++){
        float deg = 0.0;

        for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
            deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

            motor_pos[0] = 90-deg;
            set_motor_pos(fd, 1, motor_pos[0]);
            motor_pos[1] = 90+deg;
            set_motor_pos(fd, 2, motor_pos[1]);
            motor_pos[2] = 90-deg;
            set_motor_pos(fd, 3, motor_pos[2]);
            motor_pos[3] = 270+deg;
            set_motor_pos(fd, 4, motor_pos[3]);
            motor_pos[4] = 270-deg;
            set_motor_pos(fd, 5, motor_pos[4]);
            motor_pos[5] = 270+deg;
            set_motor_pos(fd, 6, motor_pos[5]);

            loop_rate.sleep();
        }
    }

    for (int i = 0; i < 6; i++)
    {
        //set_motor_release(fd, i+1);
    }
}
void yaw_right(int fd){
    ros::Rate loop_rate(100);   //Hz

    for (int j = 0; j < flipper_iter_times; j++){
        float deg = 0.0;

        for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
            deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

            motor_pos[0] = 180+deg;
            set_motor_pos(fd, 1, motor_pos[0]);
            motor_pos[1] = 180-deg;
            set_motor_pos(fd, 2, motor_pos[1]);
            motor_pos[2] = 180+deg;
            set_motor_pos(fd, 3, motor_pos[2]);
            motor_pos[3] = deg;
            set_motor_pos(fd, 4, motor_pos[3]);
            motor_pos[4] = -1*deg;
            set_motor_pos(fd, 5, motor_pos[4]);
            motor_pos[5] = deg;
            set_motor_pos(fd, 6, motor_pos[5]);

            loop_rate.sleep();
        }
    }

    for (int i = 0; i < 6; i++)
    {
        //set_motor_release(fd, i+1);
    }
}
void yaw_left(int fd){
    ros::Rate loop_rate(100);   //Hz

    for (int j = 0; j < flipper_iter_times; j++){
        float deg = 0.0;

        for (float time = 0.0; time <= flipper_period; time += (float)10 / 1000.0){
            deg = flipper_amplitude * sin(time * 3.141592 * 2 / flipper_period);

            motor_pos[0] = -1*deg;
            set_motor_pos(fd, 1, motor_pos[0]);
            motor_pos[1] = deg;
            set_motor_pos(fd, 2, motor_pos[1]);
            motor_pos[2] = -1*deg;
            set_motor_pos(fd, 3, motor_pos[2]);
            motor_pos[3] = 180-deg;
            set_motor_pos(fd, 4, motor_pos[3]);
            motor_pos[4] = 180+deg;
            set_motor_pos(fd, 5, motor_pos[4]);
            motor_pos[5] = 180-deg;
            set_motor_pos(fd, 6, motor_pos[5]);

            loop_rate.sleep();
        }
    }

    for (int i = 0; i < 6; i++)
    {
        //set_motor_release(fd, i+1);
    }
}
void flipper_ready(int fd, float target_pos_left, float target_pos_right){
    ros::Rate loop_rate(50);   //Hz
    ros::Rate wait_rate(2);   //Hz

    float angle_diff[6] = {0.0,};

    for (int i = 0; i < 3; i++) {
        angle_diff[i] = target_pos_left - motor_pos[i];
    }
    for (int j = 3; j < 6; j++) {
        angle_diff[j] = target_pos_right - motor_pos[j];
    }
    for (int l = 0; l < 6; l++) {
        if (angle_diff[l] > 181) {
            angle_diff[l] = angle_diff[l] - 360;
        }
        else if (angle_diff[l] < -181) {
            angle_diff[l] = angle_diff[l] + 360;
        }
    }

    for (float time = 0.0; time <= 1.0; time += (float)10 / 800.0){
        for (int k = 0; k < 6; k++) {
            motor_pos[k] = motor_pos[k] + angle_diff[k]/80.0;
            set_motor_pos(fd, k+1, motor_pos[k]);
        }
        loop_rate.sleep();
    }
    wait_rate.sleep();
}
