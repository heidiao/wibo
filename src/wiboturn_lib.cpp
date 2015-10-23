#include "ros/ros.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h> 
#include <vector>
#include <string.h>
#include <pthread.h>
#include <termios.h> 
#include <sys/select.h>
#include <assert.h>

#include "std_msgs/String.h"
#include "middleme/SoundLoc.h"
#include "middleme/MotorCtl.h"
#include "middleme/wiboturn_lib.h"

using namespace std;
namespace wiboturn {
    WiboTurnNode::WiboTurnNode():
        nh_("~"),
        current_x(0),
        current_y(0)
    {
        cout << "WiboTurnNode()"<< endl;

        //MultiThreadedSpinner is a blocking spinner, similar to ros::spin()
        ros::MultiThreadedSpinner spinner(2);

        uart_init();
        x_per = motor_convert(&hori, RPM_START, RPM_END, ACCEL_TIME);
        y_per = motor_convert(&vert, RPM_START, RPM_END, ACCEL_TIME);

        //Service motor control
        motor_service = nh_.advertiseService("motorctl", &WiboTurnNode::motorHandler, this);

        spinner.spin();
    }


    WiboTurnNode::motor_period WiboTurnNode::motor_convert(motor_t *motor, uint16_t start, uint16_t end, uint16_t time)
    {
        WiboTurnNode::motor_period p;
        p.start = motor_rpm2period( motor, start);
        p.end = motor_rpm2period( motor , end);
        p.time = (p.start-p.end)*( (uint32_t) p.start+p.end) / (2*time/1000.0*motor->freq);
        return p;
    }

    void WiboTurnNode::limitAngle(int id, float *current, float *angle)
    {
        float r,l;
        if(abs(*angle)>0){
            if (id==0){ 
                // x angle
                r = RANGLE;
                l = LANGLE;
            }else{
                // y angle range
                r = RANGLE_V;
                l = LANGLE_V;
            }
            if( *current+*angle > r){
                *angle = r-*current;
                *current = r;
            }else if ( *current+*angle < l){
                *angle = l-*current;
                *current = l;
            }else if (*current+*angle <= r && *current+*angle >= l){
                *current += *angle;
            }else if(*current == r || *current == l){
                *angle = 0;
            }
        }
    }

    void WiboTurnNode::uart_init()
    {
        //uart 
        struct termios  uart_config;

        // open uart device
        uart_fd = open( DEVICE, O_RDWR | O_NOCTTY);
        if( uart_fd<0 ) {
            perror( DEVICE );
            exit(-1);
        }

        //test whether a file descriptor refers to a terminal
        if(!isatty(uart_fd)){
            perror("file descriptor is not refers to a terminal");
            exit(-1);
        }

        //Get the current configuration of the serial interface
        if(tcgetattr(uart_fd, &uart_config) < 0) {
            perror("cannot get current config of the serial interface.");
            exit( -1 );
        }

        uart_config.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB);
        uart_config.c_cflag |= CS8 | CLOCAL | CREAD;
        uart_config.c_oflag = 0;
        uart_config.c_lflag = 0;
        uart_config.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);

        // Communication speed 
        if(cfsetispeed(&uart_config, BAUDRATE) < 0 || cfsetospeed(&uart_config, BAUDRATE) < 0) {
            perror("communication speed");
            exit(-1);
        }
        uart_config.c_cc[VMIN] = 1;         // block mode
        uart_config.c_cc[VTIME] = 1;

        tcflush( uart_fd, TCIOFLUSH );
        // apply the configuration
        if(tcsetattr(uart_fd, TCSAFLUSH, &uart_config) < 0) {
            perror("apply config");
            exit(-1);
        }
        ROS_INFO("UART Successfully");
    }

    bool WiboTurnNode::readMotorResult(int compare_count, float *res_x, float *res_y, int *res_status)
    {
        int nread, status, ok_count;
        uint8_t result[BUF_SIZE+1];
        bzero(result, sizeof(result));

        status = STEP_START;
        ok_count=0;

        // Read current status
        while(compare_count>0){
            nread = serial_read_message( uart_fd, result, BUF_SIZE );
            if(nread>0){
                if( result[0]==SENDER_UART && result[1]==DEV_ADDR ) {
                    switch(result[2]-CMD_ID_REPLY_ADD){
                        case CMD_ID_MOTOR_DRIVE:
                            if(result[3] ==0){
                                status = STEP_NG;
                                printf("READ:fail, ret = %d\n",result[3]);
                            }else{
                                ok_count+=1;
                                printf("READ:success, ret = %d\n",result[3]);
                            }
                            break;
                        case CMD_ID_MOTOR_STOP:
                            if( result[4] ){
                                ok_count+=1;
                                printf( "receive motor stop async event, id=%d\n", result[3]);
                            }
                            break;
                    }
                    if(ok_count == compare_count*2)
                        status = STEP_OK;
                    if(status != STEP_START)
                        break;
                }
            }
        }
        cout << "RESULT STATUS:" << status <<endl << endl;;
        *res_x = current_x;
        *res_y = current_y;
        if(status>=0){
            *res_status = status;
            return true;
        }else{
            return false;
        }
    }

    //Destructor
    WiboTurnNode::~WiboTurnNode()
    {
        cout << "~WiboTurnNode Destructor"<< endl;
        unlink(SOCKFILE);
        close(uart_fd);
        ros::shutdown();
        exit(0);
    }


    bool WiboTurnNode::motorHandler(middleme::MotorCtl::Request  &req,
             middleme::MotorCtl::Response &res)
    {
        //Declare
        int compare_count=0;
        float x,y;

        cout << "motorHandler()"<< endl;

        // Iniital 
        cmd_motor_drive_t cmd;
        dir_t dir_x, dir_y;

        // Action 
        switch(req.action)
        {
            case ACTION_CALIBRATION: // motor calibration
                return false;
                break;
            case ACTION_RESET: // Reset x, y to 0
                x = -current_x;
                y = -current_y;
                dir_x = (x>0)?CW:CCW;
                dir_y = (y>0)?CW:CCW;

                cout << "CURRENT_xy:(" << current_x << ", "<< current_y<< " ), xy("<< x <<","<<y<<")"<< endl;
                if(abs(x)!=0 ){
                    serial_send_command( uart_fd, motor_gen_cmd_motor_drive( &hori, &cmd, dir_x, motor_deg2step(&hori, abs(x)), x_per.start, x_per.end, x_per.time ), sizeof(cmd_motor_drive_t) );
                    compare_count+=1;
                }
                if(abs(y)!=0){
                    serial_send_command( uart_fd, motor_gen_cmd_motor_drive( &vert, &cmd, dir_y, motor_deg2step(&vert, abs(y)), y_per.start, y_per.end, y_per.time ), sizeof(cmd_motor_drive_t) );
                    compare_count+=1;
                }
                current_x=0;
                current_y=0;

                // Read current status
                return readMotorResult(compare_count, &res.x_current, &res.y_current, &res.status);
                break;
            case ACTION_DRIVE:  //drive x,y
                x = req.x;
                y = req.y;
                limitAngle(0, &current_x, &x);
                limitAngle(1, &current_y, &y);
                dir_x = (x>0)?CW:CCW;
                dir_y = (y>0)?CW:CCW;

                cout << "CURRENT_xy:(" << current_x << ", "<< current_y<< " ), xy("<< x <<","<<y<<")"<< endl;
                if(abs(x)!=0 ){
                    serial_send_command( uart_fd, motor_gen_cmd_motor_drive( &hori, &cmd, dir_x, motor_deg2step(&hori, abs(x)), x_per.start, x_per.end, x_per.time ), sizeof(cmd_motor_drive_t) );
                    compare_count+=1;
                }
                if(abs(y)!=0){
                    serial_send_command( uart_fd, motor_gen_cmd_motor_drive( &vert, &cmd, dir_y, motor_deg2step(&vert, abs(y)), y_per.start, y_per.end, y_per.time ), sizeof(cmd_motor_drive_t) );
                    compare_count+=1;
                }
                // Read current status
                return readMotorResult(compare_count, &res.x_current, &res.y_current, &res.status);
                break;
           default:
                return false;
                break;
        }
    }
}
