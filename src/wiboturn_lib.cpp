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
    /**
    * Constructor 
    * convert rpm to period value
    */
    WiboTurnNode::WiboTurnNode():
        nh_("~"),
        current_x(0),
        current_y(0)
    {

        hori = &pseudo_hori;
        vert = &pseudo_vert;

        //MultiThreadedSpinner is a blocking spinner, similar to ros::spin()
        ros::MultiThreadedSpinner spinner(2);

        uart_init();
        x_per = motor_convert(hori, RPM_START, RPM_END, ACCEL_TIME);
        y_per = motor_convert(vert, RPM_START, RPM_END, ACCEL_TIME);

        //Service motor control
        motor_service = nh_.advertiseService("motorctl", &WiboTurnNode::motorHandler, this);

        spinner.spin();
    }
 
    /**
    * motor_convert
    * convert rpm to period value
    */
    WiboTurnNode::motor_period WiboTurnNode::motor_convert(motor_t *motor, uint16_t start, uint16_t end, uint16_t time)
    {
        WiboTurnNode::motor_period p;
        p.start = motor_rpm2period( motor, start);
        p.end = motor_rpm2period( motor , end);
        p.time = (p.start-p.end)*( (uint32_t) p.start+p.end) / (2*time/1000.0*motor->freq);
        return p;
    }

 
    /**
    * limitAngle
    * calculate current angle and target angle.
    */
    void WiboTurnNode::limitAngle(int id, float *current, float *angle)
    {
        float positive, negative;
        if(abs(*angle)>0){
            // x angle
            if (id == 0){ 
                positive = L_ANGLE;
                negative = R_ANGLE;
            }else{
            // y angle range
                positive = UP_ANGLE;
                negative = DOWN_ANGLE;
            }
            
            // positive limitation
            if( *current+*angle > positive){ 
                *angle = positive -*current;
                *current = positive;

            // negative limitation
            }else if ( *current+*angle < negative){
                *angle = negative-*current;
                *current = negative;

            // angle in good range 
            }else if (*current+*angle <= positive && *current+*angle >= negative){
                *current += *angle;

            // already hit the limit
            }else if(*current == positive || *current == negative){
                *angle = 0;
            }
        }
    }

    /**
    * uart_init 
    * setup uart serial port, communicate with motor drive perpose
    */
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
        ROS_INFO("Initial UART [%s] Successfully", DEVICE);
    }

    /**
    * readMotorResult 
    * get uart response message
    * return true if successful, otherwise false
    */
    bool WiboTurnNode::readMotorResult(int compare_count, float *res_x, float *res_y, int *res_status)
    {
        int retval, status = STEP_START, ok_count = 0;
        bool stop_x=false, stop_y=false;
        uint8_t result[BUF_SIZE+1];
        struct timeval timeout;
        uint16_t real_steps;
        fd_set fds;

        timeout.tv_sec = 3; //timeout 3 second
        timeout.tv_usec = 0;

        FD_ZERO(&fds);
        FD_SET(uart_fd, &fds);

        bzero(result, sizeof(result));

        // Read current status
        while(compare_count>0){
            retval = select( uart_fd+1, &fds, NULL, NULL, &timeout );
            if( retval>0 ) {
                if(serial_read_message( uart_fd, result, BUF_SIZE ) >0){
                    if( result[0]==SENDER_UART && result[1]==DEV_ADDR ) {
                        real_steps=0;
                        switch(result[2]-CMD_REPLY_ADD){
                            // motor command result
                            case CMD_MOTOR_DRIVE:
                                if(result[3] ==0){
                                    status = STEP_NG;
                                    ROS_ERROR("READ: Fail, ret = %d\n",result[3]);
                                }else{
                                    ok_count+=1;
                                    ROS_INFO("READ: Success, ret = %d\n",result[3]);
                                }
                                break;
                            // motor drive is finished
                            case CMD_MOTOR_STOP:
                                real_steps = deserialize( result+4, 2 );
                                if( real_steps ){
                                    ok_count+=1;
                                    ROS_INFO("READ: Finished, id=%d, real_steps=%d\n", result[3], real_steps);
                                }
                                break;

                            // receive sensor
                            case CMD_MOTOR_REPORT_SENSOR:
                                ROS_INFO( "READ: Sensor, id = %d, sensor = %d, steps = %d\n", \
                                             result[3], result[4], deserialize( result+5, 2 ) );
                                // x sensor
                                if(result[3] == 0){
                                    switch(result[4]){
                                        case SENSOR_X_FRONT:
                                            current_x = 0;
                                            break;
                                        case SENSOR_X_LEFT:
                                            current_x = L_ANGLE;
                                            stop_x=true;
                                            break;
                                        case SENSOR_X_RIGHT:
                                            current_x = R_ANGLE ;
                                            stop_x=true;
                                            break;
                                    }
                                }
     
                                //y sensor
                                if(result[3] == 1){
                                    switch(result[4]){
                                        case SENSOR_Y_UP:
                                            current_y = UP_ANGLE;
                                            break;
                                        case SENSOR_Y_DOWN:
                                            current_y = DOWN_ANGLE;
                                            break;
                                    }
                                    stop_y=true;
                                }
                                break;
                        }

                        //read x, y is done 
                        if(ok_count == compare_count*2){
                            status = STEP_OK;
                            if(stop_x || stop_y){
                                if(stop_x && stop_y)
                                    status = STEP_STOP;
                                else if (stop_x)
                                    status = STEP_STOP_X;
                                else if (stop_y)
                                    status = STEP_STOP_Y;
                            }
                        }
                        if(status != STEP_START)
                            break;
                    }
                }

            } else if( retval==0 ){
                // timeout
                ROS_ERROR("READ: timeout");
                break;
            }

        }

        // set the current angle to service response message
        *res_x = current_x;
        *res_y = current_y;

        // return result
        if(status != STEP_NG){
            ROS_INFO("RESULT: status: %d, current_xy(%f, %f)\n", status, current_x, current_y);
            *res_status = status;
            return true;
        }else{
            return false;
        }
    }

    /**
    * motorHandler
    * set motor to calibration, reset or drive mode
    * return the service client's request,  true if successful, otherwise false
    */
    bool WiboTurnNode::motorHandler(middleme::MotorCtl::Request  &req,
             middleme::MotorCtl::Response &res)
    {
        //Declare
        int compare_count=0, ret;
        float x,y;

        // Iniital 
        cmd_motor_drive_t cmd;
        dir_t dir_x, dir_y;

        // Action 
        switch(req.action)
        {
            // setup motor calibration
            case ACTION_CALIBRATION: 

#if 0
                if(current_x < L_ANGLE){
                    serial_send_command( uart_fd, motor_gen_cmd_motor_drive( hori, &cmd, CW, motor_deg2step(hori, CALIBRATION_X), x_per.start, x_per.end, x_per.time ), sizeof(cmd_motor_drive_t) );
                    compare_count+=1;
                }
                //if(current_y < UP_ANGLE){
                //    serial_send_command( uart_fd, motor_gen_cmd_motor_drive( vert, &cmd, CW, motor_deg2step(vert, CALIBRATION_Y), y_per.start, y_per.end, y_per.time ), sizeof(cmd_motor_drive_t) );
                //    compare_count+=1;
                //}
                if(compare_count>0){
                    ret = readMotorResult(compare_count, &res.x_current, &res.y_current, &res.status);
                    compare_count = 0;
                }

                if(current_x!=0){
                    serial_send_command( uart_fd, motor_gen_cmd_motor_drive( hori, &cmd, CCW, motor_deg2step(hori, L_ANGLE), x_per.start, x_per.end, x_per.time ), sizeof(cmd_motor_drive_t) );
                    compare_count+=1;
                }
                //serial_send_command( uart_fd, motor_gen_cmd_motor_drive( hori, &cmd, CCW, motor_deg2step(hori, UP_ANGLE), x_per.start, x_per.end, x_per.time ), sizeof(cmd_motor_drive_t) );
                //compare_count+=1;

                current_x=0;
                //current_y=0;

                return readMotorResult(compare_count, &res.x_current, &res.y_current, &res.status);
#else
                return false;
#endif
                break;

            // set motor to default angle, x=0, y=0
            case ACTION_RESET: 
                x = -current_x;
                y = -current_y;
                current_x=0;
                current_y=0;
                break;

            // set motor to target angle
            case ACTION_DRIVE: 
                x = req.x;
                y = req.y;
                limitAngle(0, &current_x, &x);
                limitAngle(1, &current_y, &y);
               break;
           default:
                return false;
                break;
        }

        // send command to motor drive
        if(abs(x)>=1 ){
            dir_x = (x>0)?CW:CCW;
            ROS_INFO("SEND: x=%f", x);
            serial_send_command( uart_fd, motor_gen_cmd_motor_drive( hori, &cmd, dir_x, motor_deg2step(hori, abs(x)), x_per.start, x_per.end, x_per.time ), sizeof(cmd_motor_drive_t) );
            compare_count+=1;
        }
        if(abs(y)>=1){
            dir_y = (y>0)?CW:CCW;
            ROS_INFO("SEND: y=%f", y);
            serial_send_command( uart_fd, motor_gen_cmd_motor_drive( vert, &cmd, dir_y, motor_deg2step(vert, abs(y)), y_per.start, y_per.end, y_per.time ), sizeof(cmd_motor_drive_t) );
            compare_count+=1;
        }

        // Read current status
        return readMotorResult(compare_count, &res.x_current, &res.y_current, &res.status);
    }

    /**
    * Destructor
    */
    WiboTurnNode::~WiboTurnNode()
    {
        unlink(SOCKFILE);
        close(uart_fd);
        ros::shutdown();
        exit(0);
    }
}
