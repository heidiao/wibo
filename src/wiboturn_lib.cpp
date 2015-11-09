#include "ros/ros.h"
#include <errno.h>
#include "middleme/wiboturn_lib.h"

using namespace std;

namespace wiboturn {
    /**
    * Constructor 
    * Convert rpm to period value
    * Initial ROS service and thread
    */
    WiboTurnNode::WiboTurnNode():
        nh_("~"),
        current_x(0),
        current_y(0), 
        hori(&pseudo_hori),
        vert(&pseudo_vert)
    {
        // Initial UART device
        uart_init();

        x_per = motor_convert(hori, RPM_START, RPM_END, ACCEL_TIME);
        y_per = motor_convert(vert, RPM_START, RPM_END, ACCEL_TIME);

        //Service of motor control
        motor_service = nh_.advertiseService("motorctl", &WiboTurnNode::motorHandler, this);

        //Service of sound localizer
        soundloc_service = nh_.advertiseService("soundloc", &WiboTurnNode::soundlocHandler, this);

        sensor_pub = nh_.advertise<middleme::Sensor>("sensor", 1000, this);

        // catching signal interrupt
        signal(SIGINT, WiboTurnNode::signalIntHandler);

        //socket connection...
        soundloc_thread = boost::thread(&WiboTurnNode::soundlocSocketServer, this,  boost::ref(sockconn));

        //sensor thread
        motor_thread = boost::thread(&WiboTurnNode::motorThread, this,  \
                                            boost::ref(uart_fd), \
                                            boost::ref(sensor_pub), \
                                            boost::ref(chk_status), \
                                            boost::ref(current_x), \
                                            boost::ref(current_y), \
                                            boost::ref(stop_x), \
                                            boost::ref(stop_y)
                                            );

        ros::spin();
    }

    /**
    * Signal interrupt handler, CTRL+C
    */
    void WiboTurnNode::signalIntHandler(int sig)
    {
        ROS_INFO("%s", __func__);
        exit(0);
    }

    void WiboTurnNode::motorThread(int &uart_fd, ros::Publisher &sensor_pub, \
                                    int &chk_status, \
                                    float &current_x, \
                                    float &current_y, \
                                    bool &stop_x, \
                                    bool &stop_y)
    {
        uint8_t buf[BUF_SIZE+1];
        char output[BUF_SIZE];
        int nread;

        cmd_short_t                 cmd_lsm6ds0_init            = { DEV_ADDR, SENDER_UART, CMD_LSM6DS0_INIT };
        cmd_short_t                 cmd_lis3mdl_init            = { DEV_ADDR, SENDER_UART, CMD_LIS3MDL_INIT };
        cmd_short_t                 cmd_stop_data_streaming     = { DEV_ADDR, SENDER_UART, CMD_STOP_DATA_STREAMING };
        cmd_start_data_streaming_t  cmd_start_data_streaming    = { DEV_ADDR, SENDER_UART, CMD_START_DATA_STREAMING, SENSORS_ENABLED, 100 };

       // start data streaming
        serial_send_command( uart_fd, &cmd_start_data_streaming, sizeof(cmd_start_data_streaming) );

        // init sensors
        if( SENSORS_ENABLED ) {
            if( SENSORS_ENABLED & (SENSOR_ACCELEROMETER|SENSOR_GYROSCOPE) ) {
                serial_send_command( uart_fd, &cmd_lsm6ds0_init, sizeof(cmd_short_t) );
            }
            if( SENSORS_ENABLED & SENSOR_MAGNETIC ) {
                serial_send_command( uart_fd, &cmd_lis3mdl_init, sizeof(cmd_short_t) );
            }
        }

        while(1){
            if((nread = serial_read_message( uart_fd, buf, BUF_SIZE )) >0){
                if( buf[0]==SENDER_UART && buf[1]==DEV_ADDR ){
                    bzero(output, sizeof(output));
                    switch(buf[2]){
                        case CMD_START_DATA_STREAMING:
                            // motor command result
                            if( SENSORS_ENABLED& SENSOR_ACCELEROMETER ){
                                middleme::Sensor sensor_msg;
                                sensor_msg.sid = 0;
                                sensor_msg.x=  deserialize(buf+15,4);
                                sensor_msg.y= deserialize(buf+19,4);
                                sensor_msg.z= deserialize(buf+23,4);
                                sensor_pub.publish(sensor_msg);
                            }
                            if( SENSORS_ENABLED & SENSOR_GYROSCOPE ){
                                middleme::Sensor sensor_msg;
                                sensor_msg.sid = 1;
                                sensor_msg.x=  deserialize(buf+27,4);
                                sensor_msg.y= deserialize(buf+31,4);
                                sensor_msg.z= deserialize(buf+35,4);
                                sensor_pub.publish(sensor_msg);
                            }if( SENSORS_ENABLED & SENSOR_MAGNETIC ){
                                middleme::Sensor sensor_msg;
                                sensor_msg.sid = 2;
                                sensor_msg.x=  deserialize(buf+39,4);
                                sensor_msg.y= deserialize(buf+43,4);
                                sensor_msg.z= deserialize(buf+47,4);
                                sensor_pub.publish(sensor_msg);
                            }
                            break;

                        // motor drive is finished
                        case CMD_MOTOR_STOP|CMD_REPLY_ADD:
                        case CMD_MOTOR_STOP:
                            if( deserialize( buf+4, 2 )){
                                mutex.lock();
                                chk_status--;
                                mutex.unlock();
                            //    sprintf(output, "READ: CMD_MOTOR_STOP Finished, id=%d", buf[3]);
                            }
                            break;

                        // receive sensor
                        case CMD_MOTOR_REPORT_SENSOR|CMD_REPLY_ADD:
                            sprintf(output, "READ: CMD_MOTOR_REPORT_SENSOR, id = %d, sensor = %d, steps = %d\n", \
                                         buf[3], buf[4], deserialize( buf+5, 2 ) );
                            // x sensor is be hited
                            mutex.lock();
                            if(buf[3] == 0){
                                switch(buf[4]){
                                    case SENSOR_X_LEFT:
                                        current_x = L_ANGLE;
                                        stop_x=true;
                                        break;
                                    case SENSOR_X_RIGHT:
                                        current_x = R_ANGLE;
                                        stop_x=true;
                                        break;
                                }
                            }
                            //y sensor is be hited
                            if(buf[3] == 1){
                                switch(buf[4]){
                                    case SENSOR_Y_UP:
                                        current_y = UP_ANGLE;
                                        break;
                                    case SENSOR_Y_DOWN:
                                        current_y = DOWN_ANGLE;
                                        break;
                                }
                                stop_y=true;
                            }
                            mutex.unlock();
                            break;
                    } //end switch 
                    if(strlen(output)>0)
                        cout << COLOR_YELLOW <<  output << COLOR_RESET << endl;
                }
            } else if( nread==-1 ) {
                // read()<0 error
            } else if( nread==-2 ) {
                // USB disconnect
                exit(-1);
            } else if( nread==-3 ) {
                // resync or checksum error
            }
        } //end while
    }
 
    /**
    * sound socket service 
    * Initial socket service, connect with sound localizer socket client
    */
    void WiboTurnNode::soundlocSocketServer(int &sockconn)
    {
        // declare socket
        int sock, rval;
        struct sockaddr_un server;

        // Initialize socket stream
        sock = socket(AF_UNIX, SOCK_STREAM, 0);
        if (sock < 0) {
            perror("opening stream socket");
            exit(1);
        }
        server.sun_family = AF_UNIX;
        strcpy(server.sun_path, SOCKFILE);

        // Check socket file exists, remove.
        if (access(SOCKFILE, F_OK ) != -1)
            unlink(SOCKFILE);

        // Binding stream socket
        if (bind(sock, (struct sockaddr *) &server, sizeof(struct sockaddr_un))) {
            perror("binding stream socket");
            exit(1);
        }

        // Listen 1 connection
        listen(sock, 1);

        ROS_INFO("Waiting for socket connection...");
        ROS_INFO("    1) Open another terminal");
        ROS_INFO("    2) Execute: java -jar Soundlocalizer.jar");
        while (ros::ok())
        {
            // Listen socket connection
            sockconn = accept(sock, 0, 0);
            if (sockconn>0){
                cout <<  "Receive a connect,  sockconn:" << sockconn << endl;
            }
            ros::spinOnce();
        }
        close(sock);
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
    * motorHandler
    * set motor to calibration, reset or drive mode
    * return the service client's request,  true if successful, otherwise false
    */
    bool WiboTurnNode::motorHandler(middleme::MotorCtl::Request  &req,
             middleme::MotorCtl::Response &res)
    {
        //Declare
        int  ret;
        float x,y;
        time_t endwait;
        int seconds = 3; // timeout 3 second.

        endwait = time (NULL) + seconds ;
        mutex.lock();
        stop_x = false;
        stop_y = false;
        chk_status = 0;
        mutex.unlock();

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
                    chk_status++;
                }
                if(current_y < UP_ANGLE){
                    serial_send_command( uart_fd, motor_gen_cmd_motor_drive( vert, &cmd, CW, motor_deg2step(vert, CALIBRATION_Y), y_per.start, y_per.end, y_per.time ), sizeof(cmd_motor_drive_t) );
                    chk_status++;
                }

                // wait for finished
                while(time(NULL) < endwait){
                    if(chk_status == 0)
                        break;
                }
                endwait = time (NULL) + seconds ;

                serial_send_command( uart_fd, motor_gen_cmd_motor_drive( hori, &cmd, CCW, motor_deg2step(hori, L_ANGLE), x_per.start, x_per.end, x_per.time ), sizeof(cmd_motor_drive_t) );
                serial_send_command( uart_fd, motor_gen_cmd_motor_drive( hori, &cmd, CCW, motor_deg2step(hori, UP_ANGLE), x_per.start, x_per.end, x_per.time ), sizeof(cmd_motor_drive_t) );
                chk_status=2;

                // wait for finished
                while(time(NULL) < endwait){
                    if(chk_status == 0)
                        break;
                }

                current_x=0;
                current_y=0;
                return true;
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
            printf("%sSEND: x=%f%s\n",COLOR_YELLOW, x, COLOR_RESET);
            serial_send_command( uart_fd, motor_gen_cmd_motor_drive( hori, &cmd, dir_x, motor_deg2step(hori, abs(x)), x_per.start, x_per.end, x_per.time ), sizeof(cmd_motor_drive_t) );
            mutex.lock();
            chk_status++;
            mutex.unlock();
        }
        if(abs(y)>=1){
            dir_y = (y>0)?CW:CCW;
            printf("%sSEND: y=%f%s\n",COLOR_YELLOW, y, COLOR_RESET);
            serial_send_command( uart_fd, motor_gen_cmd_motor_drive( vert, &cmd, dir_y, motor_deg2step(vert, abs(y)), y_per.start, y_per.end, y_per.time ), sizeof(cmd_motor_drive_t) );
            mutex.lock();
            chk_status++;
            mutex.unlock();
        }
 
        // wait for finished
        while(time(NULL) < endwait){
            if(chk_status == 0){
                if(stop_x || stop_y){
                    if(stop_x && stop_y)
                        res.status = STEP_STOP;
                    else if (stop_x)
                        res.status = STEP_STOP_X;
                    else if (stop_y)
                        res.status = STEP_STOP_Y;
                }else{
                    res.status = STEP_OK;
                }
                // set the current angle to service response message
                res.x_current = current_x;
                res.y_current = current_y;
                printf("%s|__ RESULT => %s, current_xy(%.2f,%.2f)%s\n\n", \
                        COLOR_YELLOW, \
                        ((int)res.status==0)?"success":"fail", \
                        (float)res.x_current, \
                        (float)res.y_current, \
                        COLOR_RESET);
                return true;
            }
        }
        // timeout
        res.status = STEP_NG;
        cout << COLOR_YELLOW << "timeout" << COLOR_RESET << endl;
        return false;
    }

    /**
    * soundlocHandler
    * request flac path to soundlocalizer java socket, and response the x angle.
    * return the service client's request,  true if successful, otherwise false
    */
    bool WiboTurnNode::soundlocHandler(middleme::SoundLoc::Request  &req,
             middleme::SoundLoc::Response &res)
    {
        // FLAC path name
        string flacpath = req.flacpath;

        // Sound angle
        char angle[BUF_SIZE];
        int ret;
        
        if (flacpath != "" && sockconn > 0){
            if (access(flacpath.c_str(), F_OK ) == -1){
                cout << COLOR_RED << "\""<<flacpath<<"\" is not exist."<<COLOR_RESET<<endl;
                return false;
            }
            cout << "Receive FLAC path:" << flacpath << endl;

            // socket write to sound localizer
            write(sockconn ,flacpath.c_str(),strlen(flacpath.c_str()));

            // socket read from sound localizer
            bzero(angle, sizeof(angle));
            if ((ret = read(sockconn , angle, BUF_SIZE)) > 0) {
                cout << COLOR_GREEN<<"Send angle:"<< angle <<COLOR_RESET<<endl;
                res.angle = (float)atof(angle);
                return true;
            }else if(ret == 0){
                cout << COLOR_RED<<"Disconnect sockconn:"<< sockconn <<COLOR_RESET<<endl;
            }else{
                cout << COLOR_RED<<"Error read ret:"<< ret <<COLOR_RESET<<endl;
            }
        }
        return false;
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

