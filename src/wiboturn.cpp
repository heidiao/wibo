#include "ros/ros.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>

#include <vector>
#include <string.h>
#include <pthread.h>

#include "std_msgs/String.h"
#include "middleme/SoundLoc.h"
#include "middleme/MotorCtl.h"
#include "wiboutil.cpp"

extern "C" 
{
    #include   "serial_protocol.h"
    #include    "util.h"
    #include   "stepper_motor.h"
}

#define SOCKFILE "/tmp/soundloc_sockets"

#define BAUDRATE                            B921600
#define DEVICE                              "/dev/ttyACM0"
#define BUF_SIZE                            PATH_MAX


using namespace std;

void wiboSigintHandler(int sig)
{
    ROS_INFO("Shutdown");
    unlink(SOCKFILE);
    ros::shutdown();
    exit(0);
}

void motorHandler(const middleme::MotorCtl::ConstPtr& msg)
{
    //serial_send_command( uart_fd, NULL, 0);
    // x: -90~90
    // y: -90~90

    cout <<"x:" << msg->x << ", y:" << msg->y << endl;
}

void *soundLocalizerHandler(void* sound)
{
    ros::Publisher *p_soundloc_pub= (ros::Publisher*)sound;

    // declare socket
    int sock, sockconn, rval;
    char buf[1024];
    struct sockaddr_un server;

    // 10MHz
    ros::Rate loop_rate(10);
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

    ROS_INFO("Waiting for Sound localizer socket...");
    while (ros::ok())
    {
        // Listen socket connection
        sockconn = accept(sock, 0, 0);
        if (sockconn == -1)
            perror("accept");
        else do {
            bzero(buf, sizeof(buf));
            if ((rval = read(sockconn , buf, 1024)) < 0)
                perror("reading stream message");
            else if (rval == 0){
                ROS_INFO("Ending socket connection");
                break;
            }else{
                vector<string> obj = splitString(buf, "|");
                if(obj.size()==2){
                    middleme::SoundLoc soundloc_msg;
                    soundloc_msg.timestamp = ros::Time((float)atof(obj[0].c_str()));
                    soundloc_msg.angle = (float)atof(obj[1].c_str());
                    p_soundloc_pub->publish(soundloc_msg);
                    cout << "time:" << soundloc_msg.timestamp <<", angle:"<< soundloc_msg.angle <<endl;
                }
            }
        }while(rval > 0);
        close(sockconn);
        ros::spinOnce();
        loop_rate.sleep();
    }
    close(sock);
}



int main(int argc, char **argv)
{
    pthread_t threadSoundLoc;
    ros::init(argc, argv, "wiboturn", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    //MultiThreadedSpinner is a blocking spinner, similar to ros::spin()
    ros::MultiThreadedSpinner spinner(2);

    //Subscriber motor control
    ros::Subscriber motor_sub = n.subscribe("wiboturn/motorctl", 1000, motorHandler);

    // Publisher sound localizer
    ros::Publisher soundloc_pub = n.advertise<middleme::SoundLoc>("wiboturn/soundloc", 1000);

    // pthread create Sound localizer
    pthread_create( &threadSoundLoc, NULL, soundLocalizerHandler, (void*)& soundloc_pub);

    // handle CTRL+C interrupt
    signal(SIGINT, wiboSigintHandler);

    // uart 

    //motor callbacks will be called, through a call to ros::shutdown() or a Ctrl-C.
    spinner.spin();
    ros::waitForShutdown();
    return 0;
}
