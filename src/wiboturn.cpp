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

vector<string> splitString(string input, string delimiter)
{
    vector<string> output;
    size_t start = 0;
    size_t end = 0;

    while (start != string::npos && end != string::npos){
        start = input.find_first_not_of(delimiter, end);
        if (start != string::npos){
            end = input.find_first_of(delimiter, start);
            if (end != string::npos){
                output.push_back(input.substr(start, end - start));
            }else{
                output.push_back(input.substr(start));
            }
        }
    }
    return output;
}

void *soundLocalizerHandler(void*)
{

    ros::NodeHandle n;
    ros::Publisher p_soundloc_pub = n.advertise<middleme::SoundLoc>("wiboturn/soundloc", 1000);

    // declare socket
    int sock, sockconn, rval;
    char buf[BUF_SIZE];
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
            if ((rval = read(sockconn , buf, BUF_SIZE)) < 0)
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
                    p_soundloc_pub.publish(soundloc_msg);
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
    ros::init(argc, argv, "wiboturn");

    // sound localizer thread, asynchronous .
    pthread_t threadSoundLoc;
    pthread_create( &threadSoundLoc, NULL, soundLocalizerHandler, (void*)NULL);

    // start Motor Service
    wiboturn::WiboTurnNode wnode;

    ros::waitForShutdown();

    return 0;
}
