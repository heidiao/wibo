#include <iostream>
#include <stack>
#include "ros/ros.h"
#include "middleme/SoundLoc.h"
#include "middleme/MotorCtl.h"

using namespace std;
bool lock = false;
float x = 0;


ros::Subscriber sub;
ros::ServiceClient client;

void soundMotorAction(float x)
{
    middleme::MotorCtl srv;
    srv.request.action = 2;
    srv.request.x = x;
    srv.request.y = 0;
     
    if (client.call(srv))
        ROS_INFO("status: %d, x:%f, y:%f\n", (int)srv.response.status, (float)srv.response.x_current, (float)srv.response.y_current);
    else
        ROS_ERROR("Failed to call service motorctl");
    sleep(2); //filter noise when the motor driving.
}

void soundCallback(const middleme::SoundLoc::ConstPtr& msg)
{
    x = msg->angle;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "wiboturn_demo");
    ros::NodeHandle n;
    client = n.serviceClient<middleme::MotorCtl>("wiboturn/motorctl");
    sub = n.subscribe("wiboturn/soundloc", 1000, soundCallback);
    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        if(!lock && x!=0){
            lock = true;
            soundMotorAction(x);
            lock = false;
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
