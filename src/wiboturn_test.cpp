#include <iostream>
#include "ros/ros.h"
#include "middleme/MotorCtl.h"

using namespace std;
ros::ServiceClient srv_client;

void motorHandler(int action, float x, float y)
{
    if(!isnan(x) && !isnan(y)){
        ROS_INFO("action:%d, x:%f, y:%f", action, x,y);
        middleme::MotorCtl srv;
        srv.request.action = action;
        srv.request.x = x;
        srv.request.y = y;
         
        if (srv_client.call(srv))
            ROS_INFO("status: %d, x:%f, y:%f\n", (int)srv.response.status, (float)srv.response.x_current, (float)srv.response.y_current);
        else
            ROS_ERROR("Failed to call service motorctl");
    }else{
        ROS_ERROR("motorHandler argument is not vaild");
    }
}

int main(int argc, char ** argv)
{
    string input, x, y = "";
    ros::init(argc, argv, "wiboturn_test");
    ros::NodeHandle n;
    srv_client = n.serviceClient<middleme::MotorCtl>("wiboturn/motorctl");
    while(1)
    {
        do
        {
            cout << "=========== ACTION =========" << endl;
            cout << "[0] Calibration" << endl;
            cout << "[1] Reset to default angle" << endl;
            cout << "[2] Drive" << endl;
            cout << "[q] Exit" << endl;
            cout << "Input number:";
            getline(cin, input);
        }while(input.length() !=1 && input!="\n");

        switch(input[0])
        {
            case '0':
            case '1':
                motorHandler(atoi(input.c_str()), 0, 0);
                break;
            case '2':
                cout << "x (default 0):";
                getline(cin, x);
                cout << "y (default 0):";
                getline(cin, y);
                motorHandler(atoi(input.c_str()), atof(x.c_str()), atof(y.c_str()));
                break;
            case 'q':
                goto exit;
                break;
        }
    }
exit:
    exit(0);
    return 0;
}
