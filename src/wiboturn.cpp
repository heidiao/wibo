#include "ros/ros.h"
#include "middleme/wiboturn_lib.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wiboturn");

    // Start roboturn node Service
    wiboturn::WiboTurnNode wnode;

    ros::waitForShutdown();

    return 0;
}
