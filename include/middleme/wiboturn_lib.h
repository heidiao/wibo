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

extern "C" 
{
    #include    "def.h"
    #include    "serial_protocol.h"
    #include    "stepper_motor.h"
    #include    "util.h"
}


#define BAUDRATE    B921600
#define DEVICE      "/dev/ttyUSB0"
#define BUF_SIZE    PATH_MAX

#define SOCKFILE    "/tmp/soundloc_sockets"
#define LANGLE      -90
#define RANGLE      90

#define LANGLE_V    -28
#define RANGLE_V    28

#define STEP_START  99
#define STEP_OK     0
#define STEP_STOP  1
#define STEP_NG     -1

#define ACTION_CALIBRATION    0
#define ACTION_RESET    1
#define ACTION_DRIVE    2

#define RPM_START 2
#define RPM_END 12
#define ACCEL_TIME 100

namespace wiboturn {

    class WiboTurnNode
    {
    private:
        typedef struct {
            uint16_t     start;
            uint16_t    end;
            uint16_t      time;
        } motor_period;

        int uart_fd;
        motor_period  x_per;
        motor_period  y_per;
        float current_x, current_y;

        ros::NodeHandle nh_;
        ros::ServiceServer motor_service;

        motor_period motor_convert(motor_t *motor, uint16_t start, uint16_t end, uint16_t time);
        
        void limitAngle(int id, float *current, float *angle);
        void uart_init();
        bool readMotorResult(int compare_count, float *res_x, float *res_y, int *res_status);
        
    public:
        WiboTurnNode();
        ~WiboTurnNode();
        bool motorHandler(middleme::MotorCtl::Request  &req, middleme::MotorCtl::Response &res);
    };
}

