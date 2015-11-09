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
#include <boost/thread.hpp>

#include "std_msgs/String.h"
#include "middleme/SoundLoc.h"
#include "middleme/MotorCtl.h"
#include "middleme/Sensor.h"

#define COLOR_RED     "\x1b[31m"
#define COLOR_GREEN   "\x1b[32m"
#define COLOR_YELLOW  "\x1b[33m"
#define COLOR_BLUE    "\x1b[34m"
#define COLOR_MAGENTA "\x1b[35m"
#define COLOR_CYAN    "\x1b[36m"
#define COLOR_RESET   "\x1b[0m"

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

#define R_ANGLE    -90
#define L_ANGLE    90

#define DOWN_ANGLE -15
#define UP_ANGLE    40

#define CALIBRATION_X    190
#define CALIBRATION_Y    60

#define RPM_START   2
#define RPM_END     12
#define ACCEL_TIME  100

#define SENSORS_ENABLED                     (SENSOR_ACCELEROMETER|SENSOR_GYROSCOPE|SENSOR_MAGNETIC)


using namespace std;
namespace wiboturn {

    class WiboTurnNode
    {
    private:
        typedef struct {
            uint16_t     start;
            uint16_t    end;
            uint16_t      time;
        } motor_period;

        enum{
            SENSOR_X_FRONT, //0
            SENSOR_X_LEFT,  //1
            SENSOR_X_BACK,  //2
            SENSOR_X_RIGHT  //3
        };

        enum{
            SENSOR_Y_UP, //0
            SENSOR_Y_DOWN//1
        };

        enum{
            ACTION_CALIBRATION, //0
            ACTION_RESET,       //1
            ACTION_DRIVE        //2
        };

        enum{
            STEP_OK =0,  //0
            STEP_STOP_X, //1
            STEP_STOP_Y, //2
            STEP_STOP,   //3
            STEP_START=99, 
            STEP_NG=-1 
        };

        motor_t *hori;
        motor_t *vert;

        int uart_fd, sockconn, chk_status;
        motor_period  x_per;
        motor_period  y_per;
        float current_x, current_y;
        bool stop_x, stop_y;

        ros::NodeHandle nh_;
        ros::ServiceServer motor_service;
        ros::ServiceServer soundloc_service;
        ros::Publisher sensor_pub;
        boost::thread    soundloc_thread;
        boost::thread    motor_thread;
        boost::mutex    mutex;

        motor_period motor_convert(motor_t *motor, uint16_t start, uint16_t end, uint16_t time);
        void limitAngle(int id, float *current, float *angle);
        void uart_init();
        bool readMotorResult(int compare_count, middleme::MotorCtl::Response &res);
        void soundlocSocketServer(int &sockconn);
        void motorThread(int &uart_fd, ros::Publisher &sensor_pub, int &chk_status, float &current_x, float &current_y, bool &stop_x, bool &stop_y);
    public:
        WiboTurnNode();
        ~WiboTurnNode();
        bool motorHandler(middleme::MotorCtl::Request  &req, middleme::MotorCtl::Response &res);
        bool soundlocHandler(middleme::SoundLoc::Request  &req, middleme::SoundLoc::Response &res);
        static void signalIntHandler(int sig);
    };
}

