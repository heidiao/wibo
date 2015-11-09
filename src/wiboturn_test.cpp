#include "ros/ros.h"
#include "middleme/wiboturn_lib.h"
#include "sys/wait.h"

#define LOG_RED(X) printf("%s %s %s",COLOR_RED,X,COLOR_RESET)

#define TEST_TIME 5

using namespace std;
pid_t child_pid;

void motorHandler(int action, float x, float y)
{
    if(!isnan(x) && !isnan(y)){
        ros::NodeHandle n;
        ros::ServiceClient srv_client = n.serviceClient<middleme::MotorCtl>("wiboturn/motorctl");
        middleme::MotorCtl srv;
        srv.request.action = action;
        srv.request.x = x;
        srv.request.y = y;
        printf("action:%d, x:%.2f, y:%.2f\n",action, x,y );

        if (srv_client.call(srv))
            printf("%s|__ RESULT => %s, current_xy(%.2f,%.2f)%s\n\n", \
                    COLOR_GREEN, \
                    ((int)srv.response.status==0)?"success":"fail", \
                    (float)srv.response.x_current, \
                    (float)srv.response.y_current, \
                    COLOR_RESET);
        else
            LOG_RED("Failed to call service motorctl\n");
    }else{
        LOG_RED("motorHandler argument is not vaild\n");
    }
}

void soundHandler(string flacpath)
{
    if(flacpath!=""){
        ros::NodeHandle n;
        ros::ServiceClient srv_client = n.serviceClient<middleme::SoundLoc>("wiboturn/soundloc");
        middleme::SoundLoc srv;
        srv.request.flacpath = flacpath;

        if (srv_client.call(srv))
            printf("%s|__ RESULT => angle:%.2f%s\n\n", \
                    COLOR_GREEN, \
                    (float)srv.response.angle, \
                    COLOR_RESET);
        else
            LOG_RED("Failed from sound localizer service\n");
    }else{
        LOG_RED("soundHandler argument is not vaild\n");
    }
}

float RandomNumber(float min, float max)
{
    return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
}

void intHandler(int sig) {
    kill(child_pid, SIGKILL);
    exit(0);
}

void kill_child(int sig)
{
    kill(child_pid, SIGKILL);
}

int main(int argc, char ** argv)
{
    string input, time, x, y = "";
    float rx, ry;
    int t;

    ros::init(argc, argv, "wiboturn_test");

    while(1)
    {
        do
        {
            cout << "=========== ACTION =========" << endl;
            cout << "[0] Calibration" << endl;
            cout << "[1] Reset to default angle" << endl;
            cout << "[2] Drive" << endl;
            cout << "[3] Test motor drive" << endl;
            cout << "[4] Test sound localizer" << endl;
            cout << "[q] Exit" << endl;
            cout << "Input number:";
            getline(cin, input);
        }while(input.length() !=1 && input!="\n");

        switch(input[0])
        {
            case '0': //Calibration
            case '1': //Reset to default angle
                motorHandler(atoi(input.c_str()), 0, 0);
                break;

            //Drive
            case '2': 
                cout << "x (default 0):";
                getline(cin, x);
                cout << "y (default 0):";
                getline(cin, y);
                motorHandler(atoi(input.c_str()), atof(x.c_str()), atof(y.c_str()));
                break;

            //Test motor drive
            case '3':
                // ctrl+c kill process handler
                signal(SIGINT, intHandler);
                
                // kill child pid when time is up.
                signal(SIGALRM,(void (*)(int))kill_child);  

                cout << "How many second you want to test? (default: " << TEST_TIME << " sec):";
                getline(cin, time);

                child_pid = fork();
                if(child_pid == 0){ 
                    // start random xy to motor drive
                    while(1){
                        rx = RandomNumber(-90, 90);
                        ry = RandomNumber(-15, 40);
                        motorHandler(2, rx, ry);
                    }
                }else if(child_pid>0){ 
                    // parent pid for counting time
                    t = (strlen(time.c_str())==0) ? TEST_TIME: atoi(time.c_str());
                    alarm(t); 
                    // time is up, kill child process
                    wait(NULL);
                }
                printf("%sTest %d seconds is finished%s\n", COLOR_YELLOW, t, COLOR_RESET);
                break; 

            //Test sound localizer
            case '4':
                cout << "FLAC path:";
                getline(cin, input);
                soundHandler(input);
                break;

            //Quit
            case 'q':
                goto exit;
                break;
        }
    }
exit:
    exit(0);
    return 0;
}

