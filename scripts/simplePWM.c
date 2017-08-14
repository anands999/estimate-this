#include "stdio.h"
#include "RoboPiLib.h"

#define LED_OUTPUT 0
#define PUSH_BUTTON 7

int main(int argc, char *argv[])
{
    int output = 0;
    int out = 500;
    //  ros::init(argc, argv, "spfRun");
    //  ros::NodeHandle n;
    //  ros::Rate loop_rate(10);
    //  printf(argv[1]);
    RoboPiInit("/dev/ttySAC0", 115200);
    printf("Hello world\n");

    printf("Pin 16: %d",output);

    pinMode(PUSH_BUTTON,INPUT);

    output=readMode(16);

    while (1){
        if (digitalRead(PUSH_BUTTON)==1){
            servoWrite(LED_OUTPUT,out);
            //      ros::spinOnce();
            sleep(0.4);
            out = out+100;
            if (out > 2500){
                out = 500;
            }        
        }
    }
    return 0;
}

