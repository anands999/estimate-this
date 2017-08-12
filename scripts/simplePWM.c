#include "stdio.h"
#include "RoboPiLib.h"

int main(int argc, char *argv[])
{
  
  int output=0;
//  ros::init(argc, argv, "spfRun");
//  ros::NodeHandle n;
//  ros::Rate loop_rate(10);
  printf(argv[1]);
  RoboPiInit("/dev/ttySAC0", 115200);
  printf("Hello world\n");

  printf("Pin 16: %d",output);

  pinMode(16,SERVO);
  output=readMode(16);
  
  while (1){
    servoWrite(16,2500);
//    ros::spinOnce();
//    loop_rate.sleep();
  }
  
  return 0;
}

