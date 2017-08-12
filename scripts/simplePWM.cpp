#include "ros/ros.h"
#include "stdio.h"
extern "C"{
    #include "RoboPiLib.h"
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "simplePWM");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);


//  std::cout << "Serial channel: " << argv[1] << std::endl;
// RoboPiInit(argv[1], 115200);
  
  
  while (n.ok()){
    
    servoWrite(1,2500); 
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

