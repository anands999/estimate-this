#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include "imuData.h"
#include "AttFiltClass.h"

typedef const boost::function< void(const sensor_msgs::Imu &)> callBack;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "spfRun");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  imuData myData;

  callBack myCallBackFunc = boost::bind(&imuData::imuCallBack, &myData, _1);
  
//  ros::Subscriber sub = n.subscribe<sensor_msgs::Imu>("/imu/data_raw",10, myCallBackFunc);

  
  AttitudeFilter myFilter;
  
  myFilter.w_BI(0)=0.01;
  myFilter.w_BI(1)=0.02;
  myFilter.w_BI(2)=0.03;


  arma::mat test(3,3);

  test = myFilter.skewMat(myFilter.w_BI);

  std::cout << test << std::endl;



 // while (n.ok()){
 // 
 //   ROS_INFO("Outside: %f", myData.linAcc(0));
 //   ros::spinOnce();
 //   loop_rate.sleep();
 // }
  
  return 0;
}

