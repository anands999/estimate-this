#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"
#include <iostream>
#include "imuData.h"
#include "AttFiltClass.h"

typedef const boost::function< void(const sensor_msgs::Imu &)> imuCallBack;
typedef const boost::function< void(const sensor_msgs::MagneticField &)> magCallBack;


int main(int argc, char **argv)
{
  
  
  ros::init(argc, argv, "spfRun");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
 
  double P[3];
  double R[3];
  double Q[3];

  P[0] = P[1] = P[2] = 1e-3;
  R[0] = R[1] = R[2] = 1e-6;
  Q[0] = Q[1] = Q[2] = 1e-8;





  // sensor data object
  imuData myData;
  // filter objects
  AttitudeFilter myFilter;
  SPF mySPF(P,Q,R,3);
  // message object
  geometry_msgs::Vector3 rpy_msg;

  imuCallBack imuCallBackFunc = boost::bind(&imuData::imuCallback, &myData, _1);
  magCallBack magCallBackFunc = boost::bind(&imuData::magMeasCallback, &myData, _1);
 
  ros::Subscriber subImu = n.subscribe<sensor_msgs::Imu>("/imu/data_raw",10, imuCallBackFunc);
  ros::Subscriber subMag = n.subscribe<sensor_msgs::MagneticField>("/imu/mag",10, magCallBackFunc);

  ros::Publisher rpyPub = n.advertise<geometry_msgs::Vector3>("/spFilt/rpy",100);

  while (n.ok()){
  
//    myFilter.update(myData.linAcc, myData.magVec);
//    myFilter.writeMsg(rpy_msg);
    if(arma::norm(myData.linAcc) != 0.0 && arma::norm(myData.magVec) != 0.0){
      mySPF.update(10,myData.linAcc, myData.magVec);
      mySPF.writeMsg(rpy_msg);
      rpyPub.publish(rpy_msg);
    }
 
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

