#include "imuData.h"


void imuData::imuCallBack(const sensor_msgs::Imu &msg)
{
  
  this->linAcc(0) = msg.linear_acceleration.x;
  this->linAcc(1) = msg.linear_acceleration.y;
  this->linAcc(2) = msg.linear_acceleration.z;

  this->angVel(0) = msg.angular_velocity.x;
  this->angVel(1) = msg.angular_velocity.y;
  this->angVel(2) = msg.angular_velocity.z;


}

imuData::imuData () {
  linAcc = arma::zeros<arma::vec>(3);
  angVel = arma::zeros<arma::vec>(3);
}



