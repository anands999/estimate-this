#include "imuData.h"


void imuData::imuCallback(const sensor_msgs::Imu &msg)
{
  this->linAcc(0) = msg.linear_acceleration.x;
  this->linAcc(1) = msg.linear_acceleration.y;
  this->linAcc(2) = msg.linear_acceleration.z;

  this->angVel(0) = msg.angular_velocity.x;
  this->angVel(1) = msg.angular_velocity.y;
  this->angVel(2) = msg.angular_velocity.z;
}

void imuData::magMeasCallback(const sensor_msgs::MagneticField &msg)
{
  this->magVec(0) = msg.magnetic_field.x;
  this->magVec(1) = msg.magnetic_field.y;
  this->magVec(2) = msg.magnetic_field.z;
}

imuData::imuData () {
  linAcc = arma::zeros<arma::vec>(3);
  angVel = arma::zeros<arma::vec>(3);
  magVec = arma::zeros<arma::vec>(3);
}




