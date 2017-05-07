#ifndef IMUDATA_H
#define IMUDATA_H

#include "sensor_msgs/Imu.h"
#include <armadillo>

// TO-DO: Add ROS time object

class imuData {

  public:
    arma::vec linAcc;
    arma::vec angVel;
    void imuCallBack(const sensor_msgs::Imu &msg); 
    
    imuData ();
};


#endif
