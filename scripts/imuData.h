#ifndef IMUDATA_H
#define IMUDATA_H

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <armadillo>

// TO-DO: Add ROS time object

class imuData {

  public:
    // Data containers
    arma::vec linAcc;
    arma::vec angVel;
    arma::vec magVec;
    
    // Call back functions
    void imuCallback(const sensor_msgs::Imu &msg); 
    void magMeasCallback(const sensor_msgs::MagneticField &msg);

    // Constructor
    imuData ();
};


#endif
