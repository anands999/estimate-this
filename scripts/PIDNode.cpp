#include <iostream>
#include "ros/ros.h"
#include "pidctrlclass.h"
#include "control_msgs/PidState.h"

//typedef const boost::function< void(const sensor_msgs::Imu &)> imuCallBack;
//typedef const boost::function< void(const sensor_msgs::MagneticField &)> magCallBack;

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "PIDnode");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
   
//  ros::Subscriber subImu = n.subscribe<sensor_msgs::Imu>("/imu/data_raw",10, imuCallBackFunc);
//  ros::Subscriber subMag = n.subscribe<sensor_msgs::MagneticField>("/imu/mag",10, magCallBackFunc);

    ros::Publisher PID_Pub = n.advertise<control_msgs::PidState>("/PIDNode/ctl_out",10);

    PIDController pitch_controller;
    pitch_controller.set_gains(1.0,0.01,0.02);

    control_msgs::PidState pid_msg;

    while (n.ok()){
 
        pitch_controller.run(0.1,pid_msg);
        std::cout<< "Output: " << pid_msg.output <<std::endl;
        PID_Pub.publish(pid_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
  
    return 0;
}

