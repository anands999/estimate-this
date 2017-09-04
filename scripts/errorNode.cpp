#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "singleDoFClass.h"

double cmd_sig = 0;
double fdbk_sig = 0;
int new_cmd = 0;
int new_fdbk = 0;

singleDOF pitchDyn;

void cmdSigCallBack(const std_msgs::Float64 &msg){
  cmd_sig = msg.data;
  new_cmd =1;
}

void fdbkSigCallBack(const std_msgs::Float64 &msg){
  fdbk_sig = msg.data;
  new_fdbk = 1;
}

typedef const boost::function< void(const std_msgs::Float64 &msg)> errorNodeCallBackFunc;


int main(int argc, char **argv)
{
  
    ros::init(argc, argv, argv[1]);
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
  
    double error = 0;
    std_msgs::Float64 error_msg;

    if (argc < 3){
        std::cout << "insufficient arguments, must specific command and sensor topics";
        return 0;
    }
        
    std::cout<<"Reference signal topic: "<< argv[2]<< std::endl;
    std::cout<<" Feedback signal topic: "<< argv[3]<< std::endl;
           

    errorNodeCallBackFunc CmdSignalFunc = boost::bind(cmdSigCallBack, _1);
    errorNodeCallBackFunc FdbkSignalFunc = boost::bind(fdbkSigCallBack, _1);

    ros::Subscriber subCmd = n.subscribe<std_msgs::Float64>(argv[2],10, CmdSignalFunc);
    ros::Subscriber subFdbk = n.subscribe<std_msgs::Float64>(argv[3],10, FdbkSignalFunc);
    ros::Publisher errPub = n.advertise<std_msgs::Float64>(strcat(argv[1],"/error"),10);

    while (n.ok()){
        if (new_cmd == 1 && new_fdbk == 1){
            new_cmd = 0;
            new_fdbk = 0;
            error_msg.data = cmd_sig - fdbk_sig;
            errPub.publish(error_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
  
    return 0;
}

