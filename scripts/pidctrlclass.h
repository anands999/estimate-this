#ifndef PIDCTRLCLASS_H
#define PIDCTRLCLASS_H

#include <cmath>
#include <iostream>
#include "ros/ros.h"
#include "control_msgs/PidState.h"

class PIDController{
    private:
        double kp;
        double ki;
        double kd;

        double err_prev;
        double err_int;
        
        double int_up_lim;
        double int_low_lim;

        double t_prev;
        double t_cur;

        void update_int(const double &err, const double &dt);
        int update_derv(const double &err, const double &dt, double &err_derv);

        double zero_tol;

        int enable_intlim_flg;
        
    public:
        int run(const double &err, control_msgs::PidState &output);
        void set_ki(const double &k);
        void set_kd(const double &k);
        void set_kp(const double &k);
        void set_gains(const double &kp_in, const double &ki_in, const double &kd_in);
        void reset_integrator();
        PIDController();        

};

#endif
