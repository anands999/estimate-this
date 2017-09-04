#include "pidctrlclass.h"

int PIDController::run(const double &err, control_msgs::PidState &output){
    int status = 1;
    double err_derv=0.0;

    t_prev = t_cur;
    output.header.stamp = ros::Time::now();
    t_cur = ros::Time::now().toSec();
    
    double dt = t_cur - t_prev;
    
    output.error = output.p_error = err; 
    output.p_term = kp*err;

    output.i_error = output.d_error = 0.0;
    output.i_term = output.d_term = 0.0;
    output.i_max = int_up_lim;
    output.i_min = int_low_lim;

    if (ki > 0.0){
        update_int(err, dt);
        output.i_term = err_int;
    }
    if (kd > 0.0){
        status = update_derv(err,dt, err_derv);
        if (status == 1)
            output.error_dot = err_derv;
            output.d_term = kd*err_derv;
    }

    err_prev = err;
    output.output = output.p_term + output.d_term + output.i_term;
    return status;
}

void PIDController::update_int(const double &err, const double &dt){
    
    double Idx = ki*(err + err_prev)/2.0*dt;
    
    if (enable_intlim_flg == 1) {
        if ((err_int + Idx < int_up_lim) && (err_int + Idx > int_low_lim))
            err_int += Idx;
    }
    else{
        err_int += Idx;
    }

}

int PIDController::update_derv(const double &err, const double &dt, double &err_derv){
    if (dt > zero_tol){ 
        err_derv = (err - err_prev)/dt;
        return 1;
    }
    else
        return 0;
}

void PIDController::set_kp(const double &k){ kp=k;}

void PIDController::set_ki(const double &k){ ki=k;}

void PIDController::set_kd(const double &k){ kd=k;}

void PIDController::set_gains(const double &kp_in, const double &ki_in, const double &kd_in){
    set_kp(kp_in);
    set_ki(ki_in);
    set_kd(kd_in);
}

void PIDController::reset_integrator(){ err_prev = 0.0;};

PIDController::PIDController(){
    kp = ki = kd = 0.0; 
    err_int = err_prev = 0.0;
    int_up_lim = int_low_lim = 0.0;
    
    enable_intlim_flg = 0;

    zero_tol = 1e-12;
    t_cur = ros::Time::now().toSec();
    t_prev = t_cur;
}




