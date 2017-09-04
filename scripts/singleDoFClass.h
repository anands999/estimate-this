#ifndef SINGLEDOFCLASS_H
#define SINGLEDOFCLASS_H

#include <cmath>
#include <iostream>
#include <armadillo>
#include "ros/ros.h"
#include "control_msgs/PidState.h"

class singleDOF{
    private:
        arma::vec state;
        arma::mat A;
        arma::vec B;

        double input;
        double t;

        void state_eq(arma::vec &state_in, arma::vec &state_dot);
    public:

        void rk4_propagate(const double &dt);
        void getState(double &t, double &x, double &xdot);

        singleDOF();
        singleDOF(const double &x0, const double &xdot0, const double &m);
};

#endif
