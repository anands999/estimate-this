#include "singleDoFClass.h"

void singleDOF::state_eq(arma::vec &state_in, arma::vec &state_dot){
    state_dot = A*state_in+B*this->input;
}



void singleDOF::rk4_propagate(const double &dt){
    
    arma::vec tmp = arma::zeros<arma::vec>(2); 
    arma::vec kvec[4];
    
    for (int i=0;i<4;i++){kvec[i] = arma::zeros<arma::vec>(2); }
    
    this->state_eq(state,kvec[0]);
    
    tmp = state + kvec[0]*dt/2.0;
    this->state_eq(tmp, kvec[1]);
    
    tmp = state + kvec[1]*dt/2.0;
    this->state_eq(tmp, kvec[2]);
    
    tmp = state + kvec[2]*dt;
    this->state_eq(tmp, kvec[3]);

    this->state += dt/6.0*(kvec[0]+2.0*(kvec[1]+kvec[2])+kvec[3]);
    t += dt;
}

void singleDOF::getState(double &t, double &x, double &xdot){
    t = this->t;
    x = this->state(0);
    xdot = this->state(1);
}

singleDOF::singleDOF(){
    state = arma::zeros<arma::vec>(2);
    A = arma::zeros<arma::mat>(2,2);
    A(0,1) = 1;

    B = arma::zeros<arma::vec>(2);
    B(1) = 1;

    input = 0;    
    t = 0;
}

singleDOF::singleDOF(const double &x0, const double &xdot0, const double &m){
    singleDOF();
            
    state(0) = x0;
    state(1) = xdot0;
    B(1) = 1/m;
}

