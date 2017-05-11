#ifndef ATTFILTCLASS_H
#define ATTFILTCLASS_H

#include <cmath>
#include <armadillo>
#include "rotmat.h"
#include "geometry_msgs/Vector3.h"

class AttitudeFilter{
  public:
    
    arma::mat C_BI;
    arma::vec w_BI;
    
    arma::vec mag0;
    arma::vec grv0;
    
    double rpy[3];

//    arma::mat skewMat(const arma::vec &a);    
//    arma::vec invSkewMat(const arma::mat &A_x);    
    arma::mat poissonEq(const arma::mat &C, const arma::vec &w);

    void triad(const arma::vec &accel, const arma::vec &mag, arma::mat &C_AB);
    void RPYfromDCM(const arma::mat &DCM, double *rpy); 
    void update(const arma::vec &accel, const arma::vec &mag); 
    void writeMsg(geometry_msgs::Vector3 &msg);


    AttitudeFilter ();

};


class SPF : public AttitudeFilter{
  public:  
    arma::vec xk;
    arma::mat Pk;
    arma::mat Qk;
    arma::mat Rk;
  
    double kap;

    void update(const double &dt, const arma::vec &gk, const arma::vec &mk);
    
    SPF ();
    SPF (const double *P, const double *Q, const double *R, const int n);
};

#endif
