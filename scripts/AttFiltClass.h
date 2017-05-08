#ifndef ATTFILTCLASS_H
#define ATTFILTCLASS_H

#include <math.h>
#include <armadillo>

class AttitudeFilter{
  public:
    
    arma::mat C_BI;
    arma::vec w_BI;

    arma::mat skewMat(const arma::vec &a);    
    arma::vec invSkewMat(const arma::mat &A_x);    
    arma::mat poissonEq(const arma::mat &C, const arma::vec &w);

    void triad(const arma::vec &accel, const arma::vec &mag, arma::mat &C_AB);

    AttitudeFilter ();

};


//class SpfAttFilter : public AttitudeFilter{
//  public:
//    
//    arma::vec xk;
//    arma::mat Pk;
//    arma::mat Qk;
//    arma::mat Rk;
//
////    void runSPF
//
//}

#endif
