#include "AttFiltClass.h"

arma::mat AttitudeFilter::skewMat(const arma::vec &a){ 
  arma::mat A_x = arma::zeros<arma::mat>(3,3);
  A_x(0,1) = -a(2);
  A_x(1,0) = a(2);
  A_x(0,2) = a(1);
  A_x(2,0) = -a(1);
  A_x(1,2) = -a(0);
  A_x(2,1) = a(0);

  return A_x;

}

arma::vec AttitudeFilter::invSkewMat(const arma::mat &A_x){
  arma::vec a(3);
  a(0) = A_x(2,1);
  a(1) = A_x(0,2);
  a(3) = A_x(1,0);

  return a;
}

// TRIAD Function - helper functions defined first

void MatfromVec(const arma::vec &x1, const arma::vec &x2, const arma::vec &x3, arma::mat Xout)
{
  Xout = arma::zeros<arma::mat>(3,3);

  for (int i=0; i<3; i++){
    Xout(i,0) = x1(i);
    Xout(i,1) = x2(i);
    Xout(i,2) = x3(i);
  }

}

void triadBase(const arma::vec &accel, const arma::vec &mag, arma::mat &Cn)
{
  arma::vec x1b = arma::normalise(accel);
  arma::vec x2b = arma::normalise(mag);
  arma::vec v2b = arma::cross(x1b,x2b);

  v2b = arma::normalise(v2b);

  arma::vec v3b = arma::cross(x1b,v2b);
  
  v3b = arma::normalise(v3b);

  MatfromVec(x1b,v2b,v3b,Cn);

}


void AttitudeFilter::triad(const arma::vec &accel, const arma::vec &mag, arma::mat &C_AB)
{
  arma::mat C1(3,3);
  arma::mat C2(3,3);

  arma::vec g0 = arma::zeros<arma::vec>(3);
  arma::vec m0 = arma::zeros<arma::vec>(3);

  g0(2) = 1.0;
  m0(0) = 1.0;
       
  triadBase(accel,mag,C1);
  triadBase(g0,m0,C2);

  C_AB=C1*C2.t();

}

// CONSTRUCTOR

AttitudeFilter::AttitudeFilter () {
  w_BI = arma::zeros<arma::vec>(3);
  C_BI = arma::eye<arma::mat>(3,3);
}



