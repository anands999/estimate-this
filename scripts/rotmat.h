#ifndef ROTMATFUNCS_H
#define ROTMATFUNCS_H


#include <cmath>
#include <armadillo>


arma::mat skewMat(const arma::vec &a);
arma::vec invSkewMat(const arma::mat &A_x);

arma::mat ColToRotMat(const arma::vec &col);
arma::vec RotMatToCol(const arma::mat &mat);
arma::mat discretePoisson(const arma::mat &C, const arma::vec &w, const double & dt);


#endif
