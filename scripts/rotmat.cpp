#include "rotmat.h"

arma::mat skewMat(const arma::vec &a){ 
  arma::mat A_x = arma::zeros<arma::mat>(3,3);
  
  A_x(0,1) = -a(2);
  A_x(1,0) = a(2);
  A_x(0,2) = a(1);
  A_x(2,0) = -a(1);
  A_x(1,2) = -a(0);
  A_x(2,1) = a(0);

  return A_x;
}

arma::vec invSkewMat(const arma::mat &A_x){
  arma::vec a(3);
  
  a(0) = A_x(2,1);
  a(1) = A_x(0,2);
  a(2) = A_x(1,0);

  return a;
}

arma::mat ColToRotMat(const arma::vec &col){
  arma::mat C = arma::expmat(skewMat(col));
  return C;
}

arma::vec RotMatToCol(const arma::mat &C){
  arma::vec col = invSkewMat(arma::real(arma::logmat(C)));
  return col;
}

arma::mat discretePoisson(const arma::mat &C, const arma::vec &w, const double & dt){
  arma::mat Ck = ColToRotMat(w*dt)*C;
  return Ck;
}
