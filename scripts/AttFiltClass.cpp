#include "AttFiltClass.h"



// TRIAD Function - helper functions defined first

void MatfromVec(const arma::vec &x1, const arma::vec &x2, const arma::vec &x3, arma::mat &Xout){
  Xout = arma::zeros<arma::mat>(3,3);

  for (int i=0; i<3; i++){
    Xout(i,0) = x1(i);
    Xout(i,1) = x2(i);
    Xout(i,2) = x3(i);
  }

}

void triadBase(const arma::vec &accel, const arma::vec &mag, arma::mat &Cn){
  arma::vec x1b = arma::normalise(accel);
  arma::vec x2b = arma::normalise(mag);
  arma::vec v2b = arma::cross(x1b,x2b);

  v2b = arma::normalise(v2b);

  arma::vec v3b = arma::cross(x1b,v2b);
  
  v3b = arma::normalise(v3b);

  MatfromVec(x1b,v2b,v3b,Cn);

}


void AttitudeFilter::triad(const arma::vec &accel, const arma::vec &mag, arma::mat &C_AB){
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

void AttitudeFilter::RPYfromDCM(const arma::mat &DCM, double *rpy){
  
  arma::mat C = DCM.t();

  double s1c2     = -C(2,1);
  double c1c2     =  C(2,2);
  double c2c3     =  C(0,0);
  double c2s3     = -C(1,0);
  double s2       =  C(2,0);
  double zero_tol = 1e-8;
  
  if (std::abs(s1c2) > zero_tol && std::abs(c1c2) > zero_tol){
    rpy[0] = atan2(s1c2,c1c2);
    double c1 = cos(rpy[0]);

    if( abs(c1) > zero_tol){
      double c2 = c1c2/c1;
      rpy[1] = atan2(s2,c2);
    }
    else
      rpy[1] = asin(s2);
    
    rpy[2] = atan2(c2s3,c2c3);
  }
  else{
    rpy[1] = asin(s2);
    rpy[0] = atan2(c2s3,c2c3);
    rpy[2] = 0.0;
  }
}


void AttitudeFilter::update(const arma::vec &accel, const arma::vec &mag){
  this->triad(accel,mag,this->C_BI);
  this->RPYfromDCM(this->C_BI, this->rpy);
}

void AttitudeFilter::writeMsg(geometry_msgs::Vector3 &msg){
  msg.x = this->rpy[0]*180/M_PI;
  msg.y = this->rpy[1]*180/M_PI;  
  msg.z = this->rpy[2]*180/M_PI;
}


// CONSTRUCTOR
AttitudeFilter::AttitudeFilter () {
  w_BI = arma::zeros<arma::vec>(3);
  C_BI = arma::eye<arma::mat>(3,3);

  mag0 = arma::zeros<arma::vec>(3);
  grv0 = arma::zeros<arma::vec>(3);

  grv0(2) = 1.0;
  mag0(0) = 1.0;

  rpy[0] = 0.0;
  rpy[1] = 0.0;
  rpy[2] = 0.0;
}


//--------------------------------------------------------------------




void SPF::update(const double &dt, const arma::vec &gk, const arma::vec &mk){
  
  const int L     = 3;
  double KL = double(L) + kap;
  double sqrtLK = sqrt(KL);
  double alpha = 0.5/KL;

  arma::vec mu    = arma::zeros<arma::vec>(6);
  arma::vec z     = arma::zeros<arma::vec>(6);
  arma::vec m_err = arma::zeros<arma::vec>(3);
  arma::vec xk_prop = arma::zeros<arma::vec>(3);
  arma::vec xks_prop[2*L+1];
  arma::vec err = arma::zeros<arma::vec>(3);

  arma::mat S     = arma::zeros<arma::mat>(6,6);
  arma::mat Sigzz = arma::zeros<arma::mat>(6,6);
  arma::mat Sigyy = arma::zeros<arma::mat>(3,3);
  arma::mat Sigxy = arma::zeros<arma::mat>(3,3);

  arma::mat Ckm   = arma::zeros<arma::mat>(3,3);
  arma::mat Ckp   = arma::zeros<arma::mat>(3,3);
  arma::mat Pkp   = arma::zeros<arma::mat>(3,3);
  arma::mat Cmeas = arma::zeros<arma::mat>(3,3);
  arma::mat Cks[2*L+1];
  arma::mat Ck_mean = arma::zeros<arma::mat>(3,3);
  
  Sigzz.submat(0,0,2,2) = Pk;
  Sigzz.submat(3,3,5,5) = Qk;
  
  S = arma::chol(Sigzz);

  mu.subvec(0,2) = xk;

  for (int i=0;i<2*L+1;i++){
//    std::cout << i << std::endl;
    if (i == 2*L){
      z = mu;
      alpha = kap/KL;
    }
    else if(i < L){
      z = mu+sqrtLK*S.submat(0,i,5,i);
//      std::cout << z << std::endl;
    }
    else
      z = mu-sqrtLK*S.submat(0,i,5,i);
      
    Ckm = ColToRotMat(z.subvec(0,2)+z.subvec(3,5));
    Ckp = discretePoisson(Ckm, w_BI, dt);
    xks_prop[i] = RotMatToCol(Ckp);
    xk_prop += alpha*xks_prop[i];
  }

  for (int i=0;i<2*L+1;i++){
    alpha = 0.5/KL;
    if (i == 2*L)
      alpha=kap/KL;
    
    err = xks_prop[i]-xk_prop;
    Pkp += alpha*err*err.t();  
  }

  Sigzz.submat(0,0,2,2) = Pkp;
  Sigzz.submat(3,3,5,5) = Rk;
  
  S = arma::chol(Sigzz); 
  
  triad(gk,mk,Cmeas);
  Ck_mean = arma::eye<arma::mat>(3,3);

  mu = arma::zeros<arma::vec>(6);
  mu.subvec(0,2) = xk_prop;

  arma::vec mki;
  arma::vec gki;

  for (int i=0;i<2*L+1;i++){
    alpha = 0.5/KL;
    if (i==2*L)
      alpha = kap/KL;
    else if (i<L)
      z = mu + sqrtLK*S.submat(0,i,5,i);
    else
      z = mu - sqrtLK*S.submat(0,i,5,i);

    Ckm =  ColToRotMat(z.subvec(0,2));
  
    mki = Ckm*mag0;
    gki = Ckm*grv0;
  
    triad(gki,mki,Ckm);
    Cks[i] = ColToRotMat(z.subvec(3,5))*Ckm;

    Ck_mean *= ColToRotMat(alpha*RotMatToCol(Cks[i]));
  }
  
  for (int i=0;i<2*L+1;i++){
    alpha = 0.5/KL;
    if (i==2*L)
      alpha = kap/KL;
    
    err = xks_prop[i] - xk_prop;
    m_err = RotMatToCol(Cks[i].t() * Ck_mean);

    Sigyy += alpha*(m_err * m_err.t());
    Sigxy += alpha*(err * m_err.t());
    
  }

  arma::mat K_gain = Sigxy * inv(Sigyy);

  m_err = RotMatToCol(Cmeas.t() * Ck_mean);

  xk = xk_prop + K_gain * m_err;
  Pk = Pkp - K_gain * Sigxy.t(); 

  C_BI = ColToRotMat(xk);
  RPYfromDCM(C_BI, rpy);
}



// SPF class constructors
SPF::SPF (){
  xk = arma::zeros<arma::vec>(3);
  Pk = arma::eye<arma::mat>(3,3);
  Qk = arma::eye<arma::mat>(3,3);
  Rk = arma::eye<arma::mat>(3,3);
  kap = 4.0;
}

SPF::SPF (const double *P, const double *Q, const double *R, const int n)
{
  
  xk = arma::zeros<arma::vec>(n);
  Pk = arma::zeros<arma::mat>(n,n);
  Qk = arma::zeros<arma::mat>(n,n);
  Rk = arma::zeros<arma::mat>(n,n);
  kap = 4.0;

  for (int i=0;i<n;i++){
    Pk(i,i) = P[i];
    Qk(i,i) = Q[i];
    Rk(i,i) = R[i];
  }
}

