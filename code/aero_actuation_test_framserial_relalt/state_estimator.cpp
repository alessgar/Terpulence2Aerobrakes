#include "state_estimator.h"
#include <BasicLinearAlgebra.h>
#include "aero_bmp.h"
#include "aero_imu.h"
#include "fram.h"

using namespace BLA;

  Matrix<3,3> F_k={0};
  
  Matrix<3> G_k={0};

  Matrix<1,3> H_k = {0};

float sigmaA=0.1f;
float sigmaB=0.0001f;
float Ts =0.1f;

  Matrix<3,3> Q={(Ts*sigmaA)*(Ts*sigmaA)*Ts*Ts/4,   Ts/2*(Ts*sigmaA)*(Ts*sigmaA),  0,
                Ts/2*(Ts*sigmaA)*(Ts*sigmaA),       (Ts*sigmaA)*(Ts*sigmaA)    ,   0,
                             0         ,                     0 ,                sigmaB*sigmaB};

  
  Matrix<3> previousState = {0};
  Matrix<3,3> previousVariance={0.1f/100.0f,  0 ,  0,0   ,0.5f/100.0f , 0 ,0 ,  0  , 1.0f/100.0f};


  //Matrix<3,3>

float tilt_filter = 0.0f;
float height_filter = 0.0f;
float velocity_filter = 0.0f;

	float g = 9.81f;
  int n = 3;
  int p = 1;
  float R=0.0001f;
   float height_k = 0.0f;  
  float acc_k = 0.0f; //it is absolute acceleration! (not relative!)
  float theta_k =0.0f; //in rad

  float u_k=0;

  Matrix<3> xk_minus =  {0};               
Matrix<3,3> Pk_minus ={0};

Matrix<3,1>H_k_T={0};
float S_k = 0;
Matrix<3> K_k = {0};

// weight to previous estimates
Matrix<3,3> I3 = {0};
Matrix<3,3> W_k={0};

//update step (posterior)
Matrix<3>x_k = {0};
Matrix<3,3>P_k = {0};
  

void updateState(){

    // Get sensor data
  height_k = getRelAltitude();  
  acc_k = getRelAccelX(); //it is absolute acceleration! (not relative!)
  theta_k =0.0f; //in rad
  
// Create system matricies
   F_k = 
 { 1,      Ts*cos(theta_k),       -.5*Ts*Ts*(cos(theta_k) + sin(theta_k))   , 
 0,           1,                                  -Ts                     , 
 0,           0,                                   1                    
 };

 G_k = { 0.5*Ts*Ts    ,  Ts  , 0    };

 H_k = { 1    ,  0  , 0    };

// input from IMU for process prediction
u_k= -acc_k - g*cos(theta_k);

//prediction step (prior)
xk_minus =  F_k*previousState + G_k*u_k ;               
Pk_minus = F_k * previousVariance *(~F_k)+Q;

// Kalman gain
H_k_T=~H_k;

S_k = Pk_minus(1,1)+R;
K_k = Pk_minus * H_k_T/S_k;

// weight to previous estimates
 I3 = {1,0,0,0,1,0,0,0,1};
 W_k=I3-K_k*H_k;

//update step (posterior)
x_k = W_k*xk_minus+K_k*height_k;
P_k = W_k*Pk_minus*(~W_k)+K_k*R*(~K_k);

height_filter=x_k(1);
velocity_filter=x_k(2);

 previousState = x_k;
 previousVariance = P_k;
}

float getTilt(){
	return tilt_filter;
}

float getHeight(){
	return height_filter;
}

float getVelocity(){
	return velocity_filter;
}

void outputStateEstimates(){
 // Estimated Height
      insertBlankValues(1);
      framPrint(getHeight());

      insertBlankValues(1);
      framPrint(getVelocity());
}
      
