#include "state_estimator.h"
#include <BasicLinearAlgebra.h>
#include "aero_bmp.h"
#include "aero_imu.h"
#include "fram.h"
#include "sim_variables.h"

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
  

void updateState(float timeNow){

    // Get sensor data
  height_k = getRelAltitude();  
  acc_k = getRelAccelX(); //it is absolute acceleration! (not relative!)
  theta_k =0.0f; //in rad

  //Sim data
  height_k = (float) sim_Height(timeNow);
  acc_k =    (float) sim_Accel();

/*  
//if timeNow if in input sim otherwise get height and accel
if (timeNow>10.0f) {
   height_k = (float) simulateHeight(timeNow-10.0f);  
   acc_k = (float) simulateAccel(timeNow-10.0f);
} else if (timeNow>30.0f) {
   height_k = 2400.0f;  
   acc_k =0;
   } else {
     height_k = 0.0f;  
   acc_k =-9.81f;
} */
  
      //insertBlankValues(1);
      //framPrint(height_k);

      //insertBlankValues(1);
      //framPrint(acc_k);
      
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


/*
float simulateHeight(float timeNow) {
  // Test code to get height as a function of timeNow
float h;

  h=-5.712028063268500e-18*pow(timeNow,20) +1.58638919134085e-15*pow(timeNow,19)  +-2.03963339274347e-13*pow(timeNow,18)   +1.61097279401726e-11*pow(timeNow,17)  -8.74654469840995e-10*pow(timeNow,16)
  +3.46070057741233e-08*pow(timeNow,15)  -1.03211061692725e-06*pow(timeNow,14) +2.36677401044779e-05*pow(timeNow,13)    -0.000421810515211216*pow(timeNow,12)   +0.00586438366049921*pow(timeNow,11)
  -0.0634379304342509*pow(timeNow,10)    +0.529037766547874*pow(timeNow,9)      -3.34250566958262*pow(timeNow,8)        +15.5422950299675*pow(timeNow,7)         -50.7101186367172*pow(timeNow,6)
  +106.864381898660*pow(timeNow,5)       -123.313329687538*pow(timeNow,4)      +45.3049941055702*pow(timeNow,3)         +53.4638033298709*pow(timeNow,2)        +4.11040674672448*pow(timeNow,1)
  -0.417857259296764; 
h=
  -1.03211061692725e-06*pow(timeNow,14) +2.36677401044779e-05*pow(timeNow,13)    -0.000421810515211216*pow(timeNow,12)   +0.00586438366049921*pow(timeNow,11)
  -0.0634379304342509*pow(timeNow,10)    +0.529037766547874*pow(timeNow,9)      -3.34250566958262*pow(timeNow,8)        +15.5422950299675*pow(timeNow,7)         -50.7101186367172*pow(timeNow,6)
  +106.864381898660*pow(timeNow,5)       -123.313329687538*pow(timeNow,4)      +45.3049941055702*pow(timeNow,3)         +53.4638033298709*pow(timeNow,2)        +4.11040674672448*pow(timeNow,1)
  -0.417857259296764;  

h = 0.5f*30.0005f*pow(timeNow,2);  

return h;
}

float simulateAccel(float timeNow) {
  // Test code to get accel as a function of timeNow
float a;


a=1.17566104537301e-16*pow(timeNow,20) -1.97662506733408e-14*pow(timeNow,19)   +1.37485165115879e-12*pow(timeNow,18)    -4.49097198565159e-11*pow(timeNow,17)   +1.11439571036918e-10*pow(timeNow,16)
  +5.50907929567516e-08*pow(timeNow,15)  -2.71500418887854e-06*pow(timeNow,14) +7.52815104986264e-05*pow(timeNow,13)    -0.00142742653043154*pow(timeNow,12)    +0.0196098570656512*pow(timeNow,11)
  -0.198888899325335*pow(timeNow,10)    +1.48685709336237*pow(timeNow,9)       -8.03363261114509*pow(timeNow,8)         +29.9576482902292*pow(timeNow,7)        -69.1211945242324*pow(timeNow,6)
  +64.3759633958567*pow(timeNow,5)       +106.004289556379*pow(timeNow,4)      -374.110145583987*pow(timeNow,3)         +412.600835330797*pow(timeNow,2)        -185.671454730500*pow(timeNow,1)
  -80.6565766606938;

a=
  -2.71500418887854e-06*pow(timeNow,14) +7.52815104986264e-05*pow(timeNow,13)    -0.00142742653043154*pow(timeNow,12)    +0.0196098570656512*pow(timeNow,11)
  -0.198888899325335*pow(timeNow,10)    +1.48685709336237*pow(timeNow,9)       -8.03363261114509*pow(timeNow,8)         +29.9576482902292*pow(timeNow,7)        -69.1211945242324*pow(timeNow,6)
  +64.3759633958567*pow(timeNow,5)       +106.004289556379*pow(timeNow,4)      -374.110145583987*pow(timeNow,3)         +412.600835330797*pow(timeNow,2)        -185.671454730500*pow(timeNow,1)
  -80.6565766606938; 
   
  a = -30.0f;

  return a;
}
*/
      
