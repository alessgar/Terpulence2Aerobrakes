% constants
%addpath(genpath(cd))
close all
clearvars

%Ts = 0.01s for Telemega %0.05s for SRAD
Ts = 0.01;     %s      %sampling time
sigmaH = 5;    %m      %barometer standard deviation
sigmaA = 0.1;   %m/s^2  %accelerometer std. dev.
sigmaB = 1e-4; %m/s^2  %accelerometer bias std. dev.

% System states
% state is [height, velocity, bias]'
% We are assuming Q, R, and P0 to be diagonal

F = [ 1 Ts -0.5*Ts^2
      0  1    -Ts
      0  0     1   ];

G = [ 0.5*Ts^2
         Ts   
         0    ];

H = [ 1 0 0];

% process noise covariance
Q_a = [ Ts^2/4 Ts/2 %0
         Ts/2    1 ]; %specific to position, velocity 
                      %(constant acceleration model)

%Q_a = [1e-2 0
%         0  1];
Q = [ (Ts^2*sigmaA^2)*Q_a zeros(2,1)
           zeros(1,2)      sigmaB^2 ];

% sensor noise covariance
R = [ 1e-4 ]; %1e-5 %noise in barometer

% initial state estimate covariance
x0 = [0 0 0]';
P0 = [ 0.1  0   0
        0   0.5  0
        0    0   1]*1e-2; 

load('SRAD.mat')
%SRAD=SRAD_Flight_4_2_22;
SRADdata = SRAD{2}; %[time,height,velocity,Gyro_(x,y,z),Acc_(x,y,z)]
timeSRAD = SRADdata(:,1)-SRADdata(1,1);
heightSRAD = SRADdata(:,2);
velocitySRAD = SRADdata(:,3);
acc_xSRAD = -SRADdata(:,7)-9.81; %We shouldn't subtract bias at the beginning

%measurements(2,:) = data(:,7)';

lim = 2500;
load('Telemega.mat');
t = Telemega{1:lim,1} - Telemega{1,1}; %considering only ascent
h = Telemega{1:lim,3};
%h = interp1(timeSRAD,heightSRAD,t);
v = Telemega{1:lim,4};
a_x = Telemega{1:lim,5};
theta = Telemega{1:lim,14};

% figure
% plot(t,h);
% figure
% plot(t,v);
% figure
% plot(t,a_x);
% figure
% plot(t,theta);
%%
measurements(1,:) = h;
measurements(2,:) = -a_x;
%measurements(1,:) = heightSRAD;
%measurements(2,:) = -acc_xSRAD;
%measurements(3,:) = zeros(1,length(timeSRAD));
N = length(measurements);

%y = [height, accelerometer]
%measurements = [ 0  -32.4	-11.1	18.0	22.9	19.5	28.5	46.5	68.9	48.2	56.1	90.5	104.9	140.9	148.0	187.6	209.2	244.6	276.4	323.5	357.3	357.4	398.3	446.7	465.1	529.4	570.4	636.8	693.3	707.3	748.5;
%                u0  39.72	40.02	39.97	39.81	39.75	39.6	39.77	39.83	39.73	39.87	39.81	39.92	39.78	39.98	39.76	39.86	39.61	39.86	39.74	39.87	39.63	39.67	39.96	39.8	39.89	39.85	39.9	39.81	39.81	39.68;];
%measurements(3,:) = zeros(1,length(measurements));
%t = 0:Ts:(N-1)*Ts;            
state = NaN(3,N);
variance = NaN(3,3,N);

state(:,1) = x0;
variance(:,:,1) = P0;

for k=2:N
    
    sensorReadings(1) = measurements(1,k);   %height
    sensorReadings(2) = measurements(2,k-1); %accelerometer
    %sensorReadings(3) = 0;
    sensorReadings(3) = measurements(3,k-1)*pi/180; %theta_k from IMU
    [x_k, P_k] = runKalmanFilter(F,G,H,Q,R,sensorReadings,...
                                 state(:,k-1), variance(:,:,k-1));
    state(:,k) = x_k;
    variance(:,:,k) = P_k; 
end

%%

%Height
figure
hold on
grid on
plot(t,state(1,:),'lineWidth',1.4);
plot(t,measurements(1,:),'--','lineWidth',1.4);
plot(timeSRAD,heightSRAD,'-.m','lineWidth',1.4);
legend('Estimated (EKF)','height from Telemega','SRAD barometer','Location','SouthEast','fontSize',13);
title('Altitude [m]','fontSize',14);
xlabel('time [s]','fontSize',14);
ylabel('Height [m]','fontSize',14);
xlim([0,t(end)]);

%Velocity
figure
grid on
hold on
plot(t,state(2,:),'lineWidth',1.4);
plot(t,v,'--','lineWidth',1.4);
%plot(timeSRAD,velocitySRAD,'-.m','lineWidth',1.4);
ylabel('Velocty [m/s]','fontSize',14);
xlabel('Time [s]','fontSize',14);
legend('Estimated (EKF)','velocity from Telemega','SRAD velocity (MA filter)','Location','NorthEast','fontSize',13);
xlim([0,21.5]);
title('velocity [m/s]','fontSize',14);

%Acceleration
figure
hold on
plot(t,a_x,'lineWidth',1.4);
plot(timeSRAD,acc_xSRAD,'-.m','lineWidth',1.4)
ylabel('a_x [m/s^2]');
xlabel('Time [s]');
xlim([0,t(end)]);
legend('Telemega','SRAD','Location','SouthEast')
grid on

%Tilt
figure
plot(t,theta,'lineWidth',1.4);
title('Tilt off the vertical [deg]');
xlabel('Time [s]');
xlim([0,t(end)]);
grid on

%Bias
figure
plot(t,state(3,:),'lineWidth',1.4);
title('Accelerometer Bias [m/s^2]');
xlabel('Time [s]');
xlim([0,t(end)]);
grid on

%state - [height, velocity, bias]
%sensorReadings - [baro, accelerometer, theta (from IMU)]
function [x_k, P_k] = runKalmanFilter(F_k,G_k,H_k,Q,R,sensorReadings,previousState,previousVariance)
    
    
    g = 9.81; %m/s^2
    n = 3; %number of states
    p = 1; %dimensionality of outputs vector
    
    %extract the sensor input
    height_k = sensorReadings(1); %barometer
    acc_k    = sensorReadings(2); %accelerometer
    theta_k  = sensorReadings(3); %tilt orientation
    
    %Update the system matrices (EKF)
    F_k(1,3) = F_k(1,3)*(cos(theta_k) + sin(theta_k));
    F_k(1,2) = F_k(1,2)*cos(theta_k);
    %"input" from IMU for process prediction
    u_k = -acc_k - g*cos(theta_k);
    
    %prediction step (prior)
    xk_minus = F_k*previousState + G_k*u_k;
    Pk_minus = F_k * previousVariance * F_k' + Q;
    
    %Kalman gain
    S_k = H_k * Pk_minus * H_k' + R; %innovation    (pxp)
    K_k = Pk_minus * H_k' * inv(S_k);  %Kalman gain (nxp)
    
    %weightage to previous estimates (a variable for ease of use)
    W_k = eye(n) - K_k*H_k; %p = 1 in our case      (n*n)
    
    %update step (posterior)
    x_k = W_k * xk_minus + K_k * height_k;
    P_k = W_k * Pk_minus * W_k' + K_k * R * K_k';
    
end