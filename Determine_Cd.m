%determine Cd of the rocket
clear
clc
close all
load('data\5_29_22\SRAD_5_29_22.mat')
load('data\5_29_22\Telemega_5_29_22.mat')

m=30.4; %kg mass after burnout
D=m*(medfilt1(TerpIIFlight4TeleMega.accel_x(1500:2000),20,'truncate')); %drag of the rocket using median fit to smooth data
R = 287.05; %j/kg-K air gas constant
rho = TerpIIFlight4TeleMega.pressure(1500:2000)./R./(287+TerpIIFlight4TeleMega.temperature(1500:2000)); %density
time= TerpIIFlight4TeleMega.time(1500:2000);
speed=TerpIIFlight4TeleMega.speed(1500:2000);
area=0.01929028; % reference area m^2

plot(time,D,'--')
hold on
Cd=[.7 .75 .8];
for i=1:length(Cd)
plot(time,Cd(i)*.5*rho.*speed.^2*area)
end
% legend('Measured','Cd=.1','Cd=.2','Cd=.3','Cd=.4','Cd=.5','Cd=.6','Cd=.7','Cd=.8','Cd=.9','Cd=1')
legend('Measured','Cd=.7','Cd=.75','Cd=.8')

