% ENAE100
% Simulation for rocket 
clear
clc
close all

global burnTime T0 mf m0 A Cd_rocket Cd_parachute Parachute_area flap_angle time_to_deploy_flaps time_to_retract_flaps;
TT=[10 15 30];
figure
hold on

for i=1:3

time_to_deploy_flaps=TT(i);% (s)
flap_angle=45; %(degrees)
time_to_retract_flaps=100;% (s)

mass_rocket = 28; %Initial mass Kg
m0=mass_rocket; %Kg
mf=mass_rocket-4.7; %less the propellent mass (Kg)
T0=2500; %Average thrust (N)
burnTime=3.9; %burn time
A=.021; %area of rocket (m^2)
Cd_rocket=.52;
Cd_parachute = .8;
Parachute_area= 2; %(guess)%m^2
x0=[0 0];
tspan=[0 200];
tme = 50; %parachute ejection delay

[tout,xout] = ode45(@xdotsimplerocket,tspan,x0,[],tme);

tout=tout(xout(:,1)>0);
temp1=xout(xout(:,1)>0,1);
temp2=xout(xout(:,1)>0,2);
xout=[temp1 , temp2];

subplot(2,1,1)
hold on
plot(tout,xout(:,1))
ylabel('Altitude (m)')
xlabel('Time (s)')
title('Model Rocket Sim')
grid on
subplot(2,1,2)
hold on
plot(tout,xout(:,2))
ylabel('Velocity (m/s)')
xlabel('Time (s)')
grid on

end

subplot(2,1,1)
legend('Flaps deployed at 10s','Flaps deployed at 15s','Flaps not deployed')
subplot(2,1,2)
legend('Flaps deployed at 10s','Flaps deployed at 15s','Flaps not deployed')
function xdot = xdotsimplerocket(t,x,tme)

    global burnTime T0 mf m0 A  Cd_rocket Cd_parachute Parachute_area flap_angle time_to_deploy_flaps time_to_retract_flaps;

    flapArea=.0091; % 6in x 45degreees of the circumference (m)
    
    if t<burnTime
        T=T0;
        m=(mf-m0)/burnTime*t+m0;
        m_dot=(mf-m0)/burnTime;
    else
        T=0;
        m=mf;
        m_dot=0;
    end
    
    if t>tme
        Cd_p=Cd_parachute;
    else
        Cd_p=0;
    end

    if t>time_to_deploy_flaps && t<time_to_retract_flaps
        Cd_flap=4*sind(flap_angle)^3;
    else
        Cd_flap=0;
    end

    xdot(1,1) = x(2);
    xdot(2,1) = (T-sign(x(2))*.5*rho(x(1))*x(2)^2*(Cd_rocket*A+Cd_p*Parachute_area+4*Cd_flap*flapArea)-9.81*m-m_dot*x(2))/m;

end

function density=rho(h)
    p0=101325; %Pa
    T0=288.15; %K
    L=.0065; %Temp lapse rate
    R=8.31446; %Ideal gas constant
    M=.0289652; %molar mass of air

    density = p0*M/R/T0*(1-L*h/T0)^((9.8*M/R/L)-1);
end


