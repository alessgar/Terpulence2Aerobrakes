clear

tspan = 0:0.1:20;
%x0 = [2150 240]; %7000ft and Mach = 0.7
x0 = [2000 270]; %6000ft and Mach = 0.8
[t,x] = ode45(@stateEquation, tspan, x0);

h = x(:,1)*3.28;
hd = 10000*ones(length(t),1);
v = x(:,2);
v0 = zeros(length(t),1);

figure;

subplot(2,1,1)
plot(t,h);
hold on
plot(t,hd,'-.');
ylabel('Height (ft)');
title('\theta = 30^o')

subplot(2,1,2);
plot(t,v);
hold on
plot(t,v0,'-.');
xlabel('time (s)');
ylabel('Velocity (m/s)');

function dxdt = stateEquation(t,x)
        
    dxdt = zeros(2,1);
    
    h = x(1);
    v = x(2);
    
    %parameters
    m = 20; %kg
    g = 9.8; %m/s2
    %rho = 0.9; %kg/m3
    
    ka = 5e-3; %cd = 0.5 %S = pi*(0.15/2)^2 %6inches
    kb = 2e-2; %cd = 1.1 %S = pi*0.15*0.1   %4 flaps occupying 
                                            %pi/4 and 0.1m long
                                            
    theta = 30; %no input as of now
    
    D = (ka*v*abs(v));
    if(v>0)
        Fb = (kb*v*abs(v)*sind(theta))
    else
        Fb = 0;
    end
    dxdt(1) = v;
    dxdt(2) = -D/m - g - Fb/m;
end