%% Setup

% clear
clear
% clc

% constants
MU = 2.959e-04; % [au^3/day^2]

%% Inputs

% initial state
r0 = [1; 0; 0]; % [km]
v0 = [0; sqrt(MU/1); 0]; % [km/s]
x0 = [r0; v0];

% control
alpha = deg2rad(38.52); % [rad]
delta = deg2rad(-90); % [rad]
u = [alpha; delta];

% parameters
p.mu = MU; % [km^3/s^2]
p.beta = 0.1; % [-]
p.r_final = 0.48; % [km]

% not using scaling
p.t_norm = 1;
p.r_norm = 1;
p.v_norm = 1;
p.alpha_norm = 1;
p.delta_norm = 1;

% time span
t0 = 0; % [s]
tF = 500; % [s]
tspan = [t0 tF];

% solver options
epsilon = 1e-9; % [-]

%% Solve

opts = odeset('AbsTol',epsilon,'RelTol',epsilon,'Events',@(t,x)( events(x,p) ),'Stats','on');

[t,x] = ode45(@(t,x)( solarSailDynamics(x,u,p) ), tspan, x0, opts);

%% Post

% time of flight
tof = t(end);

fprintf("Time of flight = %0.3g days\n",tof)

dxdt = solarSailDynamics(x.',u,p);

r = x(:,1:3).';
v = x(:,4:6).';
a = dxdt(4:6,:);
alpha = rad2deg(alpha);
delta = rad2deg(delta);

%% Trajectory Plot

figure(1)
plot3(r(1,:), r(2,:), r(3,:),'b--')

xlim([-1.1 1.1])
ylim([-1.1 1.1])
zlim([-1.1 1.1])

title("Trajectory")

xlabel("X [au]")
ylabel("Y [au]")
zlabel("Z [au]")

grid on

%% Velocity Plot

figure(2)
subplot(3,1,1)
plot(t,v(1,:))
grid on
xlabel("Time [days]")
ylabel("Vx [au/day]")

title("Velocity")

subplot(3,1,2)
plot(t,v(2,:))
grid on
xlabel("Time [days]")
ylabel("Vy [au/day]")

subplot(3,1,3)
plot(t,v(3,:))
grid on
xlabel("Time [days]")
ylabel("Vz [au/day]")

%% Acceleration Plot

figure(3)
subplot(3,1,1)
plot(t,a(1,:))
grid on
xlabel("Time [days]")
ylabel("Ax [au/day^2]")
title("Acceleration")

subplot(3,1,2)
plot(t,a(2,:))
grid on
xlabel("Time [days]")
ylabel("Ay [au/day^2]")

subplot(3,1,3)
plot(t,a(3,:))
grid on
xlabel("Time [days]")
ylabel("Az [au/day^2]")


%% Control Plot

figure(4)
subplot(2,1,1)
plot(t,alpha*ones(size(t)))
grid on
xlabel("Time [days]")
ylabel("Alpha [deg]")
title("Control")

subplot(2,1,2)
plot(t,delta*ones(size(t)))
grid on
xlabel("Time [days]")
ylabel("Delta [deg]")

%% Energy Plot

ke = 0.5*vecnorm(v).^2;
pe = -MU./vecnorm(r);
me = ke + pe;

figure(5)
subplot(3,1,1)
plot(t,ke)
grid on
xlabel("Time [days]")
ylabel("Kinetic")
title("Specific Energy")

subplot(3,1,2)
plot(t,pe)
grid on
xlabel("Time [days]")
ylabel("Potential")

subplot(3,1,3)
plot(t,me)
grid on
xlabel("Time [days]")
ylabel("Mechanical")


%% Save Results

N = 25;

T = linspace(t(1), t(end), N);
X = interp1(t,x,T).';
U = u.*ones(2,length(T));

save bestGuess.mat T X U

%% Event function to stop at desired radius
function [val,isterm,dir] = events(x,p)

r = x(1:3);

val = norm(r) - p.r_final;
isterm = 1;
dir = [];

end 