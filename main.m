%% Setup

% clear
clear
clc

% add OptimTraj to path
addpath OptimTraj\
addpath chebfun\

% constants
MU = 2.959e-04; % [au^3/day^2]
aud2kms = 1731; % [km/s / au/day]

%% Parameters

% initial state
r0 = [1; 0; 0]; % [km]
v0 = [0; sqrt(MU/1); 0]; % [km/s]
x0 = [r0; v0];

% initial control
alpha = deg2rad(38.5); % [rad]
delta = deg2rad(-90); % [rad] constrain to planar motion
u0 = [alpha; delta];

% parameters
p.mu = MU; % [au^3/day^2]
p.beta = 0.1; % [-]
p.r_final = 0.48; % [au]


%% Problem Bounds

% constrain time to [0, Inf]
problem.bounds.initialTime.low = 0; % [day]
problem.bounds.initialTime.upp = 0; % [day]
problem.bounds.finalTime.low = 0; % [day]
problem.bounds.finalTime.upp = 1000; % [day]

% contrain alpha and detla to [0, 2*pi]
problem.bounds.control.low = [0; -pi/2]; % [rad]
problem.bounds.control.upp = [pi/2; -pi/2]; % [rad]

% constrain starting state to starting conditions
problem.bounds.initialState.low = x0;
problem.bounds.initialState.upp = x0;

% constrain states to realistic values to help narrow space
problem.bounds.state.low = -2*ones(6,1);
problem.bounds.state.upp = 2*ones(6,1);

%% Problem Guess

% % load best guess for constant alpha
% load bestGuess.mat
% 
% problem.guess.time = T;
% problem.guess.state = X;
% problem.guess.control = U;

load runs\minTOFcirc.mat solution

problem.guess.time = solution(end).grid.time;
problem.guess.state = solution(end).grid.state;
problem.guess.control = solution(end).grid.control;


%% Problem Function Handles

% dynamics
problem.func.dynamics = @(t,x,u) ( solarSailDynamics(x,u,p) );

% path objective
% problem.func.pathObj = @(t,x,u) ( ones(size(t)) );
problem.func.bndObj = @(t0,x0,tF,xF) tF;

% boundary constraints
problem.func.bndCst = @(t0,x0,tF,xF) ( solarSailConstraints(xF,p) );

%% Solver Options

% RungeKutta for better initial gues
problem.options(1).method = 'rungeKutta'; % Select the transcription method
problem.options(1).rungeKutta.nSegment = 25;
problem.options(1).rungeKutta.nSubStep = 2;

% NLP Options
problem.options(1).nlpOpt = optimset(...
    'Display','iter',...   % {'iter','final','off'}
    'TolFun',1e-3,...
    'MaxFunEvals',1e5,...   %options for fmincon
    'MaxIter',Inf);

% Trapezoid with finer grid for refinement
problem.options(2).method = 'trapezoid'; % Select the transcription method
problem.options(2).trapezoid.nGrid = 50;  %method-specific options

% NLP Options
problem.options(2).nlpOpt = optimset(...
    'Display','iter',...   % {'iter','final','off'}
    'TolFun',1e-6,...
    'MaxFunEvals',5e5,...   %options for fmincon
    'MaxIter',Inf);

% % Hermite Simpson for final solution
% problem.options(3).method = 'hermiteSimpson'; % Select the transcription method
% problem.options(3).hermiteSimpson.nSegment = 50;  %method-specific options
% 
% % NLP Options
% problem.options(3).nlpOpt = optimset(...
%     'Display','iter',...   % {'iter','final','off'}
%     'TolFun',1e-6,...
%     'MaxFunEvals',1e6,...   %options for fmincon
%     'MaxIter',Inf);

% % Hermite Simpson
% problem.options(1).method = 'hermiteSimpson'; % Select the transcription method
% problem.options(1).hermiteSimpson.nSegment = 25;  %method-specific options

% % Chebyshev
% problem.options(1).method = 'chebyshev'; % Select the transcription method
% problem.options(1).chebyshev.nColPts = 50;  %method-specific options

% % Multi Chebyshev
% problem.options(1).method = 'multiCheb'; % Select the transcription method
% problem.options(1).multiCheb.nColPts = 6;  %method-specific options
% problem.options(1).multiCheb.nSegment = 4;  %method-specific options

% % RungeKutta
% problem.options(1).method = 'rungeKutta'; % Select the transcription method
% problem.options(1).rungeKutta.nSegment = 25;
% problem.options(1).rungeKutta.nSubStep = 2;


%% Solve Problem!

start = tic;
solution = optimTraj(problem);
toc(start)

%% Post

% unpack solution
t = linspace(solution(end).grid.time(1),solution(end).grid.time(end),250);
x = solution(end).interp.state(t);
u = solution(end).interp.control(t);

% variables
r = x(1:3,:);
v = x(4:6,:)*aud2kms;

alpha = rad2deg(u(1,:));
delta = rad2deg(u(2,:));

% time of flight
tof = t(end);

fprintf("Time of flight = %0.3g days\n",tof)

%% 3D Trajectory Plot

figure(1)
plot3(r(1,:), r(2,:), r(3,:),'b--')

xlim([-1.1 1.1])
ylim([-1.1 1.1])
zlim([-1.1 1.1])

title("3D Trajectory")

xlabel("X [au]")
ylabel("Y [au]")
zlabel("Z [au]")

grid on

%% 2D Trajectory Plot

figure(2)
plot(r(1,:), r(2,:), 'b--')

xlim([-1.1 1.1])
ylim([-1.1 1.1])

title("2D Trajectory")

xlabel("X [au]")
ylabel("Y [au]")

grid on

%% Velocity Plot

figure(3)
subplot(3,1,1)
plot(t,v(1,:))
grid on
title("Velocity")
ylabel("Vx [km/s]")

subplot(3,1,2)
plot(t,v(2,:))
grid on
ylabel("Vy [km/s]")

subplot(3,1,3)
plot(t,v(3,:))
grid on
xlabel("Time [s]")
ylabel("Vz [km/s]")


%% Control Plot

figure(4)
subplot(2,1,1)
plot(t,alpha)
grid on
title("Control")
ylabel("Alpha [deg]")

subplot(2,1,2)
plot(t,delta)
grid on
xlabel("Time [days]")
ylabel("Delta [deg]")

%% Error Plot

if strcmp(solution(end).problem.options.method,'trapezoid') || strcmp(solution(end).problem.options.method,'hermiteSimpson')
    % Then we can plot an estimate of the error along the trajectory
    figure(5)
    
    % NOTE: the following commands have only been implemented for the direct
    % collocation(trapezoid, hermiteSimpson) methods, and will not work for
    % chebyshev or rungeKutta methods.
    cc = solution(end).interp.collCst(t);

    err = solution(end).info.error;
    idx = linspace(t(1),t(end),length(err));


    subplot(3,1,1)
    plot(idx,err(1,:),'.')
    title('Position Error')
    ylabel('X [AU]')
    grid on

    subplot(3,1,2)
    plot(idx,err(2,:),'.')
    ylabel('Y [AU]')
    grid on

    subplot(3,1,3)
    plot(idx,err(3,:),'.')
    ylabel('Z [AU]')
    xlabel("Time [days]")
    grid on

    figure(6)

    err(4:6,:) = err(4:6,:)*aud2kms;

    subplot(3,1,1)
    plot(idx,err(4,:),'.')
    title('Velocity Error')
    ylabel('Vx [km/s]')
    grid on

    subplot(3,1,2)
    plot(idx,err(5,:),'.')
    ylabel('Vy [km/s]')
    grid on

    subplot(3,1,3)
    plot(idx,err(6,:),'.')
    ylabel('Vz [km/s]')
    xlabel("Time [s]")
    grid on


end
