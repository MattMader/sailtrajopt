%%

% figure(1)
% 
% plot(X(1,:),X(2,:),'b.--','MarkerSize',15)
% hold on
% plot(X(1,1),X(2,1),'ko','MarkerFaceColor','g')
% plot(X(1,end),X(2,end),'ko','MarkerFaceColor','r')
% plot(0,0,'ko','MarkerFaceColor','y')
% 
% grid on
% xlim([-1.1 1.1])
% ylim([-1.1 1.1])
% 
% title("Initial Guess Trajectory")
% xlabel("X [AU]")
% ylabel("Y [AU]")
% 
% legend(["Trajectory" "Initial Position" "Final Position" "Sun"],'Location','best')
% 
% hold off


%% 

clear
close all
clc

load runs\minTOF.mat

MU = 2.959e-04; % [au^3/day^2]
aud2kms = 1731; % [km/s / au/day]

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
