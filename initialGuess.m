function [T,X,U] = initialGuess(tmax,epsilon,delta,x0,p,N)

% set time span
tspan = [0 tmax];

% sovler options
opts = odeset('AbsTol',epsilon,'RelTol',epsilon,'Events',@(t,x)( events(x,p) ));

% values of alpha to investigate
alphas = deg2rad(linspace(30,45,100));

% initialize tof
tof = zeros(size(alphas));

% not using scaling
p.t_norm = 1;
p.r_norm = 1;
p.v_norm = 1;
p.alpha_norm = 1;
p.delta_norm = 1;

% loop over alphas
for i = 1:length(alphas)

    % constant control law
    u = [alphas(i); delta];

    % solve
    [t,~] = ode45(@(t,x)( solarSailDynamics(x,u,p) ),tspan,x0,opts);

    % time of flight
    tof(i) = t(end);

    if tof(i) == tmax
        error("Maximum time reached!")
    end

end


% use best alpha
[~,i] = min(tof);

alpha = alphas(i);

% get states and time
sol = ode45(@(t,x)( solarSailDynamics(x,[alpha; delta],p) ),tspan,x0,opts);

% interpolate
T = linspace(0,sol.x(end),N);

X = deval(sol,T);

U = [alpha; delta].*ones(2,length(T));

end

%% Event function to stop at desired radius
function [val,isterm,dir] = events(x,p)

r = x(1:3);

val = norm(r) - p.r_final;
isterm = 1;
dir = [];

end 