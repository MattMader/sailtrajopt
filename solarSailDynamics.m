function dxdt = solarSailDynamics(x,u,p)
% solarSailDynamics.m Computes the state derivative for a solar sail
% spacecraft given its current state and current control vector.
%
% Note:
%   Function is vectorized for matrices of size(A,2) = N
%
% Inputs:
%   x (6xN) state vector with components [r; v] in ICRF Eq
%   u (2xN) control vector with components [alpha; delta]
%   p (struct) parameter structure
%       p.mu (1x1) standard gravitational parameter of Sun
%       p.beta (1x1) solar sail lightness factor
%
% Outputs:
%   dxdt (6xN) state vector derivative

% initialize output
dxdt = zeros(size(x));

% unpack state
r = x(1:3,:);
v = x(4:6,:);

% unpack control
alpha = u(1,:);
delta = u(2,:);

% unpack parameters
mu = p.mu;
beta = p.beta;

% magnitude of position
R = vecnorm(r);

% sun-line unit vector
r_hat = r./R;

% velocity unit vector
v_hat = v./vecnorm(v);

% angular momentum unit vector
h_hat = cross(r_hat,v_hat);

% solar sail force unit vector
n = cos(alpha).*r_hat + ...
    sin(alpha).*cos(delta).*h_hat + ...
    sin(alpha).*sin(delta).*cross(h_hat,r_hat);

% net acceleration
a = beta*mu./R.^2.*dot(r_hat,n).^2.*n - mu./R.^2.*r_hat;

% state derivatives
dxdt(1:3,:) = v; % r_dot = v
dxdt(4:6,:) = a; % v_dot = a

end % function solarSailDynamics