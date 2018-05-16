function xDot = FCPendOnCart(x, m, M, L, g, d, u)
% force controlled pendulum on a cart

% x is the state
% states are
% x(1) is cart position x
% x(2) is cart velocity x
% x(3) is pendulum angle theta
% x(4) is pendulum angular velocity dtheta/dt

% m is pendulum point mass
% M is cart mass
% L is length from pivot to point mass
% g is acceleration due to gravity
% d is viscous damping
% u is the control input

% sin of pendulum angle theta
Sx = sin(x(3));

% cosine of pendulum angle theta
Cx= cos(x(3));

% recurring denominator term
Denom = m*L*L*(M+m*(1-Cx^2));

% d/dt(cart position) = cart velocity
xDot(1,1) = x(2);

% d/dt(cart velocity)
xDot(2,1) = (1/Denom)*(-m^2*L^2*g*Cx*Sx + m*L^2*(m*L*x(4)^2*Sx - d*x(2))) + m*L*L*(1/Denom)*u;

% d/dt(pendulum angle) = pendulum angular velocity
xDot(3,1) = x(4);

% d/dt(pendulum angular velocity)
xDot(4,1) = (1/Denom)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*x(4)^2*Sx - d*x(2))) - m*L*Cx*(1/Denom)*u +.01*randn;