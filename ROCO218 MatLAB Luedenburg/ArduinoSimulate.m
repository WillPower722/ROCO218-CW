function [y, t, xout] = ArduinoSimulate(A, B, C, D, k, L, t, x0)

% get signal length
len = length(t);

% init output
y = zeros(1,len);
xout = zeros(2,len);

% record the initial state
xout(:, 1) = x0;
x = x0;
Xhat = x0;

%calculate the command
u(1) = C(1) * x(1) * C(2) * x(2);

% calculate output from theta and thetaDot states
y(1) = C(1) * x(1) + C(2) * x(2) + D(1) * u(1);

% for all remaining data points, simulate state space model using
% C-language compatible formulation
for idx = 2:len
    
    % state feedback rule
    u(idx) = - k(1) * x(1) - k(2) * x(2);
    
    % get the duration between updates
    h = t(idx) - t(idx-1);
    
    % calculate state derivative
    xdot(1) = A(1,1) * x(1) + A(1,2) * x(2) + B(1) * u(idx);
    xdot(2) = A(2,1) * x(1) + A(2,2) * x(2) + B(2) * u(idx);
    
    % update the state
    x(1) = x(1) + h * xdot(1);
    x(2) = x(2) + h * xdot(2);
    
    % record the state
    xout(:, idx) = x;
    
    % calculate output from theta and thetaDot states only
    y(idx) = C(1) * x(1) + C(2) * x(2) + D(1) * u(idx);
    
    Yhat = y(idx)-C(1)*Xhat(1) - C(2)*Xhat(2);
    
    Ycorr(1) = L(1)*Yhat;
    Ycorr(2) = L(2)*Yhat;
    
    Xhatdot(1) = A(1,1)*Xhat(1) + A(1,2)*Xhat(2) + B(1)*u(idx) + Ycorr(1);
    Xhatdot(2) = A(2,1)*Xhat(1) + A(2,2)*Xhat(2) + B(2)*u(idx) + Ycorr(2);
    
    Xhat(1) = Xhat(1) + h * Xhatdot(1);
    Xhat(2) = Xhat(2) + h * Xhatdot(2);
end