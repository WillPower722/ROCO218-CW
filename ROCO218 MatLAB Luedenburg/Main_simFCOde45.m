% simulate force controlled pendulum on cart using full non-linear model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% all rights reserved
% Author: Dr. Ian Howard
% Associate Professor (Senior Lecturer) in Computational Neuroscience
% Centre for Robotics and Neural Systems
% Plymouth University
% A324 Portland Square
% PL4 8AA
% Plymouth, Devon, UK
% howardlab.com
% 23/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clean up matlab before launching script
clear all
close all 
clc

A = [0,1; 15.32, 0.3889];
B = [1.563;-0.6077];
C = [1,0];
D = 0;

SYS = ss(A,B,C,D)

%step(SYS)

% pendulum length
L = 1;

% time steps of 100ms for integration
timeStep = 0.01;
% total time for simulation
totalTime = 1;
% build timepoint vector
tspan = 0:timeStep:totalTime;


titleMessage = 'uncontrolled nonlinear sim of FC pendulum on cart';
disp(titleMessage)


% initial conditions
% located at x=0
% velocity =0
% angle  pi (inverted)
% angular velocity = 0.5 rads-1
y0 = [0; 2];

%Declare the Poles for the system
eig(A)
PX = 20*[-1 -1.2];
k = place(A,B,PX)

LU = place(A, C', PX)

% use ode to solve with FCPendOnCart with no control force input u
% representing a force controlled pendulum on a cart
% model introduces slight amount of noise to wont stay balanced
% [t,y] = ode45(@(t,y)SSSimulateV1(y, A, B, k), tspan, y0)

% Euler method
[y, t, xout] = ArduinoSimulate(A, B, C, D, k, LU, tspan, y0)


% for all time point animate the results
range=1;
len = length(tspan);
kickFlag = zeros(1,len);

% get variables
x = 0*y(1, :);    % cart positon
th = y(1, :);   % pendulum angle


% animate pendulum
figure
AnimatePendulumCart((th+pi),  x, L/2, tspan, range, kickFlag, titleMessage);


figure
hold on;
plot (tspan, th, 'r-');
title ("Time and angle correlation")
xlabel("Time");
ylabel("Angle");