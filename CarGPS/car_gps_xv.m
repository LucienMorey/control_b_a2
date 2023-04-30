% Assessment Task 5 
% Assignment 2: Optimal Filtering
% 
%
% 48580 Control Studio B
% University of Technology Sydney, Australia
% Autumn 2023
%
% Ricardo P. Aguilera
%
% Go Little Rockstar!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;
%%
fgps=10; %GPS data per second
Ts=1/fgps;

m=1000;     %mass
b=50;       %friction

xo_hat=[2 0 0 2 0 0]';  

%% Discrete time model
A = [1, Ts, 0.5*Ts^2, 0, 0,  0;
     0, 1,  Ts,       0, 0,  0;
     0, 0,  1,        0, 0,  0;
     0, 0,  0,        1, Ts, 0.5*Ts^2;
     0, 1,  Ts,       0, 0,  0;
     0, 0,  1,        0, 0,  0];

C = [1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0];

%% Steady-State Kalman Filter Design
Qf=eye(6)

Rf=eye(4);


[P,po_dt,Kf_t] = dare(A',C',Qf,Rf,[],[]);
Kf=Kf_t'

%% simulation
disp('Simulating...')
sim('sim_car_gps_xv.slx')


%% plot
disp('Plotting...')

figure(111)
%% Add you plots here