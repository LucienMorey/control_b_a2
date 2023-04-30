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

%Car Model
m=1000; %mass
b=50;   %friction

xo_hat=[2 0 2 0]';  

%% Discrete time model
% derived from kinematic equations
% s = s_0 +v *delta_t + 0.5 * a * delta_t^2
% v = u + a*delta_t
% in discrete time, delta t is just Ts
A = [1, Ts, 0, 0;
     0, 1,  0, 0;
     0, 0, 1, Ts;
     0, 0,  0, 1];

C = [1, 0, 0, 0;
     0, 0, 1, 0];

%% Steady-State Kalman Filter Design
Qf=1e-3*eye(4);

Rf=diag([6.6294, 13.3776]);

%
[P,po_dt,Kf_t] = dare(A',C',Qf,Rf,[],[]);
Kf=Kf_t'

%% simulation
disp('Simulating...')
sim('sim_car_gps_x.slx')

%% plot
disp('Plotting...')

figure(101)
%% Add you plots here
plot(xc,yc,"r-",xc_hat, yc_hat, "bo");
