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
A=rand(4,4);

C=rand(2,4);

%% Steady-State Kalman Filter Design
Qf=eye(4)

Rf=eye(2);

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
plot(time,xc_n,"r-",time, xc, "bo");
