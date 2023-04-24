%% Plotting Results when usin MPC
% Create your MPC plot

clear

load('data_MPC.mat');   % Data stored after simulation

xc=y_n(:,1);            % Noisy measurement of cart position
theta=y_n(:,2)*180/pi;  % Noisy measurement of pendulum angle

xc_hat=x_hat(:,1);
theta_hat=x_hat(:,3)*180/pi;

xc_ref=x_ref(:,1);

figure(2021)
%Create your plot here
