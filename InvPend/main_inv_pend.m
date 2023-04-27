% Assessment Task 5 
% Assignment 2: Optimal Control
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

clc
clear;

%% Simulation Settings
SN = 12904090; %Insert here your student number

Tf = 60;    %Simulation length

Ts = 1/20;  %Sampling time for the controller

CL = 0;     % Enable closed-loop

obs = 0;    % 0: without  Observer, i.e., -F*x
            % 1: with Observer, i.e., -F*x_hat

noise = 1;  %Enable measurement output noise

controller = 1; %1: LQR
                %2:   MPC
                
N=1;            % Prediction Horizon (increase as required it)

animation = 1;  % 1: Animate the inverter pendulum

%% Input and State Constraints for MPC
% not required for LQR
% change these constraints as required
umin=-15;
umax=15;
theta_max=3*pi/180;            
theta_min=-theta_max;

xmin=[-10000;-10000;theta_min;-10000];    %Large number implies no constraint
xmax=[10000;10000;theta_max;10000];       %Large number implies no constraint

%% Initial Condition
x_o = 0;                    %cart position 
xsp_o = 0;                  %cart speed
th_o = pi;                   %0: pendulum vertically pointing upwards
                            %pi: pendulum vertically pointing downwards
w_o = 0;                    %angular speed of the pendulum

xo = [x_o xsp_o th_o w_o]'; %State initial condition
xo_hat=[0 0 0 0]';          %Observer initial condition

%% System Parameters
disp("System Parameters" + newline)
[Mc,mp,l,g,b] = MySysParam(SN)

disp("Press any key to continue"+ newline)
pause;  

%% Continupus-Time LTI Model
% Please, all of you (especially Alberic)
% Modify the system matrices according to your derived model
n=4;    %Number of states
m=1;    %number of inputs
p=2;    %number of outputs

Ac=[0    1        0                    0
    0   -b/Mc     -mp*g/Mc            0
    0    0        0                    1
    0   b/(Mc*l)   g*(mp + Mc)/(Mc*l)   0];

Bc= [0 1/Mc 0 -1/(Mc*l)]';

Cc=[1 0 0 0;
   0  0 1 0];

%% Controllability
disp('*******************************')
CM=rand(n,n);   %Compute this accordingly
rank_CM = rank(CM);
if (rank_CM==n)
    disp('System is Controllable')
else
    disp('System is Not Controllable')
end

%% Discretization
sys_ct=ss(Ac,Bc,Cc,0);
sys_dt=c2d(sys_ct,Ts);

A=sys_dt.A;
B=sys_dt.B;
C=sys_dt.C;

%% LQR design
%tune your weighting matrices for your controller
Q=diag([1 1 1 1]);
R= 1;

disp("LQR Gain Matrix: " + newline)
[K,P]=dlqr(A,B,Q,R)

AK = A-B*K;
disp(newline + "Closed-loop eigenvalues (poles):")
eig_AK = eig(AK)'

%% Steady-Sate Kalman Filter design
disp(newline + "*******************************")
OM=rand(n,n);   %Compute this accordingly
rank_OM=rank(OM);
if (rank_OM==n)
    disp('System is Observable')
end

%tune your weighting matrices for your Kalman Filter
Qf=eye(n);
Rf = [0.0290,    0.0002;
   0.0002,    0.0000]; %The diagonal of matrix Rf is the sensores covariance

[Pf,po_dt,Kf_t] = dare(A',C',Qf,Rf,[],[]);
%Pf: Lyapunonv matrix for KF
%po_dt: discrete-time observation eigenvalues
%Kf_t: Transpose of Kalman filter gain
Kf=Kf_t'

AL = A-Kf*C;

disp(newline + "Observer eigenvalues (poles): ")
eig_AL=eig(AL)'

disp(newline + "Press any key to continue" + newline)
pause; 

%% MPC setting
% use same Q, R, and P from LQR

% Implement here an algorithm that allows you to compute all the required
% matrices automatically 

%Expanded weighting matrices
QN=rand(N*n,N*n);    
RN=rand(N*m,N*m);

%Expanded system matrices 
Lambda=rand(N*n,n); 
Phi=rand(N*n,N*m);

%Cost function matrices: 
%W and F are correct provided that Phi, Lambda, QN,and RN are also correct
W=Phi'*QN*Phi+RN;
W=(W+W')/2; %to ensure symmetry, i.e., W=W'
F=Phi'*QN*Lambda;

%Bound Constraint
if (N<1)
    N=1;
end
Umax=[];
Umin=[];
Xmax=[];
Xmin=[];
for k=1:N %loop to form the 
    Umax=[Umax;umax]; 
    Umin=[Umin;umin];

    Xmax=[Xmax;xmax];
    Xmin=[Xmin;xmin];
end

%Inequality constraint  AN*U(k) < bN
% This matrix is correct, provided you have properly computed Phi
% Therefore, do not change it.
INm=eye(N*m);
AN=[INm;
   -INm;
    Phi;
   -Phi];

%% bN must be computed inside the controller

 

%% Simulation
disp(newline + "*******************************")
%warning('off','all')
if (CL==0)
    disp("Simulating Open-Loop case..." + newline)
    sim('sim_2023_A2_inv_pend_OPT.slx')
    save('data_OL.mat');  
else
    if (controller==1)
        disp("Simulating LQR case..." + newline)
        sim('sim_2023_A2_inv_pend_OPT.slx');
        save('data_LQR.mat');  
        plot_LQR;
    end
    if (controller==2)
        disp(['MPC with N=',int2str(N)])
        %disp("Checking Matrices...")
        %stop=check_MPC_matrices(W,F,Lambda,AN,Umax,Umin,Xmax,Xmin,N,n,m);
        %if (stop==0)
        %    disp("Matrices have the right dimensions!!!")
            disp("Simulating MPC case..." + newline)
            sim('sim_2023_A2_inv_pend_OPT.slx');
            save('data_MPC.mat');
            plot_MPC;
        %end
    end
end
warning('on','all')
%% Create your own plots Here

%%
if (animation==1)
    
    disp('Press any key to continue with animation')
    pause;
    disp('Animating...') 
    Ndata = length(time);    
    figure(5)
    for k=1:500:Ndata
        animated_inv_pend_uw_clkw(y(k,1),y(k,2),mp,Mc,l);
    end
end
disp('Done!!!')

