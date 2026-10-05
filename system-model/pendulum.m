%% Initialize system parameters
% Linearised state-space model of the one-axis reaction-wheel inverted
% pendulum (RWIP), around the upright position (theta_p = 0).
%
% States x = [theta_p; theta_p_dot; theta_w_dot]
%   theta_p     : tilt angle of the pendulum body [rad]
%   theta_p_dot : angular velocity of the body [rad/s]
%   theta_w_dot : angular velocity of the reaction wheel [rad/s]
% Input  u = motor current [A] (motor torque = Km*u)
%
% The script builds the model, looks at its poles/zeros and checks
% controllability/observability. Run it before RWIPSimulink1axis.slx so the
% parameters (e.g. Km) exist in the workspace.
close all;clear all;clc;

%% Values
% Comes from Solidworks
lp = 0.077; %m      distance pivot -> centre of mass of the pendulum body
lw = 0.101; %m      distance pivot -> centre of the reaction wheel
mp = 0.1782; %kg    mass of the pendulum body
mw = 0.11528; %kg   mass of the reaction wheel
Ip = 6.8E-3; %kg.m^2   inertia of the pendulum body
Iw = 6.98E-5;  %kg.m^2 inertia of the reaction wheel about its axis
g=9.81; %m.s^-2
Km=7.6E-3; %N.m/A (motor torque constant)
Tmax=4.25E-2; %N.m  maximum motor torque
Imax=5.6; %A        maximum motor current
Crc=0.00102; % viscous friction coefficient of the pendulum pivot
Crw=0.00005; % viscous friction coefficient of the wheel bearing
%tf
s=tf('s');
%% Define the state-space model [A,B,C,D]
% Shorthand used below: J = mp*lp^2 + mw*lw^2 + Ip (total inertia about the pivot)
A=[0 1 0;(mp*lp+mw*lw)*g/(mp*lp^2+mw*lw^2+Ip) -Crc/(mp*lp^2+mw*lw^2+Ip) Crw/(mp*lp^2+mw*lw^2+Ip);(mp*lp+mw*lw)*g/(-(mp*lp^2+mw*lw^2+Ip)) Crc/(mw*lw^2+Ip) -Crw*(Iw+mw*lw^2+Ip)/(Iw*(mw*lw^2+Ip))]; %state matrix
B=[0; -Km/(mp*lp^2+mw*lw^2+Ip); Km*(mp*lp^2+mw*lw^2+Ip+Iw)/(Iw*(mp*lp^2+mw*lw^2+Ip))]; %input matrix
C=eye(3); %output matrix (identity 3x3)
D=zeros(3,1); %feedthrough matrix equal 0
x0=[pi/30;0;0]; %initial condition (6 deg tilt)
y0=[-pi/30;0;0]; %initial condition (-6 deg tilt)

%% Create state-space model for MATLAB (Matrix A,B,C,D are equal to above)
disp('State-space model')
sys=ss(A,B,C,D,'InputName','u', 'OutputName',{'thetap', 'thetapdot', 'thetawdot'},'StateName',{'thetap', 'thetapdot', 'thetawdot'})

%% Calculation about A matrix
disp('T : matrix of the eigenvectors of A')
disp('Ahat : matrix of the eigenvalues of A')
% Returns diagonal matrix Ahat of eigenvalues and matrix T whose columns are the corresponding right eigenvectors
% One eigenvalue is positive -> the upright position is open-loop unstable
[T,Ahat]=eig(A); %T defines the eigenvector & Ahat the eigenvalues
syms x
Charpoly=charpoly(A,x); %characteristic polynomial of matrix A
solve(Charpoly == 0,x); %det(xI-A)=0

%% State-space model to transfer function
disp('State-space model to transfer function')
H=tf(sys) %Transfer function sys
H_zpk=zpk(sys) %conversion to Zero-Pole-Gain form
size(H) %define the number of inputs and outputs
% Pole-zero map of u -> theta_p (magenta), theta_p_dot (blue), theta_w_dot (red)
pzplot(H(1,1),'m',H(2,1),'b',H(3,1),'r')
h=findobj(gca,'type','line');
set(h,'markersize',10,'linewidth',1)
grid on
axis equal

%% Verify the controllability and observability
%Compute controllability matrix
Co=ctrb(A,B);
%Compute observability matrix
Ob=obsv(A,C);
%Controllability (Kalman's criterion)
disp('Rank of Co')
Rco=rank(Co)
if rank(Co) == size(A,1)
    disp('It is controllable')
else
   disp('It is not controllable')
end
%Observability (Kalman's criterion)
disp('Rank of Ob')
Rob=rank(Ob)
if rank(Ob) == size(A,1)
     disp('It is observable')
else
    disp('It is not observable')
end
