function [L1,Lc1,L2,Lc2,b,M1,M2,I1,I2,g]=parameter()
% PARAMETER  Physical parameters of the two-link plant used in the PID
% tuning models (DK_Vitri_Control_PID, DK_Quydao_Control_PID).
% Called from the Stateflow/MATLAB-function block inside the Simulink
% models to compute the joint accelerations ddQ, so it must be on the path.
%
% Each link is modelled as a uniform rectangular bar of length L and width b.

% Dac trung hinh hoc (geometric properties)
L1=1; %chieu dai khau 1 (length of link 1) [m]
Lc1=0.5;% Trong tam khau 1 (centre of mass of link 1, from its joint) [m]
L2=1; %chieu dai khau 2 (length of link 2) [m]
Lc2=0.5; % Trong tam khau 2 (centre of mass of link 2) [m]
b=0.2; % Be rong cac khau (width of the links) [m]
M1=1; %khoi luong khau 1 (mass of link 1) [kg]
M2=1; %khoi luong khau 2 (mass of link 2) [kg]
I1=(M1*(L1^2+b^2))/12;% Mo men quan tinh khau 1 (inertia of link 1 about its CoM) [kg.m^2]
I2=(M2*(L2^2+b^2))/12;% Mo men quan tinh khau 2 (inertia of link 2 about its CoM) [kg.m^2]
g=9.81; % gravitational acceleration [m/s^2]
end
