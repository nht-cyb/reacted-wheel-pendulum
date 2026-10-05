%% Plot Simulations
% Plot the pendulum tilt angle after running RWIPSimulink1axis.slx.
% theta_p is logged by a To Workspace block as "Structure With Time".
% Note: the current model logs thetap_x / thetap_y (two-axis version);
% rename theta_p below to one of these if it is not in the workspace.

figure('Name', 'Theta_p');
plot(theta_p.time, theta_p.signals.values*180/pi,'LineWidth',2); % rad -> deg
title('Tilt angle of the pendulum : \theta_p');
xlabel('Time, t [s]');
ylabel('Pendulum angle in x-axis, \theta_p [deg]');
grid on
