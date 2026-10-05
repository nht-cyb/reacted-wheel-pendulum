function [cost_value1,cost_value2]=costFunc_Quydao(plotfig1,plotfig2,fileName)
% COSTFUNC_QUYDAO  Cost of one set of PID gains for TRAJECTORY tracking
% (quy dao).
%
% Runs the Simulink model `fileName` with the gains Kp1..Ki2 that the
% optimiser (GWO or ACO) has already put in the base workspace, then scores
% how well each joint tracks its reference trajectory.
%
%   plotfig1, plotfig2 : 1 to plot reference vs. output of joint 1 / 2
%   fileName           : Simulink model name, e.g. 'DK_Quydao_Control_PID'
%   cost_value1        : IAE of joint 1  (inf if the run diverged)
%   cost_value2        : ITAE of joint 2 (inf if the run diverged)
%
% The model logs t, reference1/2 and output1/2 with To Workspace blocks.
% ReturnWorkspaceOutputs is off, so sim() writes them into THIS function's
% workspace.

error_triggered = false;

try % to avoid run time error in case of unstable parameters
    sim(fileName)
catch ME
    % Unstable gains make the states blow up -> give the worst possible cost
    if (strcmp(ME.identifier,'Simulink:Engine:DerivNotFinite'))
        cost_value1 = inf;
        cost_value2 = inf;
        error_triggered = true;
    else
        rethrow(ME)
    end
end

if ~error_triggered

    % ---- Joint 1 -------------------------------------------------------
    err1=reference1-output1;
    [n,~]=size(err1);
    cost_value1=0;
    for i=1:n
        %  cost_value1=cost_value1+(err1(i))^2 ;  % ISE
          cost_value1=cost_value1+abs(err1(i));  % IAE
        %cost_value1=cost_value1+t(i)*abs(err1(i));  % ITAE
       % cost_value1=cost_value1+t(i)*(err1(i))^2;  % MSE
    end
    %   cost_value1=cost_value1/t(n);  % MSE

    if plotfig1
        figure('Name','Q1','NumberTitle','off');
        plot(t,reference1,t,output1)
    end

    % ---- Joint 2 -------------------------------------------------------
    err2=reference2-output2;
    [n,~]=size(err2);
    cost_value2=0;
    for i=1:n
        %  cost_value2=cost_value2+(err2(i))^2 ;  % ISE
        %  cost_value2=cost_value2+abs(err2(i));  % IAE
        cost_value2=cost_value2+t(i)*abs(err2(i));  % ITAE
        %  cost_value2=cost_value2+t(i)*(err2(i))^2;  % MSE
    end
    %   cost_value2=cost_value2/t(n);  % MSE

    if plotfig2
        figure('Name','Q2','NumberTitle','off');
        plot(t,reference2,t,output2)
    end
end
end
