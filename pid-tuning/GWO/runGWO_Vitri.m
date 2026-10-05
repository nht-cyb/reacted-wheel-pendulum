%% runGWO_Vitri
% Tune the two PID controllers of the POSITION control model
% (DK_Vitri_Control_PID) with the Grey Wolf Optimizer (GWO).
%
% Decision vector (one wolf) = [Kp1 Kd1 Ki1 Kp2 Kd2 Ki2]
% Fitness = w*ITAE(joint 1) + (1-w)*ITAE(joint 2), see costFunc_vitri.m
%
% GWO keeps the three best wolves found so far (alpha, beta, delta) and
% moves every wolf towards them; the step size `a` shrinks from 2 to 0 so
% the pack goes from exploring the search space to exploiting the best area.
% Reference: Mirjalili et al., Advances in Engineering Software, 2014.

clear all
clc

%% Duong dan (paths) - shared model, cost function and helpers
thisDir = fileparts(mfilename('fullpath'));
addpath(fullfile(thisDir,'..','common'), fullfile(thisDir,'..','common','position'));

%% Thong so GWO (GWO settings)
SearchAgents_no = 20;            % number of wolves
Max_iter = 50;                   % number of iterations
%% Search space of the gains [Kp1 Kd1 Ki1 Kp2 Kd2 Ki2]
lb = [0.1 0.1 0.1 0.1 0.1 0.1];
ub = [100 100 100 100 100 100];
fileNameSimulink = 'DK_Vitri_Control_PID';

%% RunGWO
display(['Grey Wolf Optimization Run']);
dim = 6;
% initialize alpha, beta, and delta_pos (the three leaders of the pack)
Alpha_pos=zeros(1,dim);
Alpha_score=Inf; %change this to -inf for maximization problems

Beta_pos=zeros(1,dim);
Beta_score=Inf; %change this to -inf for maximization problems

Delta_pos=zeros(1,dim);
Delta_score=Inf; %change this to -inf for maximization problems

%Initialize the positions of search agents
Positions = initialization(SearchAgents_no,dim,ub,lb);
GWO_cg_curve=zeros(1,Max_iter); % best cost after each iteration (convergence curve)

l=0;% Loop counter
% Main loop
while l<Max_iter
    display(['Process:',num2str(l/Max_iter*100),'%']);
    for i=1:size(Positions,1)

       % Return back the search agents that go beyond the boundaries of the search space
        Flag4ub=Positions(i,:)>ub;
        Flag4lb=Positions(i,:)<lb;
        Positions(i,:)=(Positions(i,:).*(~(Flag4ub+Flag4lb)))+ub.*Flag4ub+lb.*Flag4lb;

        % Calculate objective function for each search agent:
        % push the gains to the base workspace, where the Simulink Gain
        % blocks read them, then simulate
        assignin('base', 'Kp1',Positions(i,1))
        assignin('base', 'Kd1',Positions(i,2))
        assignin('base', 'Ki1',Positions(i,3))
        assignin('base', 'Kp2',Positions(i,4))
        assignin('base', 'Kd2',Positions(i,5))
        assignin('base', 'Ki2',Positions(i,6))

        [fitness1,fitness2] = costFunc_vitri(0,0,fileNameSimulink);
        w = 0.5; % weight between joint 1 and joint 2
        fitness = w*fitness1 + (1 - w)*fitness2;

            if fitness<Alpha_score
                Alpha_score=fitness; % Update alpha
                Alpha_pos=Positions(i,:);
            end

            if fitness>Alpha_score && fitness<Beta_score
                Beta_score=fitness; % Update beta
                Beta_pos=Positions(i,:);
            end

            if fitness>Alpha_score && fitness>Beta_score && fitness<Delta_score
                Delta_score=fitness; % Update delta
                Delta_pos=Positions(i,:);
            end
    end


    a=2-l*((2)/Max_iter); % a decreases linearly fron 2 to 0

    % Update the Position of search agents including omegas
    for i=1:size(Positions,1)
        for j=1:size(Positions,2)

            r1=rand(); % r1 is a random number in [0,1]
            r2=rand(); % r2 is a random number in [0,1]

            A1=2*a*r1-a; % Equation (3.3)
            C1=2*r2; % Equation (3.4)

            D_alpha=abs(C1*Alpha_pos(j)-Positions(i,j)); % Equation (3.5)-part 1
            X1=Alpha_pos(j)-A1*D_alpha; % Equation (3.6)-part 1

            r1=rand();
            r2=rand();

            A2=2*a*r1-a; % Equation (3.3)
            C2=2*r2; % Equation (3.4)

            D_beta=abs(C2*Beta_pos(j)-Positions(i,j)); % Equation (3.5)-part 2
            X2=Beta_pos(j)-A2*D_beta; % Equation (3.6)-part 2

            r1=rand();
            r2=rand();

            A3=2*a*r1-a; % Equation (3.3)
            C3=2*r2; % Equation (3.4)

            D_delta=abs(C3*Delta_pos(j)-Positions(i,j)); % Equation (3.5)-part 3
            X3=Delta_pos(j)-A3*D_delta; % Equation (3.5)-part 3

            Positions(i,j)=(X1+X2+X3)/3;% Equation (3.7)
        end
    end
    l=l+1;
    GWO_cg_curve(l)=Alpha_score;
end
%% Import data Simulink - keep the best gains in the workspace
Kp1=Alpha_pos(1);
Kd1=Alpha_pos(2);
Ki1=Alpha_pos(3);

Kp2=Alpha_pos(4);
Kd2=Alpha_pos(5);
Ki2=Alpha_pos(6);

%% Xuat ket qua (show results)
figure('Name','Alpha Score','NumberTitle','off');
plot(GWO_cg_curve, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;

% Re-simulate with the best gains and plot reference vs. output
costFunc_vitri(1,1,fileNameSimulink);

display(['The best solution obtained by GWO is : ', num2str(Alpha_pos)]);
