%% runACO_Quydao
% Tune the two PID controllers of the TRAJECTORY tracking model
% (DK_Quydao_Control_PID) with Ant Colony Optimization for continuous
% domains (ACO_R).
%
% Decision vector (one ant) = [Kp1 Kd1 Ki1 Kp2 Kd2 Ki2]
% Fitness = w*IAE(joint 1) + (1-w)*ITAE(joint 2), see costFunc_Quydao.m
%
% ACO_R keeps an archive of the best solutions found so far, sorted by
% cost. The archive plays the role of the pheromone: every new ant picks
% one archive solution as a guide (better ranked = more likely, Eq. (1)-(2))
% and samples each gain from a Gaussian centred on that guide, whose width
% is the mean distance to the rest of the archive (Eq. (3)). The new ants
% and the archive are merged and only the best Archive_size are kept, so
% the Gaussians shrink as the colony converges.
% Reference: K. Socha, M. Dorigo, "Ant colony optimization for continuous
% domains", European Journal of Operational Research 185 (2008) 1155-1173.
% DOI: 10.1016/j.ejor.2006.06.046

clear all
clc

%% Duong dan (paths) - shared model, cost function and helpers
thisDir = fileparts(mfilename('fullpath'));
addpath(fullfile(thisDir,'..','common'), fullfile(thisDir,'..','common','trajectory'));

%% Thong so ACO (ACO settings)
Archive_size = 50;     % k : number of solutions kept in the archive
Ants_no = 50;          % m : number of new ants (simulations) per iteration
Max_iter = 100;        % number of iterations
q = 0.2;               % locality of the search: small q -> follow the best solutions,
                       % large q -> all archive solutions are used almost equally
zeta = 0.85;           % xi : pheromone evaporation-like factor, scales the Gaussian width
%% Search space of the gains [Kp1 Kd1 Ki1 Kp2 Kd2 Ki2]
lb = [0.1 0.1 0.1 0.1 0.1 0.1];
ub = [100 100 100 100 100 100];
fileNameSimulink = 'DK_Quydao_Control_PID';

%% RunACO
display(['Ant Colony Optimization Run']);
dim = 6;

% Selection probability of each archive rank (fixed, computed once)
% Eq. (1): w_l = 1/(q*k*sqrt(2*pi)) * exp(-(l-1)^2 / (2*q^2*k^2))
ranks = 1:Archive_size;
Weights = 1/(q*Archive_size*sqrt(2*pi)) * exp(-(ranks-1).^2/(2*q^2*Archive_size^2));
Probs = Weights/sum(Weights); % Eq. (2)

% Initialize the archive with random solutions and evaluate them
Archive_pos = initialization(Archive_size,dim,ub,lb);
Archive_score = zeros(Archive_size,1);
for i=1:Archive_size
    assignin('base', 'Kp1',Archive_pos(i,1))
    assignin('base', 'Kd1',Archive_pos(i,2))
    assignin('base', 'Ki1',Archive_pos(i,3))
    assignin('base', 'Kp2',Archive_pos(i,4))
    assignin('base', 'Kd2',Archive_pos(i,5))
    assignin('base', 'Ki2',Archive_pos(i,6))

    [fitness1,fitness2] = costFunc_Quydao(0,0,fileNameSimulink);
    w = 0.5; % weight between joint 1 and joint 2
    Archive_score(i) = w*fitness1 + (1 - w)*fitness2;
end
% Sort the archive from best (row 1) to worst
[Archive_score,idx] = sort(Archive_score);
Archive_pos = Archive_pos(idx,:);

Best_pos = Archive_pos(1,:);
Best_score = Archive_score(1);
ACO_cg_curve=zeros(1,Max_iter); % best cost after each iteration (convergence curve)

l=0;% Loop counter
% Main loop
while l<Max_iter
    display(['Process:',num2str(l/Max_iter*100),'%']);

    % Standard deviation of the Gaussian around each archive solution
    % Eq. (3): sigma_l,j = zeta * sum_e |s_e,j - s_l,j| / (k-1)
    Sigma = zeros(Archive_size,dim);
    for k=1:Archive_size
        Sigma(k,:) = zeta*sum(abs(Archive_pos - Archive_pos(k,:)),1)/(Archive_size-1);
    end

    New_pos = zeros(Ants_no,dim);
    New_score = zeros(Ants_no,1);
    for i=1:Ants_no
        % Choose the guiding solution by roulette wheel on the rank probabilities
        guide = find(rand() <= cumsum(Probs),1,'first');

        % Sample every gain from N(guide value, sigma)
        New_pos(i,:) = Archive_pos(guide,:) + Sigma(guide,:).*randn(1,dim);

        % Return back the ants that go beyond the boundaries of the search space
        New_pos(i,:) = min(max(New_pos(i,:),lb),ub);

        % Calculate objective function for each ant
        assignin('base', 'Kp1',New_pos(i,1))
        assignin('base', 'Kd1',New_pos(i,2))
        assignin('base', 'Ki1',New_pos(i,3))
        assignin('base', 'Kp2',New_pos(i,4))
        assignin('base', 'Kd2',New_pos(i,5))
        assignin('base', 'Ki2',New_pos(i,6))

        [fitness1,fitness2] = costFunc_Quydao(0,0,fileNameSimulink);
        w = 0.5;
        New_score(i) = w*fitness1 + (1 - w)*fitness2;
    end

    % Pheromone update: merge archive and new ants, keep the best Archive_size
    [All_score,idx] = sort([Archive_score; New_score]);
    All_pos = [Archive_pos; New_pos];
    Archive_score = All_score(1:Archive_size);
    Archive_pos = All_pos(idx(1:Archive_size),:);

    Best_pos = Archive_pos(1,:);
    Best_score = Archive_score(1);

    l=l+1;
    ACO_cg_curve(l)=Best_score;
end
%% Import data Simulink - keep the best gains in the workspace
Kp1=Best_pos(1);
Kd1=Best_pos(2);
Ki1=Best_pos(3);

Kp2=Best_pos(4);
Kd2=Best_pos(5);
Ki2=Best_pos(6);

%% Xuat ket qua (show results)
figure('Name','Best Score','NumberTitle','off');
plot(ACO_cg_curve, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;

% Re-simulate with the best gains and plot reference vs. output
costFunc_Quydao(1,1,fileNameSimulink);

display(['The best solution obtained by ACO is : ', num2str(Best_pos)]);
