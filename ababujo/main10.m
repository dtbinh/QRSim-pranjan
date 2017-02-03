% bare bones example of use of the QRSim() simulator object with one
% helicopter

clear all
close all

% include simulator
addpath(['..',filesep,'sim']);
% include controllers
addpath(['..',filesep,'controllers']);

% create simulator object
qrsim = QRSim();

% load task parameters and do housekeeping
state = qrsim.init('TaskKeepSpot10_ababujo');

% number of steps we run the simulation for
N = 900;

wp = zeros(3,10);
pids = cell(10,1);

for i=1:10
    wp(:,i) = state.platforms{i}.getX(1:3);
    %wp(2,i) = wp(2,i)+8; %ababujo: wp move 5 units in Y direction
    pids{i} = WaypointPID_ababujo(state.DT);
    % pids{i} = WaypointPID_ababujo(state.DT);
end

%ababujo: crashing scenario check
wp(1,3) = -25;
wp(1,4) = -30;

tstart = tic;

U = zeros(5,10);
for i=1:N,
    tloop=tic;
    for j=1:10
        % compute controls
        U(:,j) = pids{j}.computeU(state.platforms{j}.getEX(),wp(:,j),0);
    end
    % step simulator
    qrsim.step(U);
    
    % wait so to run in real time
    wait = max(0,state.task.dt-toc(tloop));
    pause(wait);
end
    
% get reward
% qrsim.reward();

elapsed = toc(tstart);

fprintf('running %d times real time\n',(N*state.DT)/elapsed);
disp('      X       Y      Z');
for i=1:10
    disp(state.platforms{i}.getX(1:3)');
end    