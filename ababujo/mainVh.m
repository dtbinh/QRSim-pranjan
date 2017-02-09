% bare bones example of use of the QRSim() simulator object with one
% helicopter in order to track a defined velocity profile

clear all
close all
clc

% include simulator
addpath(['..',filesep,'sim']);
% include controllers
addpath(['..',filesep,'controllers']);

% create simulator object
qrsim = QRSim();

% load task parameters and do housekeeping
state = qrsim.init('TaskGotoWP');


% target velocity (in NED coordinates)
vt = [repmat([2;0;-35;0],1,150),repmat([0;1;-20;0],1,150),repmat([0;-1;-16;0],1,150)];

% number of steps we run the simulation for
N = size(vt,2);

pid = VelocityHeightPID(state.DT);

X = zeros(3,N);

tstart = tic;

for i=1:N,
    tloop=tic;
    % one should alway make sure that the uav is valid
    % i.e. no collision or out of area event happened
    if(state.platforms{1}.isValid())
        % compute controls
        U = pid.computeU(state.platforms{1}.getEX(),vt(1:2,i),vt(3,i),vt(4,i));
        state.task.setWP([vt(1:3,i);0]);
        % step simulator
        qrsim.step(U);
        
        X(:,i) = [state.platforms{1}.getEX(18:19);-state.platforms{1}.getEX(20)];
    end
    % wait so to run in real time
    wait = max(0,state.task.dt-toc(tloop));
    pause(wait);
end
% get reward
% qrsim.reward();

elapsed = toc(tstart);

disp(state.platforms{1}.getX(1:3));
fprintf('running %d times real time\n',(N*state.DT)/elapsed);
