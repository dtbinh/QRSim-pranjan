% ababujo: Creating a scenario as follows: Plume wrappping algorithm
% change task to reflect different cases
% single point of intersection, multiple intersection and almost the whole
% mesh intersect the plume. implemented with a 3x3 mesh 

clear all
close all
% include simulator
addpath(['..',filesep,'sim']);
addpath(['..',filesep,'controllers']);

% create simulator object
qrsim = QRSim();

state = qrsim.init('TaskPlume_1');

% reminder:
% platforms in N1 -> no sensing features
% platforms in N2 -> senses everything within 10f from it
% platforms in N3 -> senses everything within 5f from it

% create a 2 x cats matrix of control inputs
% column i will contain the 2D NED velocity [vx;vy] in m/s
N = state.task.N4 ;
U = zeros(3,state.task.durationInSteps);
tstart = tic;

for i=1:state.task.durationInSteps
    tloop=tic;
    for src=1:state.task.N4
       if state.platforms{src}.isValid()     % Why this? because when a drone is dead it should not send it's coordinates.
           if state.send_coordinates == 1
               % Send UAVs coordinates. All the coordinates shall be available
               % before the next time quantum starts. We have kept it this way
               % because, in our benchmarks the message processing +
               % transmission delay was much smaller than the timestep of 0.02
               % seconds.
               %msg = uav_message(src, state, "Coordinates", 2);
               state.platforms{src}.broadcast_coordinates(src);
           end
           U(:,src) = [2;0;0];   
       end
    end
    % step simulator
    qrsim.step(U);
    
    if(state.display3dOn)
        % wait so to run in real time
        % this can be commented out obviously
        wait = max(0,state.task.dt-toc(tloop));
        pause(wait);
    end
end


elapsed = toc(tstart);figure();


fprintf('running %d times real time\n',(state.task.durationInSteps*state.DT)/elapsed);