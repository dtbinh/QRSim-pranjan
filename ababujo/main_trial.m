% ababujo: Creating a scenario as follows: Three drones moving towards an obstacle(wall) 
% and on the other side of the obstacle, two drones are moving towards each other. 
% One of the 3 drones does not have any detection capability. 
% The second drone can detect anything within a 5 feet distance from it, and when it does it comes to a sudden stop. 
% The third drone can detect anything within 10feet, and when it does- starts moving the other way. 
% The two drones on the other side of the wall should try to avoid the pending collision in someway.

clear all
close all
% include simulator
addpath(['..',filesep,'..',filesep,'sim']);
addpath(['..',filesep,'..',filesep,'controllers']);

% create simulator object
qrsim = QRSim();

state = qrsim.init('TaskTrial');

% reminder:
% platforms in N1 -> no sensing features
% platforms in N2 -> senses everything within 10f from it
% platforms in N3 -> senses everything within 5f from it

% create a 2 x cats matrix of control inputs
% column i will contain the 2D NED velocity [vx;vy] in m/s
N = state.task.N1 + state.task.N2 + state.task.N3;
U = zeros(3,N);
tstart = tic;

for i=1:state.task.durationInSteps,
    tloop=tic;
    k = 1;
    for j=1:state.task.N1,
       if(state.platforms{k}.isValid())                  
          U(:,k) = [-2;0;0];
       end
       k = k+1;
    end
    for j=1:state.task.N2,
       if(state.platforms{k}.isValid())                  
          U(:,k) = [-2;0;0];     
       end
       k = k+1;
    end
    for j=1:state.task.N3,
       if(state.platforms{k}.isValid())                  
          U(:,k) = state.platforms{k}.getX(1:3) - [5;0;0];     
       end
       k = k+1;
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


elapsed = toc(tstart);

fprintf('running %d times real time\n',(state.task.durationInSteps*state.DT)/elapsed);