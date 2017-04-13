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

for i=1:state.task.durationInSteps,
    tloop=tic;
    for j=1:state.task.N4,
       if(state.platforms{j}.isValid())                  
          U(:,j) = [2;0;0];   
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
figure


    
for j=1:N,
    X = state.task.vt{j};
    for i=1:size(U,1),
        subplot(size(U,1),1,i);
        plot((1:state.task.durationInSteps)*state.task.dt,U(i,:));
        hold on;
        
            X = state.task.vt{j};
            Y = X(i,:);
            plot((1:state.task.durationInSteps)*state.task.dt,Y,'--r');
         axis([0 state.task.durationInSteps*state.task.dt -6 6]);
         xlabel('t [s]');
         ylabel('[m/s]');
    end
end



fprintf('running %d times real time\n',(state.task.durationInSteps*state.DT)/elapsed);