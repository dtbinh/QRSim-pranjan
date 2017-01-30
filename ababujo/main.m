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
state = qrsim.init('TaskKeepSpot');

% number of steps we run the simulation for
N = 3000;

%ababujo: creating multiple waypoints
%wp = zeros(7,3);
%for k=1:size(wp,1)
 %   wp(k,:) = state.platforms{1}.getX(1:3)+ [k;2*k;2.5*k]; 
%end
wp = [0 0 -10;4 4 -7; 9 -2 -10];

% creat PID controller omainbject
pid = WaypointPID_ababujo(state.DT);

tstart = tic;
k=1;

for i=1:N,
    tloop=tic;
    % one should always make sure that the uav is valid
    % i.e. no collision or out of area event happened
    %for k=1:size(wp,1)
        if(state.platforms{1}.isValid())
            % compute controls
            U = pid.computeU(state.platforms{1}.getX(),wp(k,:)',0);
            %U = [0;0.02;0.595;0;12];
            % step simulator
            qrsim.step(U);
        end
        % wait so to run in real time
        wait = max(0,state.task.dt-toc(tloop));
        pause(wait);
        
        %ababujo: If the current waypoint is reached go to next
        if(norm(wp(k,:)'-state.platforms{1}.getX(1:3))< 0.2)
                if(k<size(wp,1))
                    k = k+1;
 
                end
                fprintf('current location with k=%d:\n',k);
               disp(state.platforms{1}.getX(1:3));
        end
    %end
end

% get reward
% qrsim.reward();

elapsed = toc(tstart);

fprintf('running %d times real time\n',(N*state.DT)/elapsed);
