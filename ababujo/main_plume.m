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
unicast = 0;
evaluate_performance = 0;
eval_network_density = 1;

% reminder:
% platforms in N1 -> no sensing features
% platforms in N2 -> senses everything within 10f from it
% platforms in N3 -> senses everything within 5f from it
% Bayesian probabilities of the reason of message loss
% create a 2 x cats matrix of control inputs
% column i will contain the 2D NED velocity [vx;vy] in m/s
N = state.task.N4 ;
U = zeros(3,N);

tstart = tic;

traj_colors = ["black", "red", "blue", "yellow", "green", "cyan", "magenta"];
if state.display3dOn == 1
    for i = 1:N
        state.platforms{i}.setTrajectoryColor(traj_colors(mod(i, length(traj_colors))+1));
    end
end

for i=1:state.task.durationInSteps
    tloop=tic;
    % Send UAVs coordinates. All the coordinates shall be available
    % before the next time quantum starts. We have kept it this way
    % because, in our benchmarks the message processing +
    % transmission delay was much smaller than the timestep of 0.02
    % seconds.
    if state.send_coordinates == 1
        qrsim.broadcast_coordinates();
    end
    % step simulator
    qrsim.step(U);
    if(state.display3dOn)
        % wait so to run in real time
        % this can be commented out obviously
        wait = max(0,state.task.dt-toc(tloop));
        pause(wait);
    end
    
    if unicast == 1
        mark_points = 1;
        pairs = state.task.furthest_pairs;
        src_drone = pairs(1,1);
        dest_drone = pairs(1,2);
        
        %         for HTL = 8:8
        %             ct = qrsim.app_unicast_flooding(src_drone, dest_drone, HTL, "no_data", 2, mark_points);
        %             if mark_points == 1; set_invisible_scatter_plots(ct+2);  end
        %         end
        petal_width = 15;
        min_petal_wid = 0.1;
        T_ub = 0.002;
        update_petal = 0;
        radius = 0;
        for boff_type = 1:1:1 % 1->random; 2-> coordinated; 3-> coordinated random
            ct  = qrsim.app_unicast_petal_routing(src_drone, dest_drone, petal_width, "no_data", mark_points, update_petal, boff_type, T_ub, radius, min_petal_wid);
            if mark_points == 1; set_invisible_scatter_plots(ct+2);  end
        end
        dloc = state.platforms{dest_drone}.getX(1:3);
        for radius=10:10:30
            for boff_type = 1:3 % 1->random; 2-> coordinated; 3-> coordinated random
                %                ct  = qrsim.app_unicast_petal_routing(src_drone, dest_drone, petal_width, "no_data", mark_points, update_petal, boff_type, T_ub, radius);
                ct  = qrsim.app_unicast_petal_routing(src_drone, dloc, petal_width, "no_data", mark_points, update_petal, boff_type, T_ub, radius);
                if mark_points == 1; set_invisible_scatter_plots(ct+2);  end
            end
        end
    end
        
    for d1=1:N
        for d2= 1:N
            state.platforms{d1}.uav_coord(:,d2) = [0,0,0];
        end
    end
    
end

elapsed = toc(tstart);

fprintf('running %d times real time\n',(state.task.durationInSteps*state.DT)/elapsed);

function set_invisible_scatter_plots(ct)
dataH = get(gca, 'Children');
for plot_points = 1:(ct)
    set(dataH(plot_points), 'visible', 'off');
end
end
