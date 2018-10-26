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
evaluate_node_mobility_effect = 0;
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
pairs = state.task.furthest_pairs;
petal_sizes = 5:10:94;
HTLs = 1:1:9;
number_of_pairs = state.task.number_of_pairs;
number_of_rows = length(petal_sizes) * state.task.number_of_pairs;

number_of_iterations = 30; % state.task.durationInSteps 
number_of_msgs = 30;
min_petal_wid = 2;

total_row_count = number_of_iterations * number_of_rows;

results_pe = zeros(total_row_count, 10);
results_pe_up = zeros(total_row_count, 10);
results_flooding = zeros(total_row_count, 10);

warm_up_iterations = 10;  % Let the UAVs get to full speed before starting performance evaluation.
xx = zeros(total_row_count+ warm_up_iterations, 1);
dist_err = zeros(number_of_iterations + warm_up_iterations, 1);

for kk = 1 : warm_up_iterations + number_of_iterations
    i = kk - warm_up_iterations;
    tloop=tic;
    % Send UAVs coordinates. All the coordinates shall be available
    % before the next time quantum starts. We have kept it this way
    % because, in our benchmarks the message processing +
    % transmission delay was much smaller than the timestep of 0.02
    % seconds.
    if state.send_coordinates == 1
        %qrsim.broadcast_coordinates();
        qrsim.location_table_exchange();
    end
    % step simulator
    qrsim.step(U);
    xx(kk) = norm(state.platforms{1}.getX(7:9));
    if(state.display3dOn)
        % wait so to run in real time
        % this can be commented out obviously
        wait = max(0,state.task.dt-toc(tloop));
        pause(wait);
    end
    if evaluate_node_mobility_effect == 1 && i >= 1
        mark_points = 0;
        boff_type = 3;
        T_ub = 0.002;

        s = pairs(1);
        d = pairs(2);
        true_loc = state.platforms{d}.getX(1:3);
        temp = state.platforms{s}.location_table(d);
        lt_lot = temp{1};
        dist_err(kk) = norm(true_loc - lt_lot);
 
        idx_start = (i - 1) * number_of_rows ;
        idx_end = idx_start + number_of_rows ;

        type = "petal";
        update_petal = 0;
        fprintf("Iter No= %d, Formation= %s, Drone Count = %d, Pair Ct= %d, Scale= %d", i, state.task.formation_type, N, state.task.number_of_pairs, state.dist_scale);
        re_pe = performace_evaluation(qrsim, state, type, pairs, petal_sizes, number_of_msgs, mark_points, update_petal, boff_type, T_ub, min_petal_wid);
        results_pe(idx_start+1: idx_end, :) = re_pe;
        
        update_petal = 1;
        re_pe_up = performace_evaluation(qrsim, state, type, pairs, petal_sizes, number_of_msgs, mark_points, update_petal, boff_type, T_ub, min_petal_wid);
        results_pe_up(idx_start+1: idx_end, :) = re_pe_up;
        
        type = "flooding";
        rf = performace_evaluation(qrsim, state, type, pairs, HTLs, number_of_msgs, mark_points, T_ub, min_petal_wid);
        results_flooding(idx_start+1: idx_end, :)= rf;
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
end

flood = sprintf("SP_%s_Flood_%d-Pair_%d-Msg_%d-iters_%d-scale_%d-minwid.csv", state.task.formation_type, number_of_pairs, number_of_msgs, number_of_iterations, state.dist_scale, min_petal_wid);
pe = sprintf("SP_%s_petal_%d-Pair_%d-Msg_%d-iters_%d-scale_%d-minwid.csv", state.task.formation_type, number_of_pairs, number_of_msgs, number_of_iterations, state.dist_scale, min_petal_wid);
pe_up = sprintf("SP_%s_petal_upd_%d-Pair_%d-Msg_%d-iters_%d-scale_%d-minwid.csv", state.task.formation_type, number_of_pairs, number_of_msgs, number_of_iterations, state.dist_scale, min_petal_wid);
dist_err_file = sprintf("SP_disterr_%s_%d-Pair_%d-Msg_%d-iters_%d-scale_%d-minwid.csv", state.task.formation_type, number_of_pairs, number_of_msgs, number_of_iterations, state.dist_scale, min_petal_wid);

csvwrite(flood, results_flooding); 
csvwrite(pe, results_pe);
csvwrite(pe_up, results_pe_up);
csvwrite(dist_err_file, dist_err);
elapsed = toc(tstart);

fprintf('running %d times real time\n',(state.task.durationInSteps*state.DT)/elapsed);

function set_invisible_scatter_plots(ct)
dataH = get(gca, 'Children');
for plot_points = 1:(ct)
    set(dataH(plot_points), 'visible', 'off');
end
end
