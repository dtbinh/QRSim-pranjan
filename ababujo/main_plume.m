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
evaluate_performance = 1;
eval_network_density = 0;

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
        break;
        dloc = state.platforms{dest_drone}.getX(1:3);
        for radius=10:10:30
            for boff_type = 1:3 % 1->random; 2-> coordinated; 3-> coordinated random
                %                ct  = qrsim.app_unicast_petal_routing(src_drone, dest_drone, petal_width, "no_data", mark_points, update_petal, boff_type, T_ub, radius);
                ct  = qrsim.app_unicast_petal_routing(src_drone, dloc, petal_width, "no_data", mark_points, update_petal, boff_type, T_ub, radius);
                if mark_points == 1; set_invisible_scatter_plots(ct+2);  end
            end
        end
        break;
    end
    
    if eval_network_density == 1
        mark_points = 0;
        number_of_msgs = 10;
        pairs = state.task.furthest_pairs;
        
        type = "petal";
        petal_sizes = 35:10:36;
        boff_type = 3;
        T_ub = 0.001;
        update_petal = 0;
        results = performace_evaluation(qrsim, state, type, pairs, petal_sizes, number_of_msgs, mark_points, update_petal, boff_type, T_ub);
        results(:,4) = state.task.N4;
        csvwrite(sprintf("petal_35w_%d_drones_%d_pairs.csv", state.task.N4, length(pairs)), results);
        
        type = "flooding";
        HTLs = 10:1:10;  % HTL array.
        results_flooding = performace_evaluation(qrsim, state, type, pairs, HTLs, number_of_msgs, mark_points, T_ub);
        results(:,4) = state.task.N4;
        csvwrite(sprintf("flooding_10HTL_%d_%d_pairs.csv", state.task.N4, length(pairs)), results_flooding);
        break;
    end
    
    if evaluate_performance == 1
        
        mark_points = 0;
        number_of_iterations = 1;
        number_of_msgs = 10;
        petal_sizes = 5:10:105;
        min_petal_wid = 0.001;
        HTLs = 1:1:11;  % HTL array.  Make sure the length of petal_sizes and HTLs is equal.

        boff_type = 3;
        T_ub = 0.002;  % seconds
        number_of_rows = length(petal_sizes) * state.task.number_of_pairs;
        
        results_petal = zeros(number_of_rows * number_of_iterations, 10);
        results_petal_1 = zeros(number_of_rows * number_of_iterations, 10);
        results_flooding = zeros(number_of_rows * number_of_iterations, 10);
        fprintf("Iter ct= %d, msgCt = %d, formation= %s, pair_ct= %d", number_of_iterations, number_of_msgs, state.task.formation_type, state.task.number_of_pairs);
        for iter_ct = 1:number_of_iterations
            fprintf("\n Iteration Number %d", iter_ct);
            pairs = state.task.furthest_pairs;
            idx_start = (iter_ct - 1) * number_of_rows ;
            
            type = "petal";  % options are "flooding" and "petal"
            update_petal = 0;
            r = performace_evaluation(qrsim, state, type, pairs, petal_sizes, number_of_msgs, mark_points, update_petal, boff_type, T_ub, min_petal_wid);
            
            results_petal(idx_start+1: idx_start + number_of_rows, :)= r;
            
            update_petal = 1;
            r1 = performace_evaluation(qrsim, state, type, pairs, petal_sizes, number_of_msgs, mark_points, update_petal, boff_type, T_ub, min_petal_wid);
            results_petal_1(idx_start+1: idx_start + number_of_rows, :)= r1;
            
            type = "flooding";
            rf = performace_evaluation(qrsim, state, type, pairs, HTLs, number_of_msgs, mark_points, T_ub, min_petal_wid);
            results_flooding(idx_start+1: idx_start + number_of_rows, :)= rf;            
            state.task.reset()
        end
        
        csvwrite(sprintf("%s_Flood_%d-Pair_%d-Msg_%d-iters.csv", state.task.formation_type, length(pairs(:, 1)), number_of_msgs, number_of_iterations), results_flooding);
        csvwrite(sprintf("%s_petal_%d-Pair_%d-Msg_%d-iters.csv", state.task.formation_type, length(pairs(:, 1)), number_of_msgs, number_of_iterations), results_petal);
        csvwrite(sprintf("%s_petal_upd_%d-Pair_%d-Msg_%d-iters.csv", state.task.formation_type, length(pairs(:, 1)), number_of_msgs, number_of_iterations), results_petal_1);
        
        break;
    end
    
    
    for d1=1:N
        for d2= 1:N
            state.platforms{d1}.uav_coord(:,d2) = [0,0,0];
        end
    end
    
end

elapsed = toc(tstart);figure();

fprintf('running %d times real time\n',(state.task.durationInSteps*state.DT)/elapsed);

function set_invisible_scatter_plots(ct)
dataH = get(gca, 'Children');
for plot_points = 1:(ct)
    set(dataH(plot_points), 'visible', 'off');
end
end

function results = performace_evaluation(qrsim, state, type, pairs, arguments, number_of_msgs, mark_points, update_petal, boff_type, T_ub, min_petal_wid)
results = zeros(length(arguments) * length(pairs(:, 1)), 10);
res_idx = 1;
fprintf("\nEvaluating type = %s\n", type);
for idx = 1:length(pairs(:, 1))
    src = pairs(idx, 1);
    dest = pairs(idx, 2);
    src_dst_dist = pairs(idx, 3);
    if src == dest; continue;   end
    for arg = arguments
        success_count = 0;
        total_number_of_hops = 0;
        overhead_transmissions = 0;
        total_end_to_end_delay = 0;
        avg_end_to_end_delay = seconds(0);
        total_redundant = 0;
        total_tot_delay = 0;
        avg_tot_delay = seconds(0);

        avg_number_hops = 0;
        
        for run_no = 1: number_of_msgs
            switch type
                case "flooding"
                    minor_axis_to_dist_percentage = arg;
                    time_now = datetime('now');
                    msg = uav_message(state, src, dest, arg, "no_data", 2, mark_points);  % arg will be HTL value
                    [scat_ct, tr_ct, success, hop_count, end_to_end_delay, redundant] = qrsim.flood_packet(msg, src, mark_points);
                    tot_delay = datetime('now') - time_now;
                    
                otherwise
                    msg = geo_message(state, src, dest, arg, "no_data", mark_points, update_petal, 0, min_petal_wid); % arg will be petal width
                    %boff_type % 1 is random, 2 is coordinated, 3 is coordinated random.
                    [scat_ct, tr_ct, success, hop_count, end_to_end_delay, redundant] = qrsim.petal_send_message(msg, src, mark_points, boff_type, T_ub);
                    minor_axis_to_dist_percentage = msg.minor_axis/msg.src_dst_dist * 100;
                    tot_delay = seconds(0);
                    
            end
            overhead_transmissions = overhead_transmissions + tr_ct - hop_count;
            success_count = success_count + success;
            total_tot_delay = total_tot_delay + tot_delay;
            if success == 1
                total_number_of_hops = total_number_of_hops + hop_count;
                total_end_to_end_delay = total_end_to_end_delay + end_to_end_delay;
                total_redundant = total_redundant + redundant;
            end
            if mark_points == 1
                set_invisible_scatter_plots(scat_ct+2);
            end
        end
        success_rate = success_count/number_of_msgs * 100;
        if success_count > 0
            avg_number_hops = total_number_of_hops/success_count;
            avg_end_to_end_delay = total_end_to_end_delay / success_count;
            avg_end_to_end_delay.Format = 'hh:mm:ss.SSSSSSSSS';
            avg_tot_delay = total_tot_delay / success_count;
            overhead = overhead_transmissions / success_count; % Number of transmissions per successfully delivered packet. (includes unsuccessfull transmissions).
            avg_redundant = total_redundant /success_count; % Number of duplicate messages the destination heard.
        else
            overhead = overhead_transmissions;
            avg_redundant = 0;
        end
        results(res_idx, : ) = [src, dest, src_dst_dist, round(minor_axis_to_dist_percentage), success_rate, seconds(avg_end_to_end_delay), avg_number_hops, overhead, seconds(avg_tot_delay), avg_redundant];
        %delay_results(res_idx, :) = seconds(avg_end_to_end_delay);
        res_idx = res_idx + 1;
        fprintf("\nS= %d, D= %d, Dist= %f, Petal width ratio= %f, Transmitted pkt ct= %d, Success Rate = %f",...
            src, dest, src_dst_dist, minor_axis_to_dist_percentage, number_of_msgs, success_rate);
        fprintf(" Avg end-to-end delay = %f, Avg number of hops= %f, Overhead= %f, Redundancy= %d", seconds(avg_end_to_end_delay), avg_number_hops, overhead, avg_redundant);
    end
end
end
