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
% Bayesian probabilities of the reason of message loss
% create a 2 x cats matrix of control inputs
% column i will contain the 2D NED velocity [vx;vy] in m/s
N = state.task.N4 ;
U = zeros(3,N);

tstart = tic;

traj_colors = ["black", "red", "blue", "yellow", "green", "cyan", "magenta"];
for i = 1:N
    state.platforms{i}.setTrajectoryColor(traj_colors(mod(i, length(traj_colors))+1));
end

unicast = 1;
evaluate_performance = 0;

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
        src_drone = 1;
        dest_drone = 36;
        
        for HTL = 1:6
            ct = qrsim.app_unicast_flooding(src_drone, dest_drone, HTL, "no_data", 2, mark_points);
            if mark_points == 1; set_invisible_scatter_plots(ct+2);  end
        end
        
        petal_width = 10;
        T_ub = 0.002;
        update_petal = 1;
        for update_petal=0:1
            for boff_type = 1:3 % 1->random; 2-> coordinated; 3-> coordinated random
                ct  = qrsim.app_unicast_petal_routing(src_drone, dest_drone, petal_width, "no_data", mark_points, update_petal, boff_type, T_ub);
                if mark_points == 1; set_invisible_scatter_plots(ct+2);  end
            end
        end
    end
    
    if evaluate_performance == 1
        mark_points = 0;
        update_petal = 0;
        pairs = state.task.furthest_pairs;
        number_of_msgs = 10;
        
         type = "petal";  % options are "flooding" and "petal"
         petal_sizes = 10:10:60;
         results_petal = performace_evaluation(qrsim, state, type, pairs, petal_sizes, number_of_msgs, mark_points, update_petal);
         plot_graph(results_petal, petal_sizes, pairs, "3D Petal Routing", type);
        
        type = "flooding";
        HTLs = 1:1:6;  % HTL array.
        results_flooding = performace_evaluation(qrsim, state, type, pairs, HTLs, number_of_msgs, mark_points);
        plot_graph(results_flooding, HTLs, pairs, "Limited Flooding", type);
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

function results = performace_evaluation(qrsim, state, type, pairs, arguments, number_of_msgs, mark_points, update_petal)
    results = zeros(length(arguments) * length(pairs(:, 1)), 8);
    res_idx = 1;
    fprintf("\n Evaluating type = %s\n", type);
    for idx = 1:length(pairs(:, 1))
        src = pairs(idx, 1);
        dest = pairs(idx, 2);
        src_dst_dist = pairs(idx, 3);
        if src == dest; continue;   end
        for arg = arguments
            success_count = 0;
            total_number_of_hops = 0;
            total_transmissions = 0;
            total_end_to_end_delay = 0;
            avg_end_to_end_delay = seconds(50);
            avg_number_hops = 50;
            overhead = inf;

            for run_no = 1: number_of_msgs
                switch type
                    case "flooding"
                        minor_axis_to_dist_percentage = arg;
                        msg = uav_message(state, src, dest, arg, "no_data", 2, mark_points);  % arg will be HTL value
                        [scat_ct, tr_ct, success, hop_count, end_to_end_delay] = qrsim.flood_packet(msg, src, mark_points);                    
                    otherwise
                        msg = geo_message(state, src, dest, arg, "no_data", mark_points, update_petal); % arg will be petal width
                        boff_type = 1;
                        T_ub = 0.002;   % seconds.
                        [scat_ct, tr_ct, success, hop_count, end_to_end_delay] = qrsim.petal_send_message(msg, src, mark_points, boff_type, T_ub);
                        minor_axis_to_dist_percentage = msg.minor_axis/msg.src_dst_dist * 100;
                end
                total_transmissions = total_transmissions + tr_ct;
                success_count = success_count + success;
                if success == 1
                    total_number_of_hops = total_number_of_hops + hop_count;
                    total_end_to_end_delay = total_end_to_end_delay + end_to_end_delay;
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
                overhead = total_transmissions / success_count; % Number of transmissions per successfully delivered packet. (includes unsuccessfull transmissions).
            end
            results(res_idx, : ) = [src, dest, src_dst_dist, round(minor_axis_to_dist_percentage), success_rate, seconds(avg_end_to_end_delay), avg_number_hops, overhead];
            %delay_results(res_idx, :) = seconds(avg_end_to_end_delay);
            res_idx = res_idx + 1;
            fprintf("\nS= %d, D= %d, Dist= %f, Petal width ratio= %f, Transmitted pkt ct= %d, Success Rate = %f",...
                src, dest, src_dst_dist, minor_axis_to_dist_percentage, number_of_msgs, success_rate);
            fprintf(" Avg end-to-end delay = %f, Avg number of hops= %f, Overhead= %f", seconds(avg_end_to_end_delay), avg_number_hops, overhead);
        end
    end
end


function plot_graph(results, args, pairs, name)
    x = args;
    figure('Name', name, 'NumberTitle','off');
    if type == "petal"
        xlabel_text = "'Petal-width' to 'src-dst distance', '%'";
    else
        xlabel_text = "HTL";
    end

    subplot(2,2,1);
    y = zeros(length(args), length(pairs(:, 1)));
    for i = 1: length(args)
        y(i, :) = results(i: length(args): length(results), 5);
    end
    bar(x, y);
    title("Success rate");
    xlabel(xlabel_text);
    ylabel("Success rate, '%'");
    
    subplot(2,2,2);
    y2 = zeros(length(args), length(pairs(:, 1)));
    for i = 1: length(args)
        y2(i, :) = results(i: length(args): length(results), 6);
    end
    bar(x, y2);
    title("Average end to end delay");
    xlabel(xlabel_text);
    ylabel("Delay 'seconds'");
    
    subplot(2,2,3);
    y3 = zeros(length(args), length(pairs(:, 1)));
    for i = 1: length(args)
        y3(i, :) = results(i: length(args): length(results), 7);
    end
    bar(x, y3);
    title("Average Number of hops");
    xlabel(xlabel_text);
    ylabel("Hop count: 50 == inf");
    
    subplot(2,2,4);
    y4 = zeros(length(args), length(pairs(:, 1)));
    for i = 1: length(args)
        y4(i, :) = results(i: length(args): length(results), 8);
    end
    bar(x, y4);
    title("Overhead");
    xlabel(xlabel_text);
    ylabel("%, 50 == inf");


end