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
        src_drone = 1;
        dest_drone = 36;
        
        %         for HTL = 8:8
        %             ct = qrsim.app_unicast_flooding(src_drone, dest_drone, HTL, "no_data", 2, mark_points);
        %             if mark_points == 1; set_invisible_scatter_plots(ct+2);  end
        %         end
        petal_width = 5;
        T_ub = 0.002;
        update_petal = 1;
        radius = 0;
        for boff_type = 1:3 % 1->random; 2-> coordinated; 3-> coordinated random
            ct  = qrsim.app_unicast_petal_routing(src_drone, dest_drone, petal_width, "no_data", mark_points, update_petal, boff_type, T_ub, radius);
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
        
        pairs = state.task.furthest_pairs;
        number_of_msgs = 100;
        
        type = "petal";  % options are "flooding" and "petal"
        petal_sizes = 5:10:105;
        boff_type = 3;
        T_ub = 0.002;  % seconds
        
        update_petal = 0;
        results_petal = performace_evaluation(qrsim, state, type, pairs, petal_sizes, number_of_msgs, mark_points, update_petal, boff_type, T_ub);
        
        update_petal = 1;
        results_petal_1 = performace_evaluation(qrsim, state, type, pairs, petal_sizes, number_of_msgs, mark_points, update_petal, boff_type, T_ub);
        
        type = "flooding";
        HTLs = 1:1:11;  % HTL array.
        results_flooding = performace_evaluation(qrsim, state, type, pairs, HTLs, number_of_msgs, mark_points, T_ub);
        plot_graph_flooding(results_flooding, HTLs, pairs, "Flooding");
        
        plot_combined_graph(results_petal, results_petal_1, results_flooding, petal_sizes, pairs, "3D petal routing");
        
        %qrsim.state.reset()
        csvwrite(sprintf("Flood_%s_%dPair_%dMsg.csv", state.task.formation_type, length(pairs(:, 1)), number_of_msgs), results_flooding);
        csvwrite(sprintf("petal_%s_%dPair_%dMsg.csv", state.task.formation_type, length(pairs(:, 1)), number_of_msgs), results_petal);
        csvwrite(sprintf("petal_1_mesh_10Pair_100Msg.csv", state.task.formation_type, length(pairs(:, 1)), number_of_msgs), results_petal_1);
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

function results = performace_evaluation(qrsim, state, type, pairs, arguments, number_of_msgs, mark_points, update_petal, boff_type, T_ub)
results = zeros(length(arguments) * length(pairs(:, 1)), 10);
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
                    msg = geo_message(state, src, dest, arg, "no_data", mark_points, update_petal, 0); % arg will be petal width
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

function plot_combined_graph(petal_results, petal_up_results, ~, args, pairs, name)
x = args;
xlabel_text = "'Petal-width' to 'src-dst distance', '%'";
figure('Name', name, 'NumberTitle', 'off');

subplot(2,2,1);
dr1 = zeros(length(args), length(pairs(:, 1)));
dr2 = zeros(length(args), length(pairs(:, 1)));
for i = 1: length(args)
    dr1(i, :) = petal_results(i: length(args): length(petal_results(:, 1)), 5);
    dr2(i, :) = petal_up_results(i: length(args): length(petal_up_results(:, 1)), 5);
end

errorbar(x, mean(dr1,2), std(dr1,0,2), 'LineStyle', '--', 'DisplayName', "Don't update petal");
hold on;
errorbar(x, mean(dr2,2), std(dr2,0,2), 'LineStyle', '-', 'DisplayName', 'Update petal');
title("Delivery Rate");
xlabel(xlabel_text);
ylabel("Delivery Rate, '%'");
ylim([0 110]);
yticks(0:10:100)
xlim([0 inf])
xticks([0, x])
legend('Location','southeast');
grid on

subplot(2,2,2);
delay = zeros(length(args), length(pairs(:, 1)));
delay_1 = zeros(length(args), length(pairs(:, 1)));

for i = 1: length(args)
    delay(i, :) = petal_results(i: length(args): length(petal_results(:, 1)), 6);
    delay_1(i, :) = petal_up_results(i: length(args): length(petal_up_results(:, 1)), 6);
end
hold on;
errorbar(x, mean(delay,2), std(delay,0,2), 'LineStyle', '--', 'DisplayName', "Don't update petal");
errorbar(x, mean(delay_1,2), std(delay_1,0,2),'LineStyle', '-', 'DisplayName', "Update petal");
grid on
title("Average end to end delay");
xlabel(xlabel_text);
ylabel("Delay 'seconds'");
ylim([0 0.25]);
yticks(0:0.02:0.25);
xticks([0, x])
xlim([0 inf])
legend();

subplot(2,2,3);
hops = zeros(length(args), length(pairs(:, 1)));
hops_1 = zeros(length(args), length(pairs(:, 1)));
for i = 1: length(args)
    hops(i, :) = petal_results(i: length(args): length(petal_results(:, 1)), 7);
    hops_1(i, :) = petal_up_results(i: length(args): length(petal_up_results(:, 1)), 7);
end
hold on;
errorbar(x, mean(hops,2), std(hops,0,2), 'LineStyle','--', 'DisplayName', "Don't update petal");
errorbar(x, mean(hops_1,2), std(hops_1,0,2), 'LineStyle', '-', 'DisplayName', "Update petal");
title("Average Number of hops");
xlabel(xlabel_text);
ylabel("Average Number of Hops");
ylim([0 15]);
yticks(0:1:15);
xticks([0, x])
xlim([0 inf])
legend();
grid on

subplot(2,2,4);
overhead = zeros(length(args), length(pairs(:, 1)));
overhead_1 = zeros(length(args), length(pairs(:, 1)));

for i = 1: length(args)
    overhead(i, :) = petal_results(i: length(args): length(petal_results(:, 1)), 8);
    overhead_1(i, :) = petal_up_results(i: length(args): length(petal_up_results(:, 1)), 8);
end
hold on;
errorbar(x, mean(overhead,2), std(overhead,0,2),'LineStyle', '--', 'DisplayName', "Don't update header");
errorbar(x, mean(overhead_1,2), std(overhead_1,0,2), 'LineStyle', '-','DisplayName', 'Update header');
grid on
title("Average Number of Transmissions");
xlabel(xlabel_text);
ylabel("Average number of transmissions");
ylim([0 100]);
yticks(0:10:100);
xlim([0 inf])
xticks([0 x])
legend();
end

function plot_graph_flooding(results, args, pairs, name)
x = args;
figure('Name', name, 'NumberTitle','off');
xlabel_text = "HTL";

subplot(2,2,1);
y = zeros(length(args), length(pairs(:, 1)));
for i = 1: length(args)
    y(i, :) = results(i: length(args): length(results(:, 1)), 5);
end
errorbar(x, mean(y,2), std(y,0,2), 'LineStyle', '-', 'DisplayName', "Flooding");
title("Delivery Rate");
xlabel(xlabel_text);
ylabel("Delivery Rate, '%'");
ylim([0 110]);
yticks(0:10:100)
xlim([0 inf])
xticks([0, x])
legend('Location','southeast');
grid on


subplot(2,2,2);
y1 = zeros(length(args), length(pairs(:, 1)));
y2 = zeros(length(args), length(pairs(:, 1)));
for i = 1: length(args)
    y1(i, :) = results(i: length(args): length(results(:, 1)), 9);
    y2(i, :) = results(i: length(args): length(results(:, 1)), 6);
end
%bar(x, y2);
hold on;
errorbar(x, mean(y2,2), std(y2,0,2), 'LineStyle', '-', 'DisplayName', "End-to-end Delay");
errorbar(x, mean(y1,2), std(y1,0,2), 'LineStyle', '--', 'DisplayName', "Total-flooding-time");
grid on
title("Average end to end delay");
xlabel(xlabel_text);
ylabel("Delay 'seconds'");
% ylim([0 0.25]);
% yticks(0:0.02:0.25);
xticks([0, x])
xlim([0 inf])
legend();

subplot(2,2,3);
y3 = zeros(length(args), length(pairs(:, 1)));
for i = 1: length(args)
    y3(i, :) = results(i: length(args): length(results(:, 1)), 7);
end
%bar(x, y3);
errorbar(x, mean(y3,2), std(y3,0,2), 'LineStyle', '-', 'DisplayName', "Flooding");

title("Average Number of hops");
xlabel(xlabel_text);
ylabel("Average Number of Hops");
ylim([0 15]);
yticks(0:1:15);
xticks([0, x])
xlim([0 inf])
legend();
grid on
%     text(-1, 18, "\approx", 'Fontsize', 20);

subplot(2,2,4);
y4 = zeros(length(args), length(pairs(:, 1)));
for i = 1: length(args)
    y4(i, :) = results(i: length(args): length(results(:, 1)), 8);
end
errorbar(x, mean(y4,2), std(y4,0,2), 'LineStyle', '-', 'DisplayName', "Flooding");

grid on
title("Average Number of Transmissions");
xlabel(xlabel_text);
ylabel("Average number of transmissions");
% ylim([0 100]);
% yticks(0:10:100);
xlim([0 inf])
xticks([0 x])
legend();
end
