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

% a = datetime('now');
% T_ub = 0.002;  % Upper bound of Backoff Time
% T_lb = 0;           % Lower bound of Backoff Time
% boff_time =  T_lb + a + rand() * T_ub;
% isbetween(datetime('now'), a, boff_time)

unicast = 0;
evaluate_petal_performance = 1;

set_invisible = 0;
petal_width = 10;
src_drone = 1;
dest_drone = 36;

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
        ct = qrsim.app_unicast(src_drone, dest_drone, petal_width, "no_data");
        if set_invisible == 1
            dataH = get(gca, 'Children');
            for plot_points = 1:(ct+2)
                set(dataH(plot_points), 'visible', 'off');
            end
        end
    end
    
    if evaluate_petal_performance == 1
        petal_sizes = 30:10:60;
        pairs = state.task.furthest_pairs;
        results = zeros(length(petal_sizes) * length(pairs(:, 1)), 8);
        %delay_results = zeros(length(petal_sizes) * length(pairs(:, 1)), 1);
        number_of_msgs = 10;
        res_idx = 1;
        
        for idx = 1:length(pairs(:, 1))
            src = pairs(idx, 1);
            dest = pairs(idx, 2);
            if src == dest; continue;   end
            for petal_width = petal_sizes
                success_count = 0;
                total_number_of_hops = 0;
                total_transmissions = 0;
                total_end_to_end_delay = 0;
                avg_end_to_end_delay = seconds(50);
                avg_number_hops = 50;
                overhead = inf;
                
                for run_no = 1: number_of_msgs
                    msg = geo_message(state, src, dest, petal_width, "no_data");
                    [scat_ct, tr_ct, success, hop_count, end_to_end_delay] = qrsim.petal_send_message(msg, src);
                    total_transmissions = total_transmissions + tr_ct;
                    success_count = success_count + success;
                    if success == 1
                        total_number_of_hops = total_number_of_hops + hop_count;
                        total_end_to_end_delay = total_end_to_end_delay + end_to_end_delay;
                    end
                    if set_invisible == 1
                        dataH = get(gca, 'Children');
                        for plot_points = 1:(scat_ct+2)
                            set(dataH(plot_points), 'visible', 'off');
                        end
                    end
                end
                minor_axis_to_dist_percentage = msg.minor_axis/msg.src_dst_dist * 100;
                success_rate = success_count/number_of_msgs * 100;
                if success_count > 0
                    avg_number_hops = total_number_of_hops/success_count;
                    avg_end_to_end_delay = total_end_to_end_delay / success_count;
                    avg_end_to_end_delay.Format = 'hh:mm:ss.SSSSSSSSS';
                    overhead = total_transmissions / success_count; % Number of transmissions per successfully delivered packet.
                end
                fprintf("\nS= %d, D= %d, Dist= %f, Petal width ratio= %f, Transmitted pkt ct= %d, Success Rate = %f",...
                         src, dest, msg.src_dst_dist, minor_axis_to_dist_percentage, number_of_msgs, success_rate);
                fprintf(" Avg end-to-end delay = %f, Avg number of hops= %f, Overhead= %f", seconds(avg_end_to_end_delay), avg_number_hops, overhead);
                results(res_idx, : ) = [src, dest, msg.src_dst_dist, round(minor_axis_to_dist_percentage), success_rate, avg_number_hops, overhead, seconds(avg_end_to_end_delay)];
                %delay_results(res_idx, :) = seconds(avg_end_to_end_delay);
                res_idx = res_idx + 1;
            end
        end
    end
    
    
    for d1=1:N
        for d2= 1:N
            state.platforms{d1}.uav_coord(:,d2) = [0,0,0];
        end
    end
    
end


elapsed = toc(tstart);figure();


fprintf('running %d times real time\n',(state.task.durationInSteps*state.DT)/elapsed);