% pranjan: Study the effects of network density on Delivery rate, hops,
% transmissions, and end-to-end delay.
clear all
close all
% include simulator
addpath(['..',filesep,'sim']);
addpath(['..',filesep,'controllers']);

% These variables should not be changed.
mark_points = 0;
petal_sizes = 35:10:36;  % Petal size of 35%
HTLs = 10:1:10;  % HTL of 10.
boff_type = 3;
T_ub = 0.002;


number_iterations = 30;
drone_count_arr = [16, 27, 36, 45, 64, 91, 125, 175, 250];
number_of_msgs = 500;
min_petal_wid = 2;
number_of_pairs = 1; % MAKE SURE this value is the same as in TaskPlume_1.m
fprintf("Number of iterations= %d, msgCt = %d", number_iterations, number_of_msgs);

results_pe = zeros(length(drone_count_arr) * number_iterations * number_of_pairs, 10);
results_pe_up = zeros(length(drone_count_arr) * number_iterations * number_of_pairs, 10);
results_flooding = zeros(length(drone_count_arr) * number_iterations * number_of_pairs, 10);
for i =  1: length(drone_count_arr)
    droneCount = drone_count_arr(i);
    qrsim = QRSim(droneCount); % create simulator object % 25 is the number of drones.
    state = qrsim.init('TaskPlume_1');
    N = state.task.N4 ;
    for j=1:number_iterations
        idx_start = (i - 1) * number_iterations * number_of_pairs + (j - 1) * number_of_pairs;
        idx_end = idx_start + number_of_pairs;
        
        U = zeros(3,N);
%         if state.send_coordinates == 1
%             % Send UAVs coordinates. All the coordinates shall be available
%             % before the next time quantum starts. We have kept it this way
%             % because, in our benchmarks the message processing +
%             % transmission delay was much smaller than the timestep of 0.02
%             % seconds.
%             qrsim.broadcast_coordinates();
%         end
        qrsim.step(U);
        pairs = state.task.furthest_pairs;

        type = "petal";
        update_petal = 0;
        fprintf("Iter No= %d, Formation= %s, Drone Count = %d, Pair Ct= %d, Scale= %d", j, state.task.formation_type, N, state.task.number_of_pairs, state.dist_scale);
        results_pe(idx_start+1:idx_end, :) = performace_evaluation(qrsim, state, type, pairs, petal_sizes, number_of_msgs, mark_points, update_petal, boff_type, T_ub, min_petal_wid);
        results_pe(idx_start+1:idx_end, 4) = state.task.N4;


        update_petal = 1;
        type = "petal";
        results_pe_up(idx_start+1:idx_end, :) = performace_evaluation(qrsim, state, type, pairs, petal_sizes, number_of_msgs, mark_points, update_petal, boff_type, T_ub, min_petal_wid);
        results_pe_up(idx_start+1:idx_end,4) = state.task.N4;


        type = "flooding";
        results_flooding(idx_start+1:idx_end, :) = performace_evaluation(qrsim, state, type, pairs, HTLs, number_of_msgs, mark_points);
        results_flooding(idx_start+1:idx_end, 4) = state.task.N4;    
        
        state.task.reset();
    end
    
    for d1=1:N
        for d2= 1:N
            state.platforms{d1}.uav_coord(:,d2) = [0,0,0];
        end
    end
    
end

f1 = sprintf("D_petal_%d-iterCt_%d-drones_%d-pairs_%d-msgs_%d-scale_%d-minwid.csv", number_iterations,length(drone_count_arr), size(pairs, 1), number_of_msgs, state.dist_scale, min_petal_wid);
csvwrite(f1, results_pe);

f2 = sprintf("D_petal_UPD_%d-iterCt_%d-drones_%d-pairs_%d-msgs_%d-scale_%d-minwid.csv", number_iterations,length(drone_count_arr), size(pairs, 1), number_of_msgs, state.dist_scale, min_petal_wid);
csvwrite(f2, results_pe_up);

f3 = sprintf("D_Flooding_%d-iterCt_%d-drones_%d-pairs_%d-msgs_%d-scale.csv",number_iterations, length(drone_count_arr), size(pairs, 1), number_of_msgs, state.dist_scale);
csvwrite(f3, results_flooding);
