% ababujo: Creating a scenario as follows: Plume wrappping algorithm
% change task to reflect different cases
% single point of intersection, multiple intersection and almost the whole
% mesh intersect the plume. implemented with a 3x3 mesh

clear all
close all
% include simulator
addpath(['..',filesep,'sim']);
addpath(['..',filesep,'controllers']);

evaluate_performance = 1;
% create simulator object
qrsim = QRSim();

state = qrsim.init('TaskPlume_1');
unicast = 0;
N = state.task.N4 ;
U = zeros(3,N);

tstart = tic;

for i=1:1
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
        
    if evaluate_performance == 1
        
        mark_points = 0;
        number_of_iterations = 30;
        number_of_msgs = 500;
        petal_sizes = 5:10:105;
        min_petal_wid = 2;
        HTLs = 1:1:11;  % HTL array.  Make sure the length of petal_sizes and HTLs is equal.

        boff_type = 3;
        T_ub = 0.002;  % seconds
        number_of_rows = length(petal_sizes) * state.task.number_of_pairs;
        
        results_petal = zeros(number_of_rows * number_of_iterations, 10);
        results_petal_1 = zeros(number_of_rows * number_of_iterations, 10);
        results_flooding = zeros(number_of_rows * number_of_iterations, 10);
        fprintf("Iter ct= %d, msgCt = %d, formation= %s, pair_ct= %d, dist_scale = %d", number_of_iterations, number_of_msgs, state.task.formation_type, state.task.number_of_pairs, state.dist_scale);
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
        
        csvwrite(sprintf("%s_Flood_%d-Pair_%d-Msg_%d-iters_%d-scale_%d-minwid.csv", state.task.formation_type, length(pairs(:, 1)), number_of_msgs, number_of_iterations, state.dist_scale, min_petal_wid), results_flooding);
        csvwrite(sprintf("%s_petal_%d-Pair_%d-Msg_%d-iters_%d-scale_%d-minwid.csv", state.task.formation_type, length(pairs(:, 1)), number_of_msgs, number_of_iterations, state.dist_scale, min_petal_wid), results_petal);
        csvwrite(sprintf("%s_petal_upd_%d-Pair_%d-Msg_%d-iters_%d-scale_%d-minwid.csv", state.task.formation_type, length(pairs(:, 1)), number_of_msgs, number_of_iterations, state.dist_scale, min_petal_wid), results_petal_1);

        break;
    end
    
    for d1=1:N
        for d2= 1:N
            state.platforms{d1}.uav_coord(:,d2) = [0,0,0];
        end
    end
    
end

elapsed = toc(tstart);

fprintf('running %d times real time\n',(state.task.durationInSteps*state.DT)/elapsed);

