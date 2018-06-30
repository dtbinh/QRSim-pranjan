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
            avg_tot_delay = total_tot_delay / number_of_msgs;
            success_rate = success_count/number_of_msgs * 100;
            if success_count > 0
                avg_number_hops = total_number_of_hops/success_count;
                avg_end_to_end_delay = total_end_to_end_delay / success_count;
                avg_end_to_end_delay.Format = 'hh:mm:ss.SSSSSSSSS';
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
