petal_size = 35;
HTL = 10;

drone_count_arr= [25, 36, 49, 64, 81, 100];
%drone_count_arr= [10, 15, 20];

pair_ct = 2;
number_of_msgs = 10;
scale= 6;
min_petal_wid = 2;

result_petal = zeros(pair_ct * length(drone_count_arr), 10);
result_petal_upd = zeros(pair_ct * length(drone_count_arr), 10);
result_petal_flood = zeros(pair_ct * length(drone_count_arr), 10);
idx = 0;
for drone_count = drone_count_arr
    r_st = idx * pair_ct;
    idx = idx + 1;
    f1 = sprintf("D_petal_%d-wid_%d-drones_%d-pairs_%d-msgs_%d-scale_%d-minwid.csv", petal_size, drone_count, pair_ct, number_of_msgs, scale, min_petal_wid);
    result_petal(r_st+1: r_st + pair_ct , :) = csvread(f1);

    f2 = sprintf("D_petal_UPD_%d-wid_%d-drones_%d-pairs_%d-msgs_%d-scale_%d-minwid.csv", petal_size, drone_count, pair_ct, number_of_msgs, scale, min_petal_wid);
    result_petal_upd(r_st+1: r_st+pair_ct, :) = csvread(f2);
    
    f3 = sprintf("D_Flooding_%d-HTL_%d-drones_%d-pairs_%d-msgs_%d-scale.csv",HTL, drone_count, pair_ct, number_of_msgs, scale);
    results_flooding(r_st+1: r_st + pair_ct, :) = csvread(f3);
end

plot_network_density(result_petal, result_petal_upd, results_flooding, drone_count_arr, pair_ct);

function plot_network_density(pe_result, pe_up_result, fl_result, drone_ct_arr, pair_ct)
    x = drone_ct_arr;
    xlabel_text = "Number of UAVs. Critical = 43";
    figure('Name', "Network Density", 'NumberTitle','off');
    subplot(2,2,1);
    dr_pe = zeros(length(x), pair_ct);
    dr_pe_up = zeros(length(x), pair_ct);
    dr_fl = zeros(length(x), pair_ct);
    
    for i = 1: pair_ct: size(pe_result, 1)
        idx = ceil(i / pair_ct);
        dr_pe(idx, :) = pe_result(i:1:i+pair_ct-1, 5);
        dr_pe_up(idx, :) = pe_up_result(i:1:i+pair_ct-1, 5);
        dr_fl(idx, :) = fl_result(i:1:i+pair_ct-1, 5);
    end
    hold on;
    errorbar(x, mean(dr_pe,2), std(dr_pe, 0, 2)/sqrt(size(dr_pe, 2)), 'LineStyle', "-.", 'DisplayName', 'Single Transmission Zone');
    errorbar(x, mean(dr_pe_up,2), std(dr_pe_up, 0, 2)/sqrt(size(dr_pe_up, 2)), 'LineStyle', '--', 'DisplayName', 'Diverged Transmission Zone');
    errorbar(x, mean(dr_fl,2), std(dr_fl,0,2)/sqrt(size(dr_fl,2)), 'LineStyle', ':', 'DisplayName', 'Flooding');
    title("Delivery Rate. Petal width = 35%, Flooding HTL = 10");
    xlabel(xlabel_text);
    ylabel("Delivery Rate, '%'");
    ylim([0 110]);
    yticks(0:10:100)
    xlim([0 x(end)+10])
    legend('Location','southeast');
    grid on

    subplot(2,2,2);
    delay = zeros(length(x), pair_ct);
    delay_up = zeros(length(x), pair_ct);
    delay_fl = zeros(length(x), pair_ct);
    tot_delay_fl = zeros(length(x), pair_ct);
    
    for i = 1: pair_ct: size(pe_result, 1)
        idx = ceil(i / pair_ct);
        delay(idx, :) = pe_result(i:1:i+pair_ct-1, 6);
        delay_up(idx, :) = pe_up_result(i:1:i+pair_ct-1, 6);
        delay_fl(idx, :) = fl_result(i:1:i+pair_ct-1, 6);
        tot_delay_fl(idx, :) = fl_result(i:1:i+pair_ct-1, 9);
    end
    hold on;
    errorbar(x, mean(delay,2), std(delay, 0, 2)/sqrt(size(delay, 2)), 'LineStyle', "-.", 'DisplayName', 'Single Transmission Zone');
    errorbar(x, mean(delay_up,2), std(delay_up, 0, 2)/sqrt(size(delay_up, 2)), 'LineStyle', '--', 'DisplayName', 'Diverged Transmission Zone');
    errorbar(x, mean(delay_fl,2), std(delay_fl,0,2)/sqrt(size(delay_fl,2)), 'LineStyle', ':', 'DisplayName', 'Flooding');
    errorbar(x, mean(tot_delay_fl,2), std(tot_delay_fl,0,2)/sqrt(size(tot_delay_fl,2)), 'LineStyle', '-', 'DisplayName', 'Total delay Flooding');

    grid on
    title("Average end to end delay. Petal width = 35%, Flooding HTL = 10");
    xlabel(xlabel_text);
    ylabel("Delay 'seconds'");
    ylim([0 0.9]);
    yticks(0:0.05:0.9);
    xlim([0 x(end)+10])
    legend('Location','northwest');

    subplot(2,2,3);
    hops = zeros(length(x), pair_ct);
    hops_up = zeros(length(x), pair_ct);
    hops_fl = zeros(length(x), pair_ct);
    for i = 1: pair_ct: size(pe_result, 1)
        idx = ceil(i / pair_ct);
        hops(idx, :) = pe_result(i:1:i+pair_ct-1, 7);
        hops_up(idx, :) = pe_up_result(i:1:i+pair_ct-1, 7);
        hops_fl(idx, :) = fl_result(i:1:i+pair_ct-1, 7);
    end

    hold on;
    errorbar(x, mean(hops,2), std(hops, 0, 2)/sqrt(size(hops, 2)), 'LineStyle', "-.", 'DisplayName', 'Single Transmission Zone');
    errorbar(x, mean(hops_up,2), std(hops_up, 0, 2)/sqrt(size(hops_up, 2)), 'LineStyle', '--', 'DisplayName', 'Diverged Transmission Zone');
    errorbar(x, mean(hops_fl,2), std(hops_fl,0,2)/sqrt(size(hops_fl,2)), 'LineStyle', ':', 'DisplayName', 'Flooding');

    title("Average Number of hops. Petal width = 35%, Flooding HTL = 10");
    xlabel(xlabel_text);
    ylabel("Average Number of Hops");
    ylim([0 15]);
    yticks(0:1:15);
    xticks([0, x])
    xlim([0 x(end)+10])
    legend();
    grid on
    
    subplot(2,2,4);
    overhead = zeros(length(x), pair_ct);
    overhead_up = zeros(length(x), pair_ct);
    overhead_fl = zeros(length(x), pair_ct);

    for i = 1: pair_ct: size(pe_result, 1)
        idx = ceil(i / pair_ct);
        overhead(idx, :) = pe_result(i:1:i+pair_ct-1, 8);
        overhead_up(idx, :) = pe_up_result(i:1:i+pair_ct-1, 8);
        overhead_fl(idx, :) = fl_result(i:1:i+pair_ct-1, 8);
    end

    hold on;
    errorbar(x, mean(overhead,2), std(overhead, 0, 2)/sqrt(size(overhead, 2)), 'LineStyle', "-.", 'DisplayName', 'Single Transmission Zone');
    errorbar(x, mean(overhead_up,2), std(overhead_up, 0, 2)/sqrt(size(overhead_up, 2)), 'LineStyle', '--', 'DisplayName', 'Diverged Transmission Zone');
    errorbar(x, mean(overhead_fl,2), std(overhead_fl,0,2)/sqrt(size(overhead_fl,2)), 'LineStyle', ':', 'DisplayName', 'Flooding');

    grid on
    title("Average Number of Transmissions. Petal width = 35%, Flooding HTL = 10");
    xlabel(xlabel_text);
    ylabel("Average number of transmissions");
    ylim([0 100]);
    yticks(0:10:100);
    xticks([0, x])
    xlim([0 x(end)+10])
    legend('Location','northwest');


end