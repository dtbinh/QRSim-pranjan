result_pe = [csvread("petal_35w_25_drones_200_pairs.csv"); csvread("petal_35w_36_drones_200_pairs.csv"); csvread("petal_35w_49_drones_200_pairs.csv"); csvread("petal_35w_64_drones_200_pairs.csv"); csvread("petal_35w_81_drones_200_pairs.csv")];
result_flood = [csvread("flooding_10HTL_25_200_pairs.csv"); csvread("flooding_10HTL_36_200_pairs.csv"); csvread("flooding_10HTL_49_200_pairs.csv"); csvread("flooding_10HTL_64_200_pairs.csv"); csvread("flooding_10HTL_81_200_pairs.csv")];
args = [25, 36, 49, 64, 81];
pe_16 = csvread("petal_35w_16_drones_120_pairs.csv");
fl_16 = csvread("flooding_10HTL_16_120_pairs.csv");
ct = 120;
plot_network_density(result_pe, result_flood, args, length(pairs), pe_16, fl_16, ct);

function plot_network_density(pe_result, fl_result, args, cou, pe_16, fl_16, ct)
    x = [16, args];
    xlabel_text = "Number of UAVs. Ideal = 43";
    figure('Name', "Network Density", 'NumberTitle','off');
    subplot(2,2,1);
    dr1 = zeros(length(args), cou);
    dr2 = zeros(length(args), cou);
    
    for i = 1: cou: length(pe_result)
        idx = ceil(i / cou);
        dr1(idx, :) = pe_result(i:1:i+cou-1, 5);
        dr2(idx, :) = fl_result(i:1:i+cou-1, 5);
    end
    errorbar(x, [mean(pe_16(:, 5)); mean(dr1,2)], [std(pe_16(:, 5), 0); std(dr1,0,2)], 'LineStyle', '--', 'DisplayName', "Petal routing");
    hold on;
    errorbar(x, [mean(fl_16(:, 5)); mean(dr2,2)], [std(fl_16(:, 5), 0); std(dr2,0,2)], 'LineStyle', '-', 'DisplayName', 'Flooding');
    title("Delivery Rate. Petal width = 35%, Flooding HTL = 10");
    xlabel(xlabel_text);
    ylabel("Delivery Rate, '%'");
    ylim([0 110]);
    yticks(0:10:100)
    xlim([0 90])
    xticks([0, x])
    legend('Location','southeast');
    grid on

    subplot(2,2,2);
    delay = zeros(length(args), cou);
    delay_1 = zeros(length(args), cou);

    for i = 1: cou: length(pe_result)
        idx = ceil(i / cou);
        delay(idx, :) = pe_result(i:1:i+cou-1, 6);
        delay_1(idx, :) = fl_result(i:1:i+cou-1, 6);
    end
    hold on;
    errorbar(x, [mean(pe_16(:, 6)); mean(delay,2)], [std(pe_16(:, 6), 0);std(delay,0,2)], 'LineStyle', '--', 'DisplayName', "Petal routing");
    errorbar(x, [mean(fl_16(:, 6)); mean(delay_1,2)], [std(fl_16(:, 5), 0); std(delay_1,0,2)],'LineStyle', '-', 'DisplayName', "Flooding");
    grid on
    title("Average end to end delay. Petal width = 35%, Flooding HTL = 10");
    xlabel(xlabel_text);
    ylabel("Delay 'seconds'");
    ylim([0 0.9]);
    yticks(0:0.05:0.9);
    xticks([0, x])
    xlim([0 90])
    legend();

    subplot(2,2,3);
    hops = zeros(length(args), cou);
    hops_1 = zeros(length(args), cou);
    for i = 1: cou: length(pe_result)
        idx = ceil(i / cou);
        hops(idx, :) = pe_result(i:1:i+cou-1, 7);
        hops_1(idx, :) = fl_result(i:1:i+cou-1, 7);
    end
    hold on;
    errorbar(x, [mean(pe_16(:, 7)); mean(hops,2)], [std(pe_16(:, 6), 0); std(hops,0,2)], 'LineStyle','--', 'DisplayName', "Petal Routing");
    errorbar(x, [mean(fl_16(:, 7)); mean(hops_1,2)], [std(fl_16(:, 6), 0);std(hops_1,0,2)], 'LineStyle', '-', 'DisplayName', "Flooding");
    title("Average Number of hops. Petal width = 35%, Flooding HTL = 10");
    xlabel(xlabel_text);
    ylabel("Average Number of Hops");
    ylim([0 15]);
    yticks(0:1:15);
    xticks([0, x])
    xlim([0 90])
    legend();
    grid on
    
    subplot(2,2,4);
    overhead = zeros(length(args), cou);
    overhead_1 = zeros(length(args), cou);
    for i = 1: cou: length(pe_result)
        idx = ceil(i / cou);
        overhead(idx, :) = pe_result(i:1:i+cou-1, 8);
        overhead_1(idx, :) = fl_result(i:1:i+cou-1, 8);
    end

    hold on;
    errorbar(x, [mean(pe_16(:, 8)); mean(overhead,2)], [std(pe_16(:, 8), 0); std(overhead,0,2)],'LineStyle', '--', 'DisplayName', "Petal Routing");
    errorbar(x, [mean(fl_16(:, 8)); mean(overhead_1,2)], [std(fl_16(:, 8), 0); std(overhead_1,0,2)], 'LineStyle', '-','DisplayName', 'Flooding');
    grid on
    title("Average Number of Transmissions. Petal width = 35%, Flooding HTL = 10");
    xlabel(xlabel_text);
    ylabel("Average number of transmissions");
    ylim([0 100]);
    yticks(0:10:100);
    xlim([0 90])
    xticks([0 x])
    legend();

end