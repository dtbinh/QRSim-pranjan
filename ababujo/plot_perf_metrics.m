clear all;
formation = "spherical"; % random spherical
pairs_ct = 1;
msgs_ct = 1000;
iterations_ct = 1;
scale = 6;
minwid = 1;
HTLs = 1:1:11;
petal_sizes = 5:10:105;
% Make sure that the above variables match from main_plume
flo = sprintf("%s_Flood_%d-Pair_%d-Msg_%d-iters_%d-scale_%d-minwid.csv", formation, pairs_ct, msgs_ct, iterations_ct, scale, minwid);

res_flooding = csvread(flo);
pe = sprintf("%s_petal_%d-Pair_%d-Msg_%d-iters_%d-scale_%d-minwid.csv",formation, pairs_ct, msgs_ct, iterations_ct, scale, minwid);
res_petal = csvread(pe);
pe1 = sprintf("%s_petal_upd_%d-Pair_%d-Msg_%d-iters_%d-scale_%d-minwid.csv", formation, pairs_ct, msgs_ct, iterations_ct, scale, minwid);
res_petal_1 = csvread(pe1);


plot_graph_flooding(res_flooding, HTLs, pairs_ct, iterations_ct, "Flooding", formation, msgs_ct, scale, minwid);
plot_graph_petals(res_petal, res_petal_1, petal_sizes, pairs_ct, iterations_ct, "3D petal routing", formation, msgs_ct, scale, minwid);

function plot_graph_petals(petal_results, petal_up_results, petal_sizes, pair_ct, iterations_ct, name, formation, msgs_ct, scale, minwid)
x = petal_sizes;
no_of_rows = length(petal_sizes);
no_of_cols = pair_ct * iterations_ct;

avg_dist = mean(petal_results(:, 3)) * scale; 
subtitle = sprintf("\nType= %s, Iterations= %d, Node pairs= %d, Avg Distance = %d m, Packets= %d", formation, iterations_ct, pair_ct, ceil(avg_dist), msgs_ct);

table_size = size(petal_results, 1);
figure('Name', name, 'NumberTitle', 'off');
xlabel_text = "'Petal-width' to 'src-dst distance', '%'";

subplot(2,2,1);

dr1 = zeros(no_of_rows, no_of_cols);
dr2 = zeros(no_of_rows, no_of_cols);
for i = 1: no_of_rows
    dr1(i, :) = petal_results(i: no_of_rows: table_size, 5);
    dr2(i, :) = petal_up_results(i: no_of_rows: table_size, 5);
end

errorbar(x, mean(dr1,2), std(dr1,0,2), 'LineStyle', '--', 'DisplayName', "Don't update petal");
hold on;
errorbar(x, mean(dr2,2), std(dr2,0,2), 'LineStyle', '-.', 'DisplayName', 'Update petal');
title(sprintf("Delivery Rate. %s", subtitle));
xlabel(xlabel_text);
ylabel("Delivery Rate, '%'");
ylim([0 110]);
yticks(0:10:100)
xlim([0 inf])
xticks([0, x])
legend('Location','southeast');
grid on

subplot(2,2,2);
delay = zeros(no_of_rows, no_of_cols);
delay_1 = zeros(no_of_rows, no_of_cols);

for i = 1: no_of_rows
    delay(i, :) = petal_results(i: no_of_rows: table_size, 6);
    delay_1(i, :) = petal_up_results(i: no_of_rows: table_size, 6);
end
hold on;
errorbar(x, mean(delay,2), std(delay,0,2), 'LineStyle', '--', 'DisplayName', "Don't update petal");
errorbar(x, mean(delay_1,2), std(delay_1,0,2),'LineStyle', '-.', 'DisplayName', "Update petal");
grid on
title(sprintf("Delay. %s", subtitle));
xlabel(xlabel_text);
ylabel("Delay 'seconds'");
ylim([0 0.25]);
yticks(0:0.02:0.25);
xticks([0, x])
xlim([0 inf])
legend();

subplot(2,2,3);
hops = zeros(no_of_rows, no_of_cols);
hops_1 = zeros(no_of_rows, no_of_cols);
for i = 1: no_of_rows
    hops(i, :) = petal_results(i: no_of_rows: table_size, 7);
    hops_1(i, :) = petal_up_results(i: no_of_rows: table_size, 7);
end
hold on;
errorbar(x, mean(hops,2), std(hops,0,2), 'LineStyle','--', 'DisplayName', "Don't update petal");
errorbar(x, mean(hops_1,2), std(hops_1,0,2), 'LineStyle', '-.', 'DisplayName', "Update petal");
title(sprintf("Hops. %s", subtitle));
xlabel(xlabel_text);
ylabel("Average Number of Hops");
ylim([0 15]);
yticks(0:1:15);
xticks([0, x])
xlim([0 inf])
legend();
grid on

subplot(2,2,4);
overhead = zeros(no_of_rows, no_of_cols);
overhead_1 = zeros(no_of_rows, no_of_cols);

for i = 1: no_of_rows
    overhead(i, :) = petal_results(i: no_of_rows: table_size, 8);
    overhead_1(i, :) = petal_up_results(i: no_of_rows: table_size, 8);
end
hold on;
errorbar(x, mean(overhead,2), std(overhead,0,2),'LineStyle', '--', 'DisplayName', "Don't update header");
errorbar(x, mean(overhead_1,2), std(overhead_1,0,2), 'LineStyle', '-.','DisplayName', 'Update header');
set(gca,'YScale','log')
grid on
title(sprintf("Overhead. %s", subtitle));
xlabel(xlabel_text);
ylabel("Average number of transmissions");
%ylim([0 100]);
%yticks(0:10:100);
xlim([0 inf])
xticks([0 x])
legend();
end




function plot_graph_flooding(results, HTLs, pair_ct, iterations_ct, name, formation, msgs_ct, scale, minwid)
x = HTLs;
no_of_rows = length(HTLs);
no_of_cols = pair_ct * iterations_ct;

figure('Name', name, 'NumberTitle','off');
xlabel_text = "HTL";
avg_dist = mean(results(:, 3)) * scale; 
subtitle = sprintf("\nType= %s, Iterations= %d, Node pairs= %d, Avg Distance = %d m, Packets= %d", formation, iterations_ct, pair_ct, ceil(avg_dist), msgs_ct);
subplot(2,2,1);
y = zeros(no_of_rows, no_of_cols);
for i = 1: no_of_rows
    y(i, :) = results(i: no_of_rows: size(results, 1), 5);
end
errorbar(x, mean(y,2), std(y,0,2), 'LineStyle', '--', 'DisplayName', "Flooding");
title(sprintf("Delivery Rate. %s", subtitle));
xlabel(xlabel_text);
ylabel("Delivery Rate, '%'");
ylim([0 110]);
yticks(0:10:100)
xlim([0 inf])
xticks([0, x])
legend('Location','southeast');
grid on


subplot(2,2,2);
y1 = zeros(no_of_rows, no_of_cols);
y2 = zeros(no_of_rows, no_of_cols);
for i = 1: length(HTLs)
    y1(i, :) = results(i: no_of_rows: size(results, 1), 9);
    y2(i, :) = results(i: no_of_rows: size(results, 1), 6);
end
%bar(x, y2);
hold on;
errorbar(x, mean(y2,2), std(y2,0,2), 'LineStyle', '-.', 'DisplayName', "End-to-end Delay");
errorbar(x, mean(y1,2), std(y1,0,2), 'LineStyle', '--', 'DisplayName', "Total-flooding-time");
%set(gca,'YScale','log')
grid on
title(sprintf("Delay. %s", subtitle));
xlabel(xlabel_text);
ylabel("Delay 'seconds'");
% ylim([0 0.25]);
% yticks(0:0.02:0.25);
xticks([0, x])
xlim([0 inf])
legend();

subplot(2,2,3);
y3 = zeros(no_of_rows, no_of_cols);
for i = 1: length(HTLs)
    y3(i, :) = results(i: no_of_rows: size(results, 1), 7);
end
%bar(x, y3);
errorbar(x, mean(y3,2), std(y3,0,2), 'LineStyle', '--', 'DisplayName', "Flooding");

title(sprintf("Hops. %s", subtitle));
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
y4 = zeros(no_of_rows, no_of_cols);
for i = 1: length(HTLs)
    y4(i, :) = results(i: no_of_rows: size(results, 1), 8);
end
errorbar(x, mean(y4,2), std(y4,0,2), 'LineStyle', '--', 'DisplayName', "Flooding");
set(gca,'YScale','log')
grid on
title(sprintf("Overhead. %s", subtitle));
xlabel(xlabel_text);
ylabel("Average number of transmissions");
% ylim([0 100]);
% yticks(0:10:100);
xlim([0 inf])
xticks([0 x])
legend();
end
