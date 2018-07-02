clear all;
global single_figure;
single_figure = 0;
petal_size = 35;
HTL = 10;

number_iterations = 30;
drone_count_arr= [16, 27, 36, 45, 64, 91, 125, 175, 250];
%drone_count_arr= [16, 25, 36];
pair_ct = 1;
number_of_msgs = 300;
scale= 6;
min_petal_wid = 2;

f1 = sprintf("D_petal_%d-iterCt_%d-drones_%d-pairs_%d-msgs_%d-scale_%d-minwid.csv", number_iterations, length(drone_count_arr), pair_ct, number_of_msgs, scale, min_petal_wid);
results_pe = csvread(f1);

f2 = sprintf("D_petal_UPD_%d-iterCt_%d-drones_%d-pairs_%d-msgs_%d-scale_%d-minwid.csv", number_iterations, length(drone_count_arr), pair_ct, number_of_msgs, scale, min_petal_wid);
results_pe_up = csvread(f2);

f3 = sprintf("D_Flooding_%d-iterCt_%d-drones_%d-pairs_%d-msgs_%d-scale.csv",number_iterations, length(drone_count_arr), pair_ct, number_of_msgs, scale);
results_flooding = csvread(f3);


plot_network_density(results_pe, results_pe_up, results_flooding, drone_count_arr, pair_ct, number_iterations, number_of_msgs);

function plot_network_density(results_pe, results_pe_up, results_flooding, drone_ct_arr, pair_ct, iter_ct, number_of_msgs)
global single_figure;
fontsize = 14;
leg_font_size = 13;
x = drone_ct_arr;
xlabel_text = "Number of UAVs. Critical = 43";
if single_figure == 1
    FigH = figure('Position', get(0, 'Screensize'), 'Name', "Network Density", 'NumberTitle','off');
end

jump_ct = iter_ct * pair_ct;
dr_pe = zeros(length(x), jump_ct);
dr_pe_up = zeros(length(x), jump_ct);
dr_fl = zeros(length(x), jump_ct);

for i = 1: jump_ct: size(results_pe, 1)
    idx = ceil(i / jump_ct);
    dr_pe(idx, :) = results_pe(i:1:i+jump_ct-1, 5);
    dr_pe_up(idx, :) = results_pe_up(i:1:i+jump_ct-1, 5);
    dr_fl(idx, :) = results_flooding(i:1:i+jump_ct-1, 5);
end
subtitle = sprintf("Petal width = 35, Flooding HTL = 10 \n Iterations= %d, Packets= %d, Node pairs= %d", iter_ct, number_of_msgs, pair_ct);

if single_figure == 1
    subplot(2,2,1);
else
    mfig = figure();
end
hold on;
errorbar(x, mean(dr_pe,2), std(dr_pe, 0, 2)/sqrt(size(dr_pe, 2)), 'LineStyle', "-.", 'DisplayName', 'Single Transmission Zone');
errorbar(x, mean(dr_pe_up,2), std(dr_pe_up, 0, 2)/sqrt(size(dr_pe_up, 2)), 'LineStyle', '--', 'DisplayName', 'Diverged Transmission Zone');
errorbar(x, mean(dr_fl,2), std(dr_fl,0,2)/sqrt(size(dr_fl,2)), 'LineStyle', ':', 'DisplayName', 'Flooding', 'LineWidth', 2);
title(sprintf("Delivery Rate. %s", subtitle), 'FontSize', fontsize);
xlabel(xlabel_text, 'FontSize', fontsize, 'Interpreter', 'latex');
ylabel("Delivery Rate $$(\%)$$", 'FontSize', fontsize, 'Interpreter', 'latex');
ylim([0 110]);
yticks(0:10:100)
xticks(x)
xlim([10 x(end)+5])
leg = legend('Location','southeast');
leg.FontSize = leg_font_size;
grid on;
if single_figure == 0
    saveas(mfig, "ND_DR.png");
end

if single_figure == 1
    subplot(2,2,2);
else
    mfig = figure();
end
delay = zeros(length(x), jump_ct);
delay_up = zeros(length(x), jump_ct);
delay_fl = zeros(length(x), jump_ct);
tot_delay_fl = zeros(length(x), iter_ct * pair_ct);

for i = 1: jump_ct: size(results_pe, 1)
    idx = ceil(i / jump_ct);
    delay(idx, :) = results_pe(i:1:i+jump_ct-1, 6);
    delay_up(idx, :) = results_pe_up(i:1:i+jump_ct-1, 6);
    delay_fl(idx, :) = results_flooding(i:1:i+jump_ct-1, 6);
    tot_delay_fl(idx, :) = results_flooding(i:1:i+jump_ct-1, 9);
end
hold on;
errorbar(x, mean(delay,2), std(delay, 0, 2)/sqrt(size(delay, 2)), 'LineStyle', "-.", 'DisplayName', 'Single Transmission Zone');
errorbar(x, mean(delay_up,2), std(delay_up, 0, 2)/sqrt(size(delay_up, 2)), 'LineStyle', '--', 'DisplayName', 'Diverged Transmission Zone');
errorbar(x, mean(delay_fl,2), std(delay_fl,0,2)/sqrt(size(delay_fl,2)), 'LineStyle', ':', 'DisplayName', 'Flooding', 'LineWidth', 2);
errorbar(x, mean(tot_delay_fl,2), std(tot_delay_fl,0,2)/sqrt(size(tot_delay_fl,2)), 'LineStyle', '-', 'DisplayName', 'Total delay Flooding');

grid on
title(sprintf("Delay. %s", subtitle), 'FontSize', fontsize);  %Petal width = $$35\%$$, Flooding HTL = 10",  'Interpreter', 'latex');
xlabel(xlabel_text, 'FontSize', fontsize, 'Interpreter', 'latex');
ylabel("Delay (seconds)", 'FontSize', fontsize, 'Interpreter', 'latex');
%ylim([0 0.6]);
%yticks(0:0.05:0.6);
xticks(x)
xlim([10 x(end)+5])
leg = legend('Location','northwest');
leg.FontSize = leg_font_size;
if single_figure == 0
    saveas(mfig, "ND_delay.png");
end

if single_figure == 1
    subplot(2,2,3);
else
    mfig = figure();
end
hops = zeros(length(x), jump_ct);
hops_up = zeros(length(x), jump_ct);
hops_fl = zeros(length(x), jump_ct);
for i = 1: jump_ct: size(results_pe, 1)
    idx = ceil(i / jump_ct);
    hops(idx, :) = results_pe(i:1:i+jump_ct-1, 7);
    hops_up(idx, :) = results_pe_up(i:1:i+jump_ct-1, 7);
    hops_fl(idx, :) = results_flooding(i:1:i+jump_ct-1, 7);
end

hold on;
errorbar(x, mean(hops,2), std(hops, 0, 2)/sqrt(size(hops, 2)), 'LineStyle', "-.", 'DisplayName', 'Single Transmission Zone');
errorbar(x, mean(hops_up,2), std(hops_up, 0, 2)/sqrt(size(hops_up, 2)), 'LineStyle', '--', 'DisplayName', 'Diverged Transmission Zone');
errorbar(x, mean(hops_fl,2), std(hops_fl,0,2)/sqrt(size(hops_fl,2)), 'LineStyle', ':', 'DisplayName', 'Flooding', 'LineWidth', 2);

title(sprintf("Average Number of hops. %s", subtitle), 'FontSize', fontsize); %, 'Interpreter', 'latex');
xlabel(xlabel_text, 'FontSize', fontsize, 'Interpreter', 'latex');
ylabel("Average Number of Hops", 'FontSize', fontsize, 'Interpreter', 'latex');
ylim([0 15]);
yticks(0:1:15);
xticks(x)
xlim([10 x(end)+5])
leg = legend();
leg.FontSize = leg_font_size;
grid on
if single_figure == 0
    saveas(mfig, "ND_hops.png");
end

if single_figure == 1
    subplot(2,2,4);
else
    mfig = figure();
end

overhead = zeros(length(x), jump_ct);
overhead_up = zeros(length(x), jump_ct);
overhead_fl = zeros(length(x), jump_ct);

for i = 1: jump_ct: size(results_pe, 1)
    idx = ceil(i / jump_ct);
    overhead(idx, :) = results_pe(i:1:i+jump_ct-1, 8);
    overhead_up(idx, :) = results_pe_up(i:1:i+jump_ct-1, 8);
    overhead_fl(idx, :) = results_flooding(i:1:i+jump_ct-1, 8);
end

hold on;
errorbar(x, mean(overhead,2), std(overhead, 0, 2)/sqrt(size(overhead, 2)), 'LineStyle', "-.", 'DisplayName', 'Single Transmission Zone');
errorbar(x, mean(overhead_up,2), std(overhead_up, 0, 2)/sqrt(size(overhead_up, 2)), 'LineStyle', '--', 'DisplayName', 'Diverged Transmission Zone');
errorbar(x, mean(overhead_fl,2), std(overhead_fl,0,2)/sqrt(size(overhead_fl,2)), 'LineStyle', ':', 'DisplayName', 'Flooding', 'LineWidth', 2);

grid on
title(sprintf("Transmissions Count. %s", subtitle), 'FontSize', fontsize);
xlabel(xlabel_text, 'FontSize', fontsize, 'Interpreter', 'latex');
ylabel("Average number of tries per successful delivery", 'FontSize', fontsize);
%ylim([0 100]);
%yticks(0:10:100);
xticks(x)
xlim([10 x(end)+5])
leg = legend('Location','northwest');
leg.FontSize = leg_font_size;
if single_figure == 0
    saveas(mfig, "ND_trans.png");
end

if single_figure == 1
    Fr = getframe(FigH);
    imwrite(Fr.cdata, 'node_density.png', 'png')
end
end