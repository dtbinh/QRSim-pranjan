clear all;

global single_figure;
single_figure = 0;

formation = "random"; % random spherical
pairs_ct = 1;
msgs_ct = 250;
iterations_ct = 15;
scale = 6;
minwid = 2;
HTLs = 1:1:11;
petal_sizes = 5:10:105;
% Make sure that the above variables match from main_plume

flo = sprintf("%s_Flood_%d-Pair_%d-Msg_%d-iters_%d-scale_%d-minwid.csv", formation, pairs_ct, msgs_ct, iterations_ct, scale, minwid);
pe = sprintf("%s_petal_%d-Pair_%d-Msg_%d-iters_%d-scale_%d-minwid.csv",formation, pairs_ct, msgs_ct, iterations_ct, scale, minwid);
pe1 = sprintf("%s_petal_upd_%d-Pair_%d-Msg_%d-iters_%d-scale_%d-minwid.csv", formation, pairs_ct, msgs_ct, iterations_ct, scale, minwid);
res_flooding = csvread(flo);
res_petal = csvread(pe);
res_petal_1 = csvread(pe1);


plot_graph_flooding(res_flooding, HTLs, pairs_ct, iterations_ct, formation, msgs_ct, scale, minwid);
plot_graph_petals(res_petal, res_petal_1, petal_sizes, pairs_ct, iterations_ct, formation, msgs_ct, scale, minwid);


function plot_graph_petals(petal_results, petal_up_results, petal_sizes, pair_ct, iterations_ct, formation, msgs_ct, scale, ~)
global single_figure;
x = petal_sizes;
no_of_rows = length(petal_sizes);
no_of_cols = pair_ct * iterations_ct;
fontsize = 14;
leg_font_size = 13;

avg_dist = mean(petal_results(:, 3)) * scale;
subtitle = sprintf("Type= %s, Iterations= %d \n Node pairs= %d, Avg Distance = %d m, Packets= %d", formation, iterations_ct, pair_ct, ceil(avg_dist), msgs_ct);
%subtitle = sprintf("Type= %s, Iterations= %d \n Node pairs= %d, Avg Distance = %d m, Packets= %d", formation, 15, 5, ceil(avg_dist), 250);

fig_name = sprintf("Petal Routing %s Iter-%d, Pairs= %d, Pkts=%d", formation, iterations_ct, pair_ct, msgs_ct);
table_size = size(petal_results, 1);
if single_figure == 1
    FigH = figure('Position', get(0, 'Screensize'),'Name', fig_name, 'NumberTitle', 'off');
end
xlabel_text = "Zone-width $$(\%)$$";

if single_figure == 1
    subplot(2,2,1);
else
    mfig = figure();
end
dr1 = zeros(no_of_rows, no_of_cols);
dr2 = zeros(no_of_rows, no_of_cols);
for i = 1: no_of_rows
    dr1(i, :) = petal_results(i: no_of_rows: table_size, 5);
    dr2(i, :) = petal_up_results(i: no_of_rows: table_size, 5);
end

errorbar(x, mean(dr1,2), std(dr1,0,2)/sqrt(size(dr1, 2)), 'LineStyle', '--', 'DisplayName', "Single Transmission Zone");
hold on;
errorbar(x, mean(dr2,2), std(dr2,0,2)/sqrt(size(dr2, 2)), 'LineStyle', '-.', 'DisplayName', 'Diverged Transmission Zones');
title(sprintf("Delivery ratio. %s", subtitle), 'Interpreter', 'latex', 'FontSize', fontsize, 'FontWeight', 'bold');
xlabel(xlabel_text, 'Interpreter', 'latex', 'FontWeight', 'bold', 'FontSize', fontsize);
ylabel("Delivery ratio $$(\%)$$", 'Interpreter', 'latex', 'FontSize', fontsize);
ylim([0 110]);
yticks(0:10:100);
xlim([0 x(end)+5]);
xticks([0, x])
lgd = legend('Location','southeast');
lgd.FontSize = leg_font_size;
grid on
fname = sprintf("pe_DR_%s.png", formation);
if single_figure == 0
    saveas(mfig, fname);
end

if single_figure == 1
    subplot(2,2,2);
else
    mfig = figure();
end
delay = zeros(no_of_rows, no_of_cols);
delay_1 = zeros(no_of_rows, no_of_cols);

for i = 1: no_of_rows
    delay(i, :) = petal_results(i: no_of_rows: table_size, 6);
    delay_1(i, :) = petal_up_results(i: no_of_rows: table_size, 6);
end
hold on;
errorbar(x, mean(delay,2), std(delay,0,2)/sqrt(size(delay, 2)), 'LineStyle', '--', 'DisplayName', "Single Transmission Zone");
errorbar(x, mean(delay_1,2), std(delay_1,0,2)/sqrt(size(delay_1, 2)),'LineStyle', '-.', 'DisplayName', "Diverged Transmission Zones");
grid on
title(sprintf("Delay. %s", subtitle), 'Interpreter', 'latex', 'FontSize', fontsize);
xlabel(xlabel_text, 'Interpreter', 'latex', 'FontSize', fontsize);
ylabel("Delay (seconds)", 'Interpreter', 'latex', 'FontSize', fontsize);
ylim([0 0.25]);
yticks(0:0.02:0.25);
xticks([0, x]);
xlim([0 x(end)+5]);
leg = legend();
leg.FontSize = leg_font_size;
fname = sprintf("pe_delay_%s.png", formation);
if single_figure == 0
    saveas(mfig, fname);
end

if single_figure == 1
    subplot(2,2,3);
else
    mfig = figure();
end
hops = zeros(no_of_rows, no_of_cols);
hops_1 = zeros(no_of_rows, no_of_cols);
for i = 1: no_of_rows
    hops(i, :) = petal_results(i: no_of_rows: table_size, 7);
    hops_1(i, :) = petal_up_results(i: no_of_rows: table_size, 7);
end
hold on;
errorbar(x, mean(hops,2), std(hops,0,2)/sqrt(size(hops, 2)), 'LineStyle','--', 'DisplayName', "Single Transmission Zone");
errorbar(x, mean(hops_1,2), std(hops_1,0,2)/sqrt(size(hops_1, 2)), 'LineStyle', '-.', 'DisplayName', "Diverged Transmission Zones");
title(sprintf("Hop Count. %s", subtitle), 'Interpreter', 'latex', 'FontSize', fontsize);
xlabel(xlabel_text, 'Interpreter', 'latex', 'FontSize', fontsize);
ylabel("Average Number of Hops", 'Interpreter', 'latex', 'FontSize', fontsize);
ylim([0 15]);
yticks(0:1:15);
xticks([0, x])
xlim([0 x(end)+5]);
leg = legend();
leg.FontSize = leg_font_size;
grid on;
fname= sprintf("pe_hops_%s.png", formation);
if single_figure == 0
    saveas(mfig, fname);
end

if single_figure == 1
    subplot(2,2,4);
else
    mfig = figure();
end
overhead = zeros(no_of_rows, no_of_cols);
overhead_1 = zeros(no_of_rows, no_of_cols);

for i = 1: no_of_rows
    overhead(i, :) = petal_results(i: no_of_rows: table_size, 8);
    overhead_1(i, :) = petal_up_results(i: no_of_rows: table_size, 8);
end
hold on;
errorbar(x, mean(overhead,2), std(overhead,0,2)/sqrt(size(overhead, 2)),'LineStyle', '--', 'DisplayName', "Single Transmission Zone");
errorbar(x, mean(overhead_1,2), std(overhead_1,0,2)/sqrt(size(overhead_1, 2)), 'LineStyle', '-.','DisplayName', 'Diverged Transmission Zones');
%ylim([0 10000]);
set(gca,'YScale','log');
grid on
title(sprintf("Overhead. %s", subtitle), 'Interpreter', 'latex', 'FontSize', fontsize);
xlabel(xlabel_text, 'Interpreter', 'latex', 'FontSize', fontsize);
ylabeltext = sprintf("Average number of transmissions \n per successful packet delivery");
ylabel(ylabeltext, 'Interpreter', 'latex', 'FontSize', fontsize);
%yticks(0:10:100);
xlim([0 x(end)+5]);
xticks([0 x]);
leg = legend();
leg.FontSize = leg_font_size;
fname = sprintf("pe_trans_%s.png", formation);
if single_figure == 0
    saveas(mfig, fname);
end

if single_figure == 1
    fname = sprintf("Petal_%s_Iter-%d_Pairs_%d_Pkts_%d_Scale_%d.png", formation, iterations_ct, pair_ct, msgs_ct, scale);
    saveas(FigH, fname);
end
end







function plot_graph_flooding(results, HTLs, pair_ct, iterations_ct, formation, msgs_ct, scale, ~)
global single_figure;
fontsize = 14;
leg_font_size = 13;
x = HTLs;
no_of_rows = length(HTLs);
no_of_cols = pair_ct * iterations_ct;
fig_name = sprintf("Flooding, %s Iter-%d, Pairs= %d, Pkts=%d", formation, iterations_ct, pair_ct, msgs_ct);
if single_figure == 1
    FigH = figure('Position', get(0, 'Screensize'), 'Name', fig_name, 'NumberTitle','off');
end
xlabel_text = "Hops To Live";
avg_dist = mean(results(:, 3)) * scale;
subtitle = sprintf("Type= %s, Iterations= %d, \nNode pairs= %d, Avg Distance = %d m, Packets= %d", formation, iterations_ct, pair_ct, ceil(avg_dist), msgs_ct);
%subtitle = sprintf("Type= %s, Iterations= %d, \nNode pairs= %d, Avg Distance = %d m, Packets= %d", formation, 15, 5, ceil(avg_dist), 250);

if single_figure == 1
    subplot(2,2,1);
else
    mfig = figure();
end
y = zeros(no_of_rows, no_of_cols);
for i = 1: no_of_rows
    y(i, :) = results(i: no_of_rows: size(results, 1), 5);
end
errorbar(x, mean(y,2), std(y,0,2)/sqrt(size(y, 2)), 'LineStyle', '--', 'DisplayName', "Delivery ratio - Flooding");
title(sprintf("Delivery ratio. %s", subtitle), 'FontSize', fontsize, 'Interpreter', 'latex');
xlabel(xlabel_text, 'FontSize', fontsize, 'Interpreter', 'latex');
ylabel("Delivery ratio $$(\%)$$", 'FontSize', fontsize, 'Interpreter', 'latex');
ylim([0 110]);
yticks(0:10:100);
xlim([0 x(end)+1]);
xticks([0, x]);
leg = legend('Location','southeast');
leg.FontSize = leg_font_size;
grid on
fname = sprintf("fl_DR_%s.png", formation);
if single_figure == 0
    saveas(mfig, fname);
end


if single_figure == 1
    subplot(2,2,2);
else
    mfig = figure();
end
y1 = zeros(no_of_rows, no_of_cols);
y2 = zeros(no_of_rows, no_of_cols);
for i = 1: length(HTLs)
    y1(i, :) = results(i: no_of_rows: size(results, 1), 9);
    y2(i, :) = results(i: no_of_rows: size(results, 1), 6);
end
%bar(x, y2);
hold on;
errorbar(x, mean(y2,2), std(y2,0,2)/sqrt(size(y2, 2)), 'LineStyle', '-.', 'DisplayName', "End-to-end Delay");
errorbar(x, mean(y1,2), std(y1,0,2)/sqrt(size(y1, 2)), 'LineStyle', '--', 'DisplayName', "Total-flooding-time");
%set(gca,'YScale','log')
grid on
title(sprintf("Delay. %s", subtitle), 'Interpreter', 'latex', 'FontSize', fontsize);
xlabel(xlabel_text, 'Interpreter', 'latex', 'FontSize', fontsize);
ylabel("Delay (seconds)", 'Interpreter', 'latex', 'FontSize', fontsize);
ylim([0 0.25]);
yticks(0:0.02:0.25);
xticks([0, x]);
xlim([0 x(end)+1]);
leg = legend();
leg.FontSize = leg_font_size;
fname = sprintf("fl_delay_%s.png", formation);
if single_figure == 0
    saveas(mfig, fname);
end


if single_figure == 1
    subplot(2,2,3);
else
    mfig = figure();
end
y3 = zeros(no_of_rows, no_of_cols);
for i = 1: length(HTLs)
    y3(i, :) = results(i: no_of_rows: size(results, 1), 7);
end
%bar(x, y3);
errorbar(x, mean(y3,2), std(y3,0,2)/sqrt(size(y3, 2)), 'LineStyle', '--', 'DisplayName', "Hop Count - Flooding");

title(sprintf("Hop Count. %s", subtitle), 'Interpreter', 'latex', 'FontSize', fontsize);
xlabel(xlabel_text, 'Interpreter', 'latex', 'FontSize', fontsize);
ylabel("Average Number of Hops", 'Interpreter', 'latex', 'FontSize', fontsize);
ylim([0 15]);
yticks(0:1:15);
xticks([0, x])
xlim([0 x(end)+1]);
leg = legend();
leg.FontSize = leg_font_size;
grid on
fname = sprintf("fl_hops_%s.png", formation);
if single_figure == 0
    saveas(mfig, fname);
end

if single_figure == 1
    subplot(2,2,4);
else
    mfig = figure();
end
y4 = zeros(no_of_rows, no_of_cols);
for i = 1: length(HTLs)
    y4(i, :) = results(i: no_of_rows: size(results, 1), 8);
end
errorbar(x, mean(y4,2), std(y4,0,2)/sqrt(size(y4, 2)), 'LineStyle', '--', 'DisplayName', "Avg Transmissions - Flooding");
%ylim([10 1100]);
set(gca,'YScale','log')
grid on
title(sprintf("Overhead. %s", subtitle), 'Interpreter', 'latex', 'FontSize', fontsize);
xlabel(xlabel_text, 'Interpreter', 'latex', 'FontSize', fontsize);
ylabeltext = sprintf("Average number of transmissions \n per successful packet delivery");
ylabel(ylabeltext, 'FontSize', fontsize);
% yticks(0:10:100);
xlim([0 x(end)+1]);
xticks([0 x])
leg = legend();
leg.FontSize = leg_font_size;
fname = sprintf("fl_trans_%s.png", formation);
if single_figure == 0
    saveas(mfig, fname);
end



if single_figure == 1
    fnam = sprintf("Flooding%s_Iter%d_Pairs%d_Pkts%d_scale_%d.png", formation, iterations_ct, pair_ct, msgs_ct, scale);
    saveas(FigH, fnam);
end
end