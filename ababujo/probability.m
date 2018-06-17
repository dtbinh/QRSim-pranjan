function x = probability()
    T = 17;
    Dth = -87;        % dBm, Threshold for a successful message receipt.
    N = 500;
    F = 2400;  % MHz
    mmean = -20;
    sigma = 2;
    recv = zeros(1,N);
    for i=1:N
        recv(i) = helper(T, i, F, mmean, sigma);
    end
    figure();
    subplot(2,1,1)
    plot(recv);
    hold on;
    for i=1:N
        recv(i) = helper(T, i, F, mmean, 0);
    end
    text(250, -40, ['T= ', num2str(T) , ' dBm, F= ',  num2str(F),  ' MHz,  sigma= ',  num2str(sigma)], 'FontSize', 20);
    plot(recv)
    grid on;
    hold off;
    xlabel("Distance (m)", 'FontSize', 20);
    ylabel("dBm", 'FontSize', 20);
    title("Received signal strength vs Distance", 'FontSize', 20);
    ylim([-110 0]);
    %title([' T= ', num2str(T), ' dBm, F= ', num2str(F), ' m, mean= ', num2str(mmean), ', sigma= ', num2str(sigma)]);
    
    subplot(2,1,2)
    %dsts = [10, sqrt(2) * 10, 20, sqrt(2) * 20, 30, 30 * sqrt(2)];
    dsts = 1:N;
    arrrs = zeros(length(dsts));
    for j = 1: length(dsts)
        arrr = zeros(1,1000);
        for i= 1:1000
            p = helper(T, dsts(j), F, mmean, sigma);
            if p > Dth
                arrr(i) = 1;
            else
                arrr(i) = 0;
            end
        end
        arrrs(j) = 1000-sum(arrr);
    end
    plot(dsts, arrrs);
    grid on;
    hold on;
    text(300, 600, ['T= ', num2str(T) , ' dBm, F= ',  num2str(F),  ' MHz,  sigma= ',  num2str(sigma)], 'FontSize', 20);
    ylabel("Lost packets count", 'FontSize', 20);
    xlabel("Distance (m)", 'FontSize', 20);
    title("Packet loss count per 1000 packets", 'FontSize', 20);
end

function RecPower = helper(T, D, F, mmean, sigma)
    % T in Dbm
    % D in m
    % F in MHz
    FSPL = 20 * log10(D/1000) + 20 * log10(F) + 32.44; % Free space path loss
    N = normrnd(mmean, sigma);
    RecPower = T - FSPL + N;
    
end

