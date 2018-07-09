function x = probability()
    fontsize = 40;
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
    FigH = figure('Position', get(0, 'Screensize'));   
    %subplot(2,1,1);
    plot(recv);
    hold on;
    for i=1:N
        recv(i) = helper(T, i, F, mmean, 0);
    end
    text(100, -20, ['T= ', num2str(T) , ' dBm, F= ',  num2str(F),  ' MHz,  $$\sigma$$= ',  num2str(sigma)], 'FontSize', fontsize, 'Interpreter', 'latex');
    plot(recv)
    grid on;
    hold off;
    xlabel("Distance (m)", 'FontSize', fontsize, 'Interpreter', 'latex');
    ylabel("Signal Strength (dBm)", 'FontSize', fontsize, 'Interpreter', 'latex');
    set(gca, 'FontSize', fontsize)
    title("Received signal strength vs Distance", 'FontSize', fontsize, 'Interpreter', 'latex');
    ylim([-110 0]);
    Fr = getframe(FigH);
    imwrite(Fr.cdata, 'signal_strength.png', 'png')
    
    FigH = figure('Position', get(0, 'Screensize'));   
    %subplot(2,1,2)
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
        arrrs(j) = (1000-sum(arrr))/10;
    end
    plot(dsts, arrrs);
    grid on;
    hold on;
    text(150, 20, ['T= ', num2str(T) , ' dBm, F= ',  num2str(F),  ' MHz,  $$\sigma$$= ',  num2str(sigma)], 'FontSize', fontsize, 'Interpreter', 'latex');
    ylabel("Packet loss ($\%$)", 'FontSize', fontsize, 'Interpreter', 'latex');
    xlabel("Distance (m)", 'FontSize', fontsize, 'Interpreter', 'latex');
    set(gca, 'FontSize', fontsize)
    title_1 = sprintf("Packet loss vs Distance");
    title(title_1, 'FontSize', fontsize, 'Interpreter', 'latex');  
    Fr    = getframe(FigH);
    imwrite(Fr.cdata, 'packet_loss.png', 'png')
end

function RecPower = helper(T, D, F, mmean, sigma)
    % T in Dbm
    % D in m
    % F in MHz
    FSPL = 20 * log10(D/1000) + 20 * log10(F) + 32.44; % Free space path loss
    N = normrnd(mmean, sigma);
    RecPower = T - FSPL + N;
    
end

