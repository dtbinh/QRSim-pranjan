% Finite elemnet method. Google this
function uav_force_fn() % fn, d_ideal, m_dst, elastic, f_max)
    %test_fields()
    fontsize = 30;
    N= 119; xmax = 60; a = 12.5; dmin= 15; dmax = 20; b = 22.5; c = 35; d = 47.5; e = 50; fmax = 5; x = 1;
%    N = 150; xmax = 75.5; a = 12.5; dmin= 20; dmax = 30; b = 37.5; c = 50; d = 62.5; e = 75; fmax = 5; x = 1;

    step = 0.5;
    xaxis = 1:step:xmax;
    res = zeros(1, N);
    
    for i = 1:N
        res(i) = helper(a, dmin, dmax, b, c, d, e, xaxis(i), fmax, x);
    end
    mfig = figure() %'Position', get(0, 'Screensize')); 
    plot(xaxis, res, 'r', 'LineWidth',2);

    title("Force magnitude vs inter-UAV distance", 'FontSize', fontsize, 'Interpreter', 'latex');
    xlabel("Distance", 'FontSize', fontsize, 'Interpreter', 'latex');
    ylabel("Force", 'FontSize', fontsize, 'Interpreter', 'latex');
    set(gca, 'FontSize', fontsize); 
    text(dmin-2, 1, "$$D_{min}$$", 'FontSize', fontsize, 'Interpreter', 'latex');
    text(dmax, -1, "$$D_{max}$$", 'FontSize', fontsize, 'Interpreter', 'latex');
    text(a, -6, "a", 'FontSize', fontsize+5, 'Interpreter', 'latex');
    text(b, +6, "b", 'FontSize', fontsize+5, 'Interpreter', 'latex');
    text(c, 11, "c", 'FontSize', fontsize+5, 'Interpreter', 'latex');
    text(d, +6, "d", 'FontSize', fontsize+5, 'Interpreter', 'latex');
    text(e, +1, "e", 'FontSize', fontsize+5, 'Interpreter', 'latex');
    line([0 60], [0,0], 'LineWidth', 1);
    line([a,a], [-15, 15], 'LineStyle',':', 'LineWidth',1)
    line([dmin,dmin], [-15, 15], 'LineStyle',':', 'LineWidth',1)
    line([dmax,dmax], [-15, 15], 'LineStyle',':', 'LineWidth',1)
    line([c,c], [-15, 15], 'LineStyle',':', 'LineWidth',1)
    line([d,d], [-15, 15], 'LineStyle',':', 'LineWidth',1)
    line([e,e], [-15, 15], 'LineStyle',':', 'LineWidth',1)
    fmax = helper(a, dmin, dmax, b, c, d, e, c, fmax, x);
    line([0, 60], [fmax, fmax], 'LineStyle',':', 'LineWidth',1);
    line([0, 60], [-fmax, -fmax], 'LineStyle',':', 'LineWidth',1);
    yticks([-fmax, 0, fmax]);
    yticklabels(["-Fmax", "0", "Fmax"]);
    %yticklabels(["$- F_{max}$", "0", "$F_{max}$"], 'Interpreter', 'latex');
    %xticks([a, dmin, dmax, b, c, d, e]);
%    Fr = getframe(mfig);
%    imwrite(Fr.cdata, 'force_function.pdf')
end

function test_fields() % d_ideal, elastic, f_max)
    figure;
    hold on;
    x = 0:1:10;
    y = zeros(1, 11);
    plot(x, y);
    x = 0:0.1:5;
    z = sinh(x);
    plot(10+ x, z/2);
    q = zeros(1, 25);
    for i = 0:1:24
        q(i+1) = (i / (1 + abs(i))) * 50; 
    end
    plot((0:1:24) + 15 , q+ 37);
    k = 1;
    for i = 24: -1: 0
        q(k) = (i / (1 + abs(i))) * 50;
        k = k+1;
    end
    plot((0:1:24) + 40 , q + 37);
    
    x = 5: -0.1: 0;
    z = sinh(x);
    plot(64 + (0:0.1:5), z/2);
    
    x = 0:-0.1: -5;
    z = sinh(x);
    plot(x, z/2);
    k = 1;
    for i = 0:-1:-24
        q(k) = (i / (1 + abs(i))) * 50;
        k = k + 1;
    end
    plot((0:-1:-24) - 5, q - 37.1);
end

function f = helper(a, dmin, dmax, b, c, d, e, mydst, fmax, deno)
    y1 = sinh(b-dmax)/deno;
    %x = (c-b);
    %y2 = y1 + (x/1 + abs(x)) * fmax;
    
    if mydst < a
        x = mydst - a; % x will be negative;
        f = (x / (1 + abs(x))) * fmax - y1;
    elseif mydst < dmin
        f = sinh(mydst - dmin)/deno;
    elseif mydst < dmax
        f = 0;
    elseif mydst < b
        f = sinh(mydst - dmax)/deno;
    elseif mydst < c
        x = mydst - b;
        f = (x/ (1 + abs(x))) * fmax + y1;
    elseif mydst < d
        x = d - mydst;
        f = (x / (1 + abs(x))) * fmax + y1;
    elseif mydst < e
        f = sinh(e-mydst)/deno;
    else
        f = 0;
    end
end
