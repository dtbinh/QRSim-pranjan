function force_function()
    test_mysigmoid()
    mass =  1.68;
    f_max = 5;
    dist_scale = 1;
    N = 260;
    % scale m_dst and d_ideal
    recv = zeros(1,N);
    
    d_ideal = [33.000000, 50.000000, 66.000000, 59.908263, 82.800966, 110.000000, 114.843372, 128.280942];  % Scale d_ideal.
    d_ideal = d_ideal * dist_scale;

    elastic = 0.5;
    for x = d_ideal
        for i = 1:N
            recv(i) = helper(x, i*dist_scale, elastic, f_max);
        end
        plot(recv);
        hold on;
    end
end


function f = helper(d_ideal, m_dst, elastic, f_max)
    buff = d_ideal * elastic;
    d_max = d_ideal + buff;
    d_max_2 = d_ideal + 2 * buff;
    
    d_min = d_ideal - buff;
    d_min_2 = d_ideal - 2 * buff;
    
    slope = f_max / buff;
    
    if m_dst >= d_max_2
        f = f_max;
    elseif m_dst <= d_min_2
        f = -f_max;
    elseif m_dst >= d_min && m_dst <= d_max
        f = 0;
    elseif m_dst < d_min
        f = -(slope * (d_min - m_dst));
    else
        f = (slope * (m_dst - d_max));
    end
end

function test_sigmoid()
    x = -10:.01:10;
    %plot(x,sigmoid(x, 0, 1))
    %hold on
    x = 1:100;
    
    plot(x, sigmoid(x, 60, 1))
    hold on
    plot(x, sigmoid(x, 60, 0.1))
end

function y = sigmoid(x, c, a)  % c = 0 , a = 1
    y = 1./(1 + exp(-a.*(x-c)));
end

function test_mysigmoid()
    N = 5;
    res = zeros(1,2*N);
    for i= 1:2*N
        res(i) = mysigmoid(-N + i);
    end
    plot(res);
end
function y = mysigmoid(x)
    y = x / sqrt(1+ x*x);
end
