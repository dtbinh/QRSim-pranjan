% Finite elemnet method. Google this
function f = force_fields(fn, f_max, m_dst, d_min, d_max, d_max_ex)
    if fn == "sigmoid_field"
        f = sigmoid_field(f_max, m_dst, d_min, d_max, d_max_ex);
    elseif fn == "linear_force_field"
        f = linear_force_field(f_max, m_dst, d_min, d_max, d_max_ex);
    elseif fn == "test"
        test_fields(d_ideal, elastic, f_max);
    end
end
  
function test_fields(f_max, d_min, d_max, d_max_ex)
    hold on
    N = 10;
    res = zeros(d_ideal * N);
    d_min = 5;  d_max = 15;    D1 = 17.5;  d_max_ex = 20;
    for m_dst = 1: d_ideal * N
        res(m_dst) = tapered_sigmoid_field(d_min, d_max, D1, d_max_ex, m_dst, f_max);            
    end
    title("Force magnitude vs inter-UAV distance");
    xlabel("distance x units");
    ylabel("Force y units");
    plot(res);

    for m_dst = 1: d_ideal * N
        res(m_dst) = sigmoid_field(f_max, m_dst, d_min, d_max, d_max_ex);            
    end
    %plot(res);
    
    for m_dst = 1: d_ideal * N
        res(m_dst) = linear_force_field(d_ideal, m_dst, elastic, f_max);            
    end
    %plot(res);
    hold off
end

function f = tapered_sigmoid_field(d_min, d_max, D1, DX, m_dst, f_max)
    con = 0.5;
    if m_dst >= d_min && m_dst <= d_max
        f = 0;
    elseif m_dst < d_min
        x = m_dst - d_min;   % x will be negative
        f = (x / (con + abs(x))) * f_max;
    elseif m_dst <= D1
        x = m_dst - d_max;
        f = (x / (con + abs(x))) * f_max;
    else
        if m_dst > DX
            f = 0;
        else
            x = DX - m_dst;
            f = (x / (con + abs(x))) * f_max;
        end
    end
end

function f = sigmoid_field(f_max, m_dst, d_min, d_max, d_max_ex)
    con = 0.5;
    if m_dst >= d_min && m_dst <= d_max
        f = 0;
    elseif m_dst < d_min
        x = m_dst - d_min;
        f = (x/(con + abs(x))) * f_max;
   elseif m_dst > d_max_ex
        f = f_max;
    else
        x = m_dst - d_max;
        f = (x/ (con + abs(x))) * f_max;
    end
end


function F = linear_force_field(f_max, m_dst, d_min, d_max, d_max_ex)
%     buff = d_ideal*(elastic);  % TODO
%     d_max = d_ideal + buff;
%     d_min = buff;
%     d_max_ex = d_max + buff;
    F = 0;
    if m_dst >= d_min && m_dst <= d_max
        F = 0;
    elseif m_dst < d_min
        F = -((f_max/buff) * (d_min - m_dst));
    elseif m_dst > d_max_ex
        F = f_max;
    elseif m_dst > d_max
        F = (f_max/buff) * (m_dst - d_max);
    end
end