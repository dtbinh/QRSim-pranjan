% Finite elemnet method. Google this
function f = force_fields(fn, d_ideal, m_dst, elastic, f_max)
    if fn == "skewed_sigmoid_field"
        f = skewed_sigmoid_field(d_ideal, m_dst, elastic, f_max);
    elseif fn == "sigmoid_field"
        f = sigmoid_field(d_ideal, m_dst, elastic, f_max);
    elseif fn == "no_push_sigmoid_field"
        f = no_push_sigmoid_field(d_ideal, m_dst, elastic, f_max);
    elseif fn == "skewed_force_field"
        f = skewed_force_field(d_ideal, m_dst, elastic, f_max);
    elseif fn == "even_force_field"
        f = even_force_field(d_ideal, m_dst, elastic, f_max);
    elseif fn == "test"
        test_fields(d_ideal, elastic, f_max);
    end
end
  
function test_fields(d_ideal, elastic, f_max)
    hold on
    res = zeros(d_ideal * 2);
    for m_dst = 1: 1: d_ideal * 2
        res(m_dst) = skewed_sigmoid_field(d_ideal, m_dst, elastic, f_max);            
    end
    %plot(res);
    for m_dst = 1: 1: d_ideal * 2
        res(m_dst) = sigmoid_field(d_ideal, m_dst, elastic, f_max);            
    end
    plot(res);

    for m_dst = 1: 1: d_ideal * 2
        res(m_dst) = even_force_field(d_ideal, m_dst, elastic, f_max);            
    end
    plot(res);
    for m_dst = 1: 1: d_ideal * 2
        res(m_dst) = skewed_force_field(d_ideal, m_dst, elastic, f_max);            
    end
    %plot(res);
    for m_dst = 1: 1: d_ideal * 2
        res(m_dst) = no_push_sigmoid_field(d_ideal, m_dst, elastic, f_max);            
    end
    %plot(res);
    hold off
end

function f = sigmoid_field(d_ideal, m_dst, elastic, f_max)
    buff = d_ideal * elastic;
    d_max = d_ideal + buff;
    d_min = d_ideal - buff;
    con = 0.5;
    if m_dst >= d_min && m_dst <= d_max
        f = 0;
    elseif m_dst < d_min
        x = m_dst - d_min; % x will be negative
        f = (x / (con + abs(x))) * f_max;
    else
        x = m_dst - d_max;
        f = (x/ (con + abs(x))) * f_max;
    end
    
end

function f = skewed_sigmoid_field(d_ideal, m_dst, elastic, f_max)
    buff = d_ideal * elastic;
    d_max = d_ideal + buff;
    d_min = buff;
    con = 0.5;
    if m_dst >= d_min && m_dst <= d_max
        f = 0;
    elseif m_dst < d_min
        x = m_dst - d_min;
        f = (x/(con + abs(x))) * f_max;
    else
        x = m_dst - d_max;
        f = (x/ (con + abs(x))) * f_max;
    end
end

function f = no_push_sigmoid_field(d_ideal, m_dst, elastic, f_max)
    % y = x / (con + abs(x))
    buff = d_ideal * elastic;
    d_max = d_ideal + buff;
    con = 0.5;
    if m_dst <= d_max
        f = 0;
    else
        x = m_dst - d_max;
        f = (x / (con + abs(x))) * f_max;
    end
end

function F = skewed_force_field(d_ideal, m_dst, elastic, f_max)
    buff = d_ideal*(elastic);  % TODO
    d_max = d_ideal + buff;
    d_min = buff;
    d_max_ex = d_max + buff;
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

function f = no_push_field(d_ideal, m_dst, elastic, f_max)
    % Only exerts PULL forces.
    buff = d_ideal * elastic;
    d_max = d_ideal + buff;
    d_max_2 = d_ideal + 2 * buff;

    slope = f_max / buff;
    if m_dst >= d_max_2
        f = f_max;
    elseif m_dst <= d_max
        f = 0;
    else
        f = (slope * (m_dst - d_max));
    end
end    

function f = even_force_field(d_ideal, m_dst, elastic, f_max)
    % Even force field.
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
