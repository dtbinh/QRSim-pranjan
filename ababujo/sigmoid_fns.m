function sigmoid_fns()
    hold on;
    R = 1;
    N = 200;
    for c=20:20
        range = -N/2: 1: N/2;
        res = zeros(length(range));
        for x = -N/2: 1: N/2
            res(x+N/2+1) = helper_2(x, c);            
        end
        plot(range, res);
    end
    hold off;
end


function y = helper(x, c)
    y = x / (c + abs(x));
end

function y = helper_1(x, c)
    y = 1- (x / (c + abs(x)));
end

function y = helper_2(x, c)
    y = -1 + (x / (c + abs(x)));
end