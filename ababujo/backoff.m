function backoff()
    dloc = [100; 0; 0];
    sloc = [0; 0; 0];
    tloc = sloc;
    T_ub = 0.5;
    T_x = 0.2;
    results = zeros(11, 5);
    Xi = 1:11;
    Yi = 1:6;
    
    X = 0:10:100;
    RADIUS = 0:10:50;
    figure();
    xlabel("X");
    ylabel("Y");
    zlabel("Z");
    hold on;
    for i = Xi
        x = X(i);
        k = 1;
        for j = Yi
            radius = RADIUS(j);
            r2 = power(radius, 2);
            for y = 0:radius
                z = sqrt(r2 - power(y, 2));
                pt = [x; y; z]; 
                results(i, k) = coordinated_boff(dloc, sloc, tloc, pt, T_ub, T_x);
                k = k + 1;
                scatter3(x, y, z);
            end
        end
    end
    figure
    contour(results);
end


function boff_time = coordinated_boff(dloc, sloc, tloc, pt, T_ub, t_x)
    % https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_vector_formulation
      u = dloc - sloc;
      AP = pt - sloc;
      distance_from_line = norm(cross(AP, u)) / norm(u);
      dfs = norm(AP);
      
      p = sqrt(abs(power(dfs, 2) - power(distance_from_line, 2)));

      sdd = norm(dloc - sloc);
        
      t_ub_1 = T_ub * (sdd - p) /sdd;

      u1 = dloc - tloc;
      AP1 = pt - tloc;
      distance_from_new_line = norm(cross(AP1, u1)) / norm(u1);
      tdd = norm(u1);
      
      t_ub_2 = t_x * distance_from_new_line/ tdd;
      boff_time = t_ub_1 + t_ub_2;      
end
