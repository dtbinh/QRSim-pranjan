classdef geo_message
    properties
        id          % unique uuid
        src         % Source UAV no.
        sloc        % Message origin location
        dest        % Destination UAV no.
        dloc        % Message destination location.
        radius      % If dest is 0 then any node in the sphere with center at dloc and radius is a destination.
        tloc        % Last transmitter's location.
        boff_time   % backoff_time before this message is retransmitted.
        timestamp   % timestamp when the message originated
        minor_axis  % semi minor-axis length of the spheroid.
        major_axis  % semi major-axis length of the spheroid.
        src_dst_dist    % Distance between the source and the destination.
        data        % The data
        hop_count   % Number of hops this packet made to reach the destination.
        can_update  % Boolean to indicate whether a transmitter is allowed to modify petal_width, minor_axis, major_axis and tloc
        petal_width_percent % minor_axis * 100 / src_dst_distance
    end
    
    methods
        function obj = geo_message(simState, src, dest, petal_width_percent, data, mark_points, can_update, radius)
            % dest should  be either a scalar or a (3 rows * 1 col) vector
            %petal_width is the SEMI-minor axis length of the prolate
            %spheroid.
            obj.id = char(java.util.UUID.randomUUID);
            obj.src = src;
            obj.sloc = simState.platforms{src}.getX(1:3);
            obj.tloc = obj.sloc;
            if length(dest) == 1
                obj.dest = dest;  % dest is a natural number which represents a drone ID at application level.
                obj.dloc = simState.platforms{dest}.getX(1:3);
            else
                obj.dest = 0;   % 0 means broadcast message
                obj.dloc = dest;    % Any drone inside a spherical volume around this region is a recepient of this message.
            end
            obj.radius = radius; % * simState.dist_scale;
            obj.boff_time = 0;
            obj.timestamp = datetime('now');
            obj.hop_count = 0;
            obj.can_update = can_update;
            
            obj.src_dst_dist = norm(obj.sloc - obj.dloc) * simState.dist_scale;
            D = obj.src_dst_dist / 2;
            
            obj.petal_width_percent = petal_width_percent;
            petal_width = petal_width_percent * obj.src_dst_dist / 100;
            obj.minor_axis = petal_width;
            obj.major_axis = sqrt(power(D,2) + power(petal_width, 2));
            obj.data = data;
            if mark_points == 1
                scatter3(obj.sloc(1), obj.sloc(2), obj.sloc(3)-2, 60, 'Magenta', 'filled');
                scatter3(obj.dloc(1), obj.dloc(2), obj.dloc(3)-2, 60, "*", 'Magenta');
%                 if can_update == 1
%                     draw_spheroid(obj.sloc', obj.dloc', obj.minor_axis/(simState.dist_scale * 2));
%                 end
                
                %fprintf("Distance between the source and the destination = %f\n", D*2);
                %fprintf("Major axis length = %f\n", obj.major_axis);
                %fprintf("Minor axis length = %f\n", obj.minor_axis);
                
                %
                %             [x,y,z] = ellipsoid(p(1), p(2), p(3), obj.major_axis/simState.dist_scale, obj.minor_axis/simState.dist_scale, obj.minor_axis/simState.dist_scale);
                %
                %
%                             p = (obj.sloc + obj.dloc)/2;
%                 
%                             xp = obj.dloc(1);
%                             yp = obj.dloc(2);
%                             zp = obj.dloc(3);
%                 
%                             xc = obj.sloc(1);
%                             yc = obj.sloc(2);
%                             zc = obj.sloc(3);
% 
%                             gamma = rad2deg(atan(sqrt(((xp-xc)^2+(yp-yc)^2)/abs((zp-zc)))));
%                             alpha = rad2deg(atan(sqrt(((zp-zc)^2+(yp-yc)^2)/abs((xp-xc))))) ;
%                             beta = rad2deg(atan(sqrt(((xp-xc)^2+(zp-zc)^2)/abs((yp-yc)))));
%                 
%                             scatter3(p(1), p(2), p(3), 60, "+", 'Magenta');
%                             [x,y,z] = ellipsoid(p(1), p(2), p(3), obj.major_axis/simState.dist_scale, obj.minor_axis/simState.dist_scale, obj.minor_axis/simState.dist_scale);
%                             S = surf(x,y,z);
%                             rotate(S, [1,0,0], -alpha, p);
%                             rotate(S, [0,1,0], -beta, p);
%                             rotate(S, [0,0,1], -gamma, p);
                
            end
        end
    end
end

