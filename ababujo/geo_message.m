classdef geo_message
    properties
        id          % unique uuid       
        src         % Source UAV no.
        sloc        % Message origin location
        dest        % Destination UAV no.
        dloc        % Message destination location.
        boff_time   % backoff_time before this message is retransmitted.
        timestamp   % timestamp when the message originated
        minor_axis  % semi minor-axis length of the spheroid.
        major_axis  % semi major-axis length of the spheroid.
        src_dst_dist    % Distance between the source and the destination.
        data        % The data
        hop_count   % Number of hops this packet made to reach the destination.
        repeat_locations % hash table of the drone->location
    end
    
    methods
        function obj = geo_message(simState, src, dest, petal_width_percent, data)
            %petal_width is the SEMI-minor axis length of the prolate
            %spheroid.
            obj.id = char(java.util.UUID.randomUUID);
            obj.src = src;
            obj.sloc = simState.platforms{src}.getX(1:3);
            obj.dest = dest;  % 0 means broadcast message
            obj.dloc = simState.platforms{dest}.getX(1:3);
            obj.boff_time = 0;
            obj.timestamp = datetime('now');
            obj.hop_count = 0;
            
            obj.src_dst_dist = pdist([obj.sloc'; obj.dloc'], 'euclidean') * simState.dist_scale;
            D = obj.src_dst_dist / 2;
            
            petal_width = petal_width_percent * obj.src_dst_dist / 100;
            obj.minor_axis = petal_width;
            obj.major_axis = sqrt(power(D,2) + power(petal_width, 2));
            obj.data = data;
            obj.repeat_locations = containers.Map();
            mark_src_dst = 0;
            if mark_src_dst == 1
                scatter3(obj.sloc(1), obj.sloc(2), obj.sloc(3)-2, 60, 'Magenta', 'filled');
                scatter3(obj.dloc(1), obj.dloc(2), obj.dloc(3)-2, 60, "*", 'Magenta');
            end
            %fprintf("Distance between the source and the destination = %f\n", D*2);
            %fprintf("Major axis length = %f\n", obj.major_axis);
            %fprintf("Minor axis length = %f\n", obj.minor_axis);
            %aaa = (obj.sloc - obj.dloc)/2;
            %[x, y, z] = ellipsoid(aaa(1), aaa(2), aaa(3) ,obj.major_axis, obj.minor_axis, obj.minor_axis, 10);
        end
    end
end

