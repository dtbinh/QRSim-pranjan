classdef geo_message
    properties
        id          % unique uuid
        src         % Source UAV no.
        sloc        % Message origin location
        dest        % Destination UAV no.
        dloc        % Message destination location.
        dloc_ts     % timestamp of how old the dloc data is. If an intermediate node (N) has newer location then the N will update it.
        radius      % If dest is 0 then any node in the sphere with center at dloc and radius is a destination.
        tid         % Last transmitter's ID.
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
        min_width % Minimum absolute width of a petal.
    end
    
    methods
        function obj = geo_message(simState, src, dest, petal_width_percent, data, mark_points, can_update, radius, min_width_percent)
            % dest should  be either a scalar or a (3 rows * 1 col) vector
            %petal_width is the SEMI-minor axis length of the prolate
            %spheroid.
            obj.id = char(java.util.UUID.randomUUID);
            obj.src = src;
            obj.sloc = simState.platforms{src}.getX(1:3);
            obj.tid = src;
            obj.tloc = obj.sloc;
            obj.timestamp = datetime('now');
            if length(dest) == 1
                obj.dest = dest;  % dest is a natural number which represents a drone ID at application level.
                %obj.dloc = simState.platforms{dest}.getX(1:3);
                temp_d = simState.platforms{dest}.location_table(dest);
                obj.dloc = temp_d{1};
                obj.dloc_ts = temp_d{2};
                obj.can_update = can_update;
                obj.radius = 0; % * simState.dist_scale;
            else
                obj.dest = 0;   % 0 means broadcast message
                obj.dloc = dest;    % Any drone inside a spherical volume around this region is a recepient of this message.
                obj.dloc_ts = obj.timestamp;
                obj.can_update = 0;
                obj.radius = radius; % * simState.dist_scale;
            end
            obj.boff_time = 0;
            obj.hop_count = 0;
            
            obj.src_dst_dist = norm(obj.sloc - obj.dloc) * simState.dist_scale;
            D = obj.src_dst_dist / 2;
            
            obj.petal_width_percent = petal_width_percent;
            petal_width = petal_width_percent * obj.src_dst_dist / 100;
            obj.min_width = min_width_percent * obj.src_dst_dist / 100;
            
            if can_update == 1
                obj.minor_axis = max(petal_width, obj.min_width);
            else
                obj.minor_axis = petal_width;
            end
            
            obj.major_axis = sqrt(power(D,2) + power(obj.minor_axis, 2));
            obj.data = data;
            if mark_points == 1
                scatter3(obj.sloc(1), obj.sloc(2), obj.sloc(3)-2, 60, 'Magenta', 'filled');
                scatter3(obj.dloc(1), obj.dloc(2), obj.dloc(3)-2, 60, "*", 'Magenta');    
            end
        end
    end
end

