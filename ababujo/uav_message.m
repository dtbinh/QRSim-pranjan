classdef uav_message
    %MESSAGE that is to be sent from one UAV to another
    %   Detailed explanation goes here
    
    properties
        id          % unique uuid 
        timestamp   % timestamp when the message originated
        src         % Source UAV no.
        dest        % Destination UAV no.
        HTL         % Hops to Live
        origin_coord  % The x,y,z coordinate of the src UAV when the message was transmitted
        type        % 1= plumeDetected, 2=CoordinateUpdate
        data          % The data
        hop_count     %  % Number of hops this packet made to reach the destination.
        boff_time       % Not used for now.
        can_update      % Dummy, just to make it compatile with functions accepting geo_message.
    end
    
    methods
        function obj = uav_message(sim_state, src, dest, HTL, data, type, mark_points)
            %MESSAGE Construct an instance of this class
            %   Detailed explanation goes here
            obj.id = char(java.util.UUID.randomUUID);
            obj.timestamp = datetime('now');
            obj.src = src;
            obj.dest = dest;  % 0 means broadcast message
            obj.HTL = HTL;
            obj.hop_count = 0;
            obj.origin_coord = sim_state.platforms{src}.getX(1:3);
            obj.data = data;
            obj.type = type;
            obj.can_update = 0;
            if mark_points== 1
                sloc = sim_state.platforms{src}.getX(1:3);
                dloc = sim_state.platforms{dest}.getX(1:3);
                scatter3(sloc(1), sloc(2), sloc(3)-2, 60, 'Magenta', 'filled');
                scatter3(dloc(1), dloc(2), dloc(3)-2, 60, "*", 'Magenta');
            end
            
        end
    end
end

