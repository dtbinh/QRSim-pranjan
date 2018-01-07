classdef uav_message
    %MESSAGE that is to be sent from one UAV to another
    %   Detailed explanation goes here
    
    properties
        id          % unique uuid 
        timestamp   % timestamp when the message originated
        src         % Source UAV no.
        dest        % Destination UAV no.
        origin_coord  % The x,y,z coordinate of the src UAV when the message was transmitted
        data          % The string data
    end
    
    methods
        function obj = uav_message(src, sim_state, data)
            %MESSAGE Construct an instance of this class
            %   Detailed explanation goes here
            obj.id = char(java.util.UUID.randomUUID);
            obj.timestamp = datetime('now');
            obj.src = src;
            %obj.dest = dest;
            obj.origin_coord = sim_state.platforms{src}.getX(1:3);
            obj.data = data;
        end
    end
end

