classdef Pelican<Steppable & Platform
    % Class that implementatios dynamic and sensors of an AscTec Pelican quadrotor
    % The parameters are derived from the system identification of one of
    % the UCL quadrotors
    %
    % Pelican Properties:
    % X   - state = [px;py;pz;phi;theta;psi;u;v;w;p;q;r;thrust]
    %       px,py,pz         [m]     position (NED coordinates)
    %       phi,theta,psi    [rad]   attitude in Euler angles right-hand ZYX convention
    %       u,v,w            [m/s]   velocity in body coordinates
    %       p,q,r            [rad/s] rotational velocity  in body coordinates
    %       thrust           [N]     rotors thrust
    %
    % eX  - estimated state = [~px;~py;~pz;~phi;~theta;~psi;0;0;0;~p;~q;~r;0;~ax;~ay;~az;
    %                          ~h;~pxdot;~pydot;~hdot]
    %       ~px,~py,~pz      [m]     position estimated by GPS (NED coordinates)
    %       ~phi,~theta,~psi [rad]   estimated attitude in Euler angles right-hand ZYX convention
    %       0,0,0                    placeholder (the uav does not provide velocity estimation)
    %       ~p,~q,~r         [rad/s] measured rotational velocity in body coordinates
    %       0                        placeholder (the uav does not provide thrust estimation)
    %       ~ax,~ay,~az      [m/s^2] measured acceleration in body coordinates
    %       ~h               [m]     estimated altitude from altimeter NED, POSITIVE UP!
    %       ~pxdot           [m/s]   x velocity from GPS (NED coordinates)
    %       ~pydot           [m/s]   y velocity from GPS (NED coordinates)
    %       ~hdot            [m/s]   altitude rate from altimeter (NED coordinates)
    %
    % U   - controls  = [pt,rl,th,ya,bat]
    %       pt  [-0.89..0.89]  [rad]   commanded pitch
    %       rl  [-0.89..0.89]  [rad]   commanded roll
    %       th  [0..1]         unitless commanded throttle
    %       ya  [-4.4,4.4]     [rad/s] commanded yaw velocity
    %       bat [9..12]        [Volts] battery voltage
    %
    % Pelican Methods:
    %    Pelican(objparams) - constructs object
    %    reset()            - resets all the platform subcomponents
    %    setX(X)            - reinitialise the current state and noise
    %    isValid()          - true if the state is valid
    %    getX()             - returns the state (noiseless)
    %    getEX()            - returns the estimated state (noisy)
    %    getEXasX()         - returns the estimated state (noisy) formatted as the noiseless state
    %
    properties (Constant)
        CONTROL_LIMITS = [-0.9,0.9; -0.9,0.9; 0,1; -4.5,4.5; 9,12]; %limits of the control inputs
        SI_2_UAVCTRL = [-1/degsToRads(0.025);-1/degsToRads(0.025);4097;-1/degsToRads(254.760/2047);1]; % conversion factors
        BATTERY_RANGE = [9,12]; % range of valid battery values volts
        % The parameters of the system dynamics are defined in the
        % pelicanODE function
        G = 9.81;    %  gravity m/s^2
        MASS = 1.68; %  mass of the platform Kg
        F_MAX = 8;   %  pranjan. Unit: Newton. The magnitude of the maximum force (Push or pull) that a drone shall experience when it goes out of range. i.e. <d_min or >d_max
        F_PLUME = 5; % pranjan. Newton. The force experienced when a drone gets a plume detected message.
        labels = {'px','py','pz','phi','theta','psi','u','v','w','p','q','r','thrust'};
    end
    
    properties (Access = public)
        % Source: http://www.ti.com/lit/ds/symlink/wl1805mod.pdf section 5.7
        % 18 Mbps OFDM, TYP transmitter power,  
        transmitter_strength = 17;  % pranjan in Dbm. WL18xxMOD WiLink™ Wi-Fi®  Used in Beaglebone black
        receiver_threshold = -87; % pranjan in Dbm. same source. section 5.6.
        transmission_frequency = 2400; % MHz
        d_min = 1;     % if distance between A and B is less than d_min, both experience a PUSH force.
        d_max = 35;     % If diatance between A and B is larger than d_max, both experience a PULL force.
        % Currently d_min and d_max are arbitrary. TODO: Decide on an
        % appropriate value. This should be different for each drone and
        % depend on the transmitter strength.
        in_msg_queue;   % pranjan.
        out_msg_queue;  % pranjan.
        messages;       % pranjan. The messages which are in the queue.
        tr_msgs;         % pranjan. The messages that have been transmitted by this node.
        uav_coord;      % pranjan. A platform shall store the coordinates of other UAVs. This info shall be collected through messages.
        plume_coord;    % pranjan. A platform shall store the coordinates where plumes have been detected.
        peer_contact_time; % pranjan. The last time this peer had contacted.
        distances = [];      % pranjan. Ideal distance to other drones.
        elasticity = 0.1;     % pranjan. Explanation coming soon....
        target_velocity = [2;0;0];  % pranjan. Initial velocities of the drones.
    end
    properties (Access = protected)
        gpsreceiver; % handle to the gps receiver
        aerodynamicTurbulence;  % handle to the aerodynamic turbulence
        ahars ;      % handle to the attitude heading altitude reference system
        graphics;    % handle to the quadrotor graphics
        a;           % linear accelerations in body coordinates [ax;ay;az]
        collisionD;  % distance from any other object that defines a collision
        dynNoise;    % standard deviation of the noise dynamics
        behaviourIfStateNotValid = 'warning'; % what to do when the state is not valid
        prngIds;     %ids of the prng stream used by this object
        stateLimits; % 13 by 2 vector of allowed values of the state
        X;           % state [px;py;pz;phi;theta;psi;u;v;w;p;q;r;thrust]
        eX;          % estimated state  [~px;~py;~pz;~phi;~theta;~psi;0;0;0;~p;~q;~r;0;~ax;~ay;~az;~h;~pxdot;~pydot;~hdot]
        valid;       % the state of the platform is invalid
        graphicsOn;  % true if graphics is on
        senseD;  % ababujo: Added to change the capability to sense an obstacle or other drone
    end
    
    methods (Access = public)
        function obj = Pelican(objparams)
            % constructs the platform object and initialises its subcomponent
            % The configuration of the type and parameters of the subcomponents are read
            % from the platform config file e.g. pelican_config.m
            %
            % Example:
            %
            %   obj=Pelican(objparams);
            %                objparams.dt - timestep of this object
            %                objparams.on - 1 if the object is active
            %                objparams.aerodynamicturbulence - aerodynamicturbulence parameters
            %                objparams.sensors.ahars - ahrs parameters
            %                objparams.sensors.gpsreceiver - gps receiver parameters
            %                objparams.graphics - graphics parameters
            %                objparams.stateLimits - 13 by 2 vector of allowed values of the state
            %                objparams.collisionDistance - distance from any other object that defines a collision
            %                objparams.dynNoise -  standard deviation of the noise dynamics
            %                objparams.state - handle to simulator state
            %                objparams.transmitter_strength -
            
            obj=obj@Steppable(objparams);
            obj=obj@Platform(objparams);
            
            obj.prngIds = [1;2;3;4;5;6] + obj.simState.numRStreams;
            obj.simState.numRStreams = obj.simState.numRStreams + 6;
            
            assert(isfield(objparams,'stateLimits'),'pelican:nostatelimits',...
                'the platform config file must define the stateLimits parameter');
            obj.stateLimits = objparams.stateLimits;
            
            assert(isfield(objparams,'collisionDistance'),'pelican:nocollisiondistance',...
                'the platform config file must define the collisionDistance parameter');
            obj.collisionD = objparams.collisionDistance;
            obj.senseD = objparams.senseDistance;
            assert(isfield(objparams,'dynNoise'),'pelican:nodynnoise',...
                'the platform config file must define the dynNoise parameter');
            obj.dynNoise = objparams.dynNoise;
            obj.in_msg_queue = [];  % pranjan. Initialize a message queue on each platform.
            obj.out_msg_queue = [];
            obj.messages = containers.Map();
            obj.tr_msgs = containers.Map();
            %obj.transmitter_strength = 20;  % pranjan TODO: Initialize it through objparams.
            obj.uav_coord = zeros(3, obj.simState.task.N4);
            obj.plume_coord = zeros(3, obj.simState.task.N4);
            obj.peer_contact_time = datetime(zeros(1,obj.simState.task.N4), 0, 0);
            for idx = 1: obj.simState.task.N4
                obj.peer_contact_time(idx) = datetime('now');
            end
            
            if(isfield(objparams,'behaviourIfStateNotValid'))
                obj.behaviourIfStateNotValid = objparams.behaviourIfStateNotValid;
            end
            
            %instantiation of sensor and wind objects, with some "manual" type checking
            
            % TURBULENCE
            objparams.aerodynamicturbulence.DT = objparams.DT;
            objparams.aerodynamicturbulence.dt = objparams.dt;
            objparams.aerodynamicturbulence.state = objparams.state;
            if(objparams.aerodynamicturbulence.on)
                
                assert(isfield(objparams.aerodynamicturbulence,'type'),'pelican:noaerodynamicturbulencetype',...
                    'the platform config file must define an aerodynamicturbulence.type ');
                
                limits = obj.simState.environment.area.getLimits();
                objparams.aerodynamicturbulence.zOrigin = limits(6);
                
                tmp = feval(objparams.aerodynamicturbulence.type, objparams.aerodynamicturbulence);
                if(isa(tmp,'AerodynamicTurbulence'))
                    obj.aerodynamicTurbulence = tmp;
                else
                    error('c.aerodynamicturbulence.type has to extend the class AerodynamicTurbulence');
                end
            else
                obj.aerodynamicTurbulence = feval('AerodynamicTurbulence', objparams.aerodynamicturbulence);
            end
            
            % AHARS
            assert(isfield(objparams.sensors,'ahars')&&isfield(objparams.sensors.ahars,'on'),'pelican:noahars',...
                'the platform config file must define an ahars');
            objparams.sensors.ahars.DT = objparams.DT;
            assert(isfield(objparams.sensors.ahars,'type'),'pelican:noaharstype',...
                'the platform config file must define an ahars.type');
            objparams.sensors.ahars.state = objparams.state;
            tmp = feval(objparams.sensors.ahars.type,objparams.sensors.ahars);
            if(isa(tmp,'AHARS'))
                obj.ahars = tmp;
            else
                error('c.sensors.ahars.type has to extend the class AHRS');
            end
            
            % GPS
            assert(isfield(objparams.sensors,'gpsreceiver')&&isfield(objparams.sensors.gpsreceiver,'on'),'pelican:nogpsreceiver',...
                'the platform config file must define a gps receiver if not needed set gpsreceiver.on = 0');
            objparams.sensors.gpsreceiver.DT = objparams.DT;
            objparams.sensors.gpsreceiver.state = objparams.state;
            if(objparams.sensors.gpsreceiver.on)
                assert(~strcmp(class(obj.simState.environment.gpsspacesegment),'GPSSpaceSegment'),...
                    'pelican:nogpsspacesegment','the task config file must define a gpsspacesegment if a gps receiver is in use');
                
                assert(isfield(objparams.sensors.gpsreceiver,'type'),'pelican:nogpsreceivertype',...
                    'the platform config file must define a gpsreceiver.type');
                tmp = feval(objparams.sensors.gpsreceiver.type,objparams.sensors.gpsreceiver);
                if(isa(tmp,'GPSReceiver'))
                    obj.gpsreceiver = tmp;
                else
                    error('c.sensors.gpsreceiver.type has to extend the class GPSReceiver');
                end
            else
                obj.gpsreceiver = feval('GPSReceiver',objparams.sensors.gpsreceiver);
            end
            
            % GRAPHICS
            assert(isfield(objparams,'graphics')&&isfield(objparams.graphics,'on'),'pelican:nographics',...
                'the platform config file must define a graphics parameter if not needed set graphics.on = 0');
            objparams.graphics.DT = objparams.DT;
            objparams.graphics.state = objparams.state;
            if(objparams.graphics.on)
                obj.graphicsOn = 1;
                assert(isfield(objparams.graphics,'type'),'pelican:nographicstype',...
                    'the platform config file must define a graphics.type');
                obj.graphics=feval(objparams.graphics.type,objparams.graphics);
            else
                obj.graphicsOn = 0;
            end
        end
        
        function X = getX(obj,varargin)
            % returns the state (noiseless)
            % X = [px;py;pz;phi;theta;psi;u;v;w;p;q;r;thrust]
            %
            % Examples
            %    allX = obj.getX(); returns the whole state vector
            %  shortX = obj.getX(1:3); returns only the first three elements of teh state vector
            %
            if(isempty(varargin))
                X = obj.X;
            else
                X = obj.X(varargin{1});
            end
        end
        
        function eX = getEX(obj,varargin)
            % returns the estimated state (noisy)
            % eX = [~px;~py;~pz;~phi;~theta;~psi;0;0;0;~p;~q;~r;0;~ax;~ay;~az;~h;~pxdot;~pydot;~hdot]
            %
            % Examples
            %    allX = obj.getEX(); returns the whole estimated state vector
            %  shortX = obj.getEX(1:3); returns only the first three elements of the estimated state vector
            %
            if(isempty(varargin))
                eX = obj.eX;
            else
                eX = obj.eX(varargin{1});
            end
        end
        
        function X = getEXasX(obj,varargin)
            % returns the estimated state (noisy) formatted as the noiseless state
            % eX = [~px;~py;-~h;~phi;~theta;~psi;~u;~v;~w;~p;~q;~r]
            %
            % Examples
            %    allX = obj.getEXasX(); returns the whole 12 elements state vector
            %  shortX = obj.getEXasX(1:3); returns only the first three elements of the state vector
            %
            uvw = dcm(obj.X)*[obj.eX(18:19);-obj.eX(20)];
            X = [obj.eX(1:2);-obj.eX(17);obj.eX(4:6);uvw;obj.eX(10:12)];
            if(~isempty(varargin))
                X = X(varargin{1});
            end
        end
        
        function iv = isValid(obj)
            % true if the state is valid
            iv = obj.valid;
        end
        
        function obj = setX(obj,X)
            % reinitialise the current state and noise
            %
            % Example:
            %
            %   obj.setState(X)
            %       X - platform new state vector [px,py,pz,phi,theta,psi,u,v,w,p,q,r,thrust]
            %           if the length of the X vector is 12, thrust is initialized automatically
            %           if the length of the X vector is 6, all the velocities are set to zero
            %
            
            assert((size(X,1)==6)||(size(X,1)==12)||(size(X,1)==13),'pelican:wrongsetstate',...
                'setState() on a pelican object requires an input of length 6, 12 or 13 instead we have %d',size(X,1));
            
            assert(obj.thisStateIsWithinLimits(X),'pelican:settingoobstate',...
                'the state passed through setState() is not valid (i.e. out of limits)');
            
            if(size(X,1)==6)
                X = [X;zeros(6,1)];
            end
            
            if(size(X,1)==12)
                X = [X;abs(obj.MASS*obj.G)];
            end
            
            obj.X = X;
            
            % set things
            obj.gpsreceiver.setState(X);
            obj.ahars.setState(X);
            obj.aerodynamicTurbulence.setState(obj.X);
            
            obj.a  = zeros(3,1);
            
            
            % now rest to make sure components are initialised correctly
            obj.gpsreceiver.reset();
            obj.aerodynamicTurbulence.reset();
            obj.ahars.reset();
            obj.resetAdditional();
            
            % get measurements
            estimatedAHA = obj.ahars.getMeasurement([obj.X;obj.a]);
            estimatedPosNED = obj.gpsreceiver.getMeasurement(obj.X);
            
            obj.eX = [estimatedPosNED(1:3);estimatedAHA(1:3);zeros(3,1);...
                estimatedAHA(4:6);0;estimatedAHA(7:10);estimatedPosNED(4:5);estimatedAHA(11)];
            
            obj.valid = 1;
            
            % clean the trajectory plot if any
            if(obj.graphicsOn)
                obj.graphics.reset();
            end
            
            obj.bootstrapped = 1;
        end
        
        function obj = reset(obj)
            % resets all the platform subcomponents
            %
            % Example:
            %   obj.reset();
            %
            assert(false,'pelican:rset',['A platform ca not be simply reset since that would its state undefined',...
                ' use setX instead, that will take care of resetting what necessary']);
        end
        
        function d = getCollisionDistance(obj)
            % returns collision distance
            d = obj.collisionD;
        end
        
        function d = getSensingDistance(obj)
            % ababujo: returns the distance uptp which this platform can sense
            d = obj.senseD;
        end
        
        function p = get_message_reception_probability(obj, T, D, F, Dth)
            sigma = 2;     % Standard deviation for the gaussian random variable N
            mmean = -10;       % Mean for the gaussian radom variable N
            RecPower = obj.get_rec_power(T, D, F, mmean, sigma);
            if RecPower  > Dth
                p = 1;
            else
                p = 0;
            end 
        end

        function RecPower = get_rec_power(~, T, D, F, mmean, sigma)
            % T in Dbm
            % D in m
            % F in MHz
            FSPL = 20 * log10(D/1000) + 20 * log10(F) + 32.44; % Free space path loss
            N = normrnd(mmean, sigma);
            %fprintf(" %f %f", FSPL, N);
            RecPower = T - FSPL + N;
        end
        
        function boff_time = random_boff_time(~, T_ub)
            boff_time = T_ub * rand();
        end
        
        function boff_time = coordinated_boff_time(obj, T_ub, pt, msg)
            % https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_vector_formulation
              u = msg.dloc - msg.sloc;
              AP = pt - msg.sloc;
              distance_from_line = norm(cross(AP, u)) / norm(u);
              
              distance_from_source = pdist([pt'; msg.sloc'], 'euclidean');
              p = sqrt(abs(power(distance_from_source, 2) - power(distance_from_line, 2))) * obj.simState.dist_scale;
              
              t_ub_1 = T_ub * p /msg.src_dst_dist;
              
              u1 = msg.tloc - msg.sloc;
              AP1 = pt - msg.tloc;
              distance_from_new_line = norm(cross(AP1, u1)) / norm(u1);
              tr_dst_distance = pdist([msg.tloc'; msg.dloc'], 'euclidean');
              boff_time = t_ub_1 * distance_from_new_line / tr_dst_distance;
        end
        
        function success = send_one_hop_message(obj, msg, transmitter, dest, T_ub, boff_type)
            if dest ~= transmitter && dest ~= msg.src && isKey(obj.simState.platforms{dest}.messages, msg.id) == 0 && isKey(obj.simState.platforms{dest}.tr_msgs, msg.id) == 0
                    % the message has not been already heard or transmitted
                    % by the destination.
                dest_coord = obj.simState.platforms{dest}.getX(1:3);
                transmitter_coord = obj.simState.platforms{transmitter}.getX(1:3);
                D = pdist([transmitter_coord'; dest_coord'], 'euclidean'); % Eucledian Distance
                D = D * obj.simState.dist_scale; % Scale D      
                T = obj.simState.platforms{transmitter}.transmitter_strength; % Souce Transmission strength.
                Dth = obj.simState.platforms{dest}.receiver_threshold;
                F = obj.simState.platforms{transmitter}.transmission_frequency; % Transmission Frequency
                if obj.simState.message_loss == 1
                    reception_probability = obj.get_message_reception_probability(T, D, F, Dth);
                else
                    reception_probability = 1;
                end
                if reception_probability < 0.5
                    success = 0;
                else
                    success = 1;
                    switch boff_type
                        case 1 % "random"
                             %boff_time =  datetime('now') +  T_ub * seconds(rand());
                             %msg.boff_time = boff_time;
                             boff_time = seconds(obj.random_boff_time(T_ub));
                             msg.boff_time = datetime('now') +  boff_time;
                             %dt = msg.boff_time - msg.timestamp;
                             %dt.Format = 'hh:mm:ss.SSSSSSSSS';
                             %dt
                        case 2 % "coordinated"
                            boff_time = seconds(obj.coordinated_boff_time(T_ub, transmitter_coord, msg));
                            msg.boff_time = datetime('now') + boff_time;
                        case 3 % "coordinated_random"
                            boff_time = obj.coordinated_boff_time(T_ub, transmitter_coord, msg);
                            msg.boff_time = datetime('now') +  seconds(obj.random_boff_time(boff_time));
                        otherwise 
                            msg.boff_time = msg.timestamp;
                    end
                    if msg.can_update == 1 && msg.src ~= transmitter
                        msg.tloc = obj.simState.platforms{transmitter}.getX(1:3);
                        D = pdist([msg.tloc'; msg.dloc'], 'euclidean') * obj.simState.dist_scale/2;
                        petal_width = msg.petal_width_percent * D / 100;
                        msg.minor_axis = petal_width;
                        msg.major_axis = sqrt(power(D,2) + power(petal_width, 2));
                    end
                    obj.simState.platforms{dest}.messages(msg.id) = msg;
                end
            end
        end
        
        function success = send_message(obj, msg, transmitter, dest)
            if transmitter ~= dest && msg.HTL > 0
                msg.HTL = msg.HTL - 1;
                dest_coord = obj.simState.platforms{dest}.getX(1:3);
                transmitter_coord = obj.simState.platforms{transmitter}.getX(1:3);
                D = pdist([transmitter_coord'; dest_coord'], 'euclidean'); % Eucledian Distance
                D = D * obj.simState.dist_scale; % Scale D      
                T = obj.simState.platforms{transmitter}.transmitter_strength; % Souce Transmission strength.
                Dth = obj.simState.platforms{dest}.receiver_threshold;
                F = obj.simState.platforms{transmitter}.transmission_frequency; % Transmission Frequency
                if obj.simState.message_loss == 1
                    reception_probability = obj.get_message_reception_probability(T, D, F, Dth);
                else
                    reception_probability = 1;
                end
                if reception_probability < 0.5
                    success = 0;
                else
                    success = 1;
                    % Deliver the message in the in_msg_queue of destination.
                    obj.simState.platforms{dest}.in_msg_queue = [obj.simState.platforms{dest}.in_msg_queue, msg];
                end
            end
        end
        
        function out_message = read_plume_detected_message(obj, uav_no)
            out_message = [];
            if obj.simState.send_plume_detected == 1
                if ~isempty(obj.in_msg_queue)
                    out_message = obj.simState.platforms{uav_no}.in_msg_queue(1);
                    %obj.in_msg_queue(1) = [];  % remove the message from queue.
                end
            end
        end
        
        function out = read_message(obj, uav_no)
            % % Method: Reads all the messages from the in-queue.
            
            % Iterate over each message in the queue.
            % If the message is a plume detected message append it to
            % out_message queue.
            % if the message is a coordinate update message then update the
            % coordinates in uav_coord field.
            nrows = length(obj.simState.platforms{uav_no}.in_msg_queue);
            out = [0,0];
            for i=1:nrows
                msg = obj.simState.platforms{uav_no}.in_msg_queue(i);
                if msg.type == 2  % Coordinate update
                    %last_contact_time = obj.simState.platforms{uav_no}.peer_contact_time(msg.src);
                    %if last_contact_time <= msg.timestamp
                        obj.simState.platforms{uav_no}.uav_coord(:,msg.src) = msg.origin_coord;
                        out(2) = out(2) + 1;
                    %end
                elseif msg.type == 1  % Plume detected message
                    obj.simState.platforms{uav_no}.plume_coord(:,msg.src) = msg.origin_coord;
                    out(1) = out(1) + 1;
                end
            end
            obj.simState.platforms{uav_no}.in_msg_queue = [];
        end
        
        function ob = PlumeDetect(obj, uav_no)
            % ababujo: Global code if this platform detects any obstacle or
            % other drone in its sensing distance
            ob = 0 ;
            
            x = obj.X(1);
            y = obj.X(2);
            z = obj.X(3);
            % [x,y,z] = obj.simState.platforms{1}.getX(1:3);
            xp = obj.simState.environment.area.plume(1,1);
            yp = obj.simState.environment.area.plume(2,1);
            zp = obj.simState.environment.area.plume(3,1);
            r = obj.simState.environment.area.plume(4,1);
            if(sqrt(((xp-x)*(xp-x)) + ((yp-y)*(yp-y)) + ((zp+z)*(zp+z)))< r)  % why is zp + z instead of zp - z?
                % Because, due to some unknown reason, z coordinate of plume is being displayed at -z
                ob = 1;
                if obj.simState.send_plume_detected == 0  % If drones don't send plume detected message.
                    return
                end

                %velocity = obj.getX(7:9);

                if ~isempty(obj.out_msg_queue) && ~obj.simState.repeat_plume_msg
                    % If the drone has already sent a message and should not repeat plume detected message; then don't resend it.
                    return
                end
                
                % Create a message with data as plumedetect
                dest = 0; % 0 means every node is a destination.
                HTL = 1;  % Hops to Live.
                msg = uav_message(obj.simState, uav_no, dest, HTL,  "plumeDetected", 1);
                if isempty(obj.out_msg_queue)
                    obj.out_msg_queue = [obj.out_msg_queue, msg];
                end
                for dst=1:length(obj.simState.platforms)
                    if dst ~= uav_no
                        obj.send_message(msg, uav_no, dst);  % For each other platform call send message
                    end
                end
                %obj.simState.task.p{j} = 1;
                %obj.simState.task.in{j} = 1;
                % else
                %    obj.simState.task.p{j} = 0;
            end
        end
        
        function ob = ObDetect(obj)
            % ababujo: Global code if this platform detects any obstacle or
            % other drone in its sensing distance
            ob = 0 ;
            for j=1:length(obj.simState.platforms),
                for p=1:size(obj.simState.environment.area.obstacles,2),
                    if(abs(abs(obj.simState.environment.area.obstacles(1,p))-abs(obj.simState.platforms{j}.X(1))) < obj.simState.platforms{j}.getSensingDistance())
                        obj.simState.task.f{j} = 1;
                        ob = 1;
                    end
                end
                for i=1:length(obj.simState.platforms),
                    if(obj.simState.platforms{i} ~= obj.simState.platforms{j})
                        if(norm(obj.simState.platforms{i}.X(1:3)-obj.simState.platforms{j}.X(1:3))< obj.simState.platforms{j}.getSensingDistance())
                            obj.simState.task.d{j} = 1;
                            ob = 1;
                        end
                    end
                end
            end
        end
        
        function setTrajectoryColor(obj, color)
            obj.graphics.setTrajectoryColor(color);
        end
        
        function setTrajectoryjLineWidth(obj, lw)
            obj.graphics.setTrajectoryjLineWidth(lw);
        end
    end
    
    methods (Sealed,Access=protected)
        
        function US = scaleControls(obj,U)
            % scales the controls from SI units to what required by the ODE model
            % The dynamic equations (and the real model) require the following input ranges
            % pt  [-2048..2048] 1=4.36332313e-4 rad = 0.25 deg commanded pitch
            % rl  [-2048..2048] 1=4.36332313e-4 rad = 0.25 deg commanded roll
            % th  [0..4096] 1=4.36332313e-4 rad = 0.25 deg commanded throttle
            % ya  [-2048..2048] 1=2.17109414e-3 rad/s = 0.124394531 deg/s commanded yaw velocity
            % bat [9..12] Volts battery voltage
            %
            assert(size(U,1)==5,'pelican:inputoob','wrong size of control inputs should be 5xN\n\tU = [pt;rl;th;ya;bat] \n');
            
            for i = 1:size(U,2),
                assert(~(any(U(:,i)<obj.CONTROL_LIMITS(:,1)) || any(U(:,i)>obj.CONTROL_LIMITS(:,2))),...
                    'pelican:inputoob',['control inputs values not within limits \n',...
                    '\tU = [pt;rl;th;ya;bat] \n\n\tpt  [-0.9..0.9] rad commanded pitch \n\trl  [-0.9..0.9] rad commanded roll \n',...
                    '\tth  [0..1] unitless commanded throttle \n\tya  [-4.5..4.5] rad/s commanded yaw velocity \n\tbat [9..12] Volts battery voltage \n']);
            end
            
            US = U.*obj.SI_2_UAVCTRL;
        end
        
        function valid = thisStateIsWithinLimits(obj,X)
            % returns 0 if the state is out of bounds
            to = min(size(X,1),size(obj.stateLimits,1));
            
            valid = all(X(1:to)>=obj.stateLimits(1:to,1)) && all(X(1:to)<=obj.stateLimits(1:to,2));
            
            
        end
        
        %ababujo: : We dont want to discard all the platforms if there is a
        % collision, but just bring the drone down to the ground
        function coll = inCollision(obj)
            % returns 1 if a collision is occourring
            coll = 0;
            return
            for i=1:length(obj.simState.platforms)
                
                for j=1:length(obj.simState.platforms)
                    if(obj.simState.platforms{i} ~= obj.simState.platforms{j})
                        if(norm(obj.simState.platforms{i}.X(1:3)-obj.simState.platforms{j}.X(1:3))< obj.simState.platforms{j}.getCollisionDistance())
                            coll = 1;
                            fprintf('%d\n',obj.simState.platforms{j}.X(1:3));
                            if norm(obj.simState.platforms{j}.getX(7:9)) > 0.1  % pranjan. Temporary solution. Only bring down the drones which are moving. not the stopped ones.
                                fprintf('Platform %d too close to another platform\n',j);
                                x = obj.simState.platforms{j}.X(1);
                                y = obj.simState.platforms{j}.X(2);
                                z = 0;
                                obj.simState.platforms{j}.setX([x;y;z;obj.simState.platforms{j}.X(4:6)]);
                                obj.simState.platforms{j}.valid = 0.0;
                            end
                        end
                    end
                end
                for p=1:size(obj.simState.environment.area.obstacles,2),
                    if(abs(abs(obj.simState.environment.area.obstacles(1:2,p))-abs(obj.simState.platforms{i}.X(1:2))) < obj.simState.platforms{i}.getCollisionDistance())
                        coll = 1;
                        fprintf('%d\n',obj.simState.platforms{i}.X(1:3));
                        fprintf('Platform %d colliding with obstacle %d\n',i,p);
                        x = obj.simState.platforms{i}.getX(1);
                        y = obj.simState.platforms{i}.getX(2);
                        z = 0;
                        obj.simState.platforms{i}.valid=0;
                        % obj.simState.platforms{i}.setX([x;y;z;obj.simState.platforms{i}.getX(4:6)]);
                        break;
                    end
                end
                
            end
        end
        
        function obj = printStateNotValidError(obj)
            % display state error info
            if(strcmp(obj.behaviourIfStateNotValid,'continue'))
                
            else
                if(strcmp(obj.behaviourIfStateNotValid,'error'))
                    if(obj.inCollision())
                        fprintf('platform state not valid, in collision!\n');
                    else
                        fprintf('platform state not valid, values out of bounds!\n');
                    end
                else
                    if(obj.inCollision())
                        fprintf('warning: platform state not valid, in collision!');
                    else
                        ids = (obj.X(1:12) < obj.stateLimits(:,1)) | (obj.X(1:12) > obj.stateLimits(:,2));
                        problematics = '';
                        for k=1:size(ids)
                            if(ids(k))
                                problematics = [problematics,',',obj.labels{k}]; %#ok<AGROW>
                            end
                        end
                        fprintf(['warning: platform state not valid, values out of bounds (',problematics,')!\n',num2str(obj.X'),'\nNormally this should not happen; ',...
                            'however if you think this is fine and you want to stop this warning use the task parameter behaviourIfStateNotValid\n']);
                    end
                end
            end
        end
    end
    
    methods (Access=protected)
        
        function obj=resetAdditional(obj)
            % used by subclasses to reset additional stuff
        end
        
        function obj=updateAdditional(obj,U)
            % used by subclasses to update additional stuff
        end
        
        function obj=updateAdditionalGraphics(obj,X)
            % used by subclasses to update additional graphics stuff
        end
        
        function obj = update(obj,U)
            % updates the state of the platform and of its components
            %
            % In turns this:
            %  updates turbulence model
            %  updates the state of the platform applying controls
            %  updates local part of gps model
            %  updates ahars noise model
            %  updates the graphics
            %
            % Note:
            %  this method is called automatically by the step() of the Steppable parent
            %  class and should not be called directly.
            %
            
            if(obj.valid)
                
                % do scaling of inputs
                US = obj.scaleControls(U);
                
                if (size(U,1)~=5)
                    error('a 5 element column vector [-2048..2048;-2048..2048;0..4096;-2048..2048;9..12] is expected as input ');
                end
                
                %wind and turbulence this closely mimic the Simulink example "Lightweight Airplane Design"
                % asbSkyHogg/Environment/WindModels
                meanWind = obj.simState.environment.wind.getLinear(obj.X);
                
                obj.aerodynamicTurbulence.step(obj.X);
                turbWind = obj.aerodynamicTurbulence.getLinear(obj.X);
                
                accNoise = obj.dynNoise.*[randn(obj.simState.rStreams{obj.prngIds(1)},1,1);
                    randn(obj.simState.rStreams{obj.prngIds(2)},1,1);
                    randn(obj.simState.rStreams{obj.prngIds(3)},1,1);
                    randn(obj.simState.rStreams{obj.prngIds(4)},1,1);
                    randn(obj.simState.rStreams{obj.prngIds(5)},1,1);
                    randn(obj.simState.rStreams{obj.prngIds(6)},1,1)];
                
                % dynamics
                [obj.X obj.a] = ruku2('pelicanODE', obj.X, [US;meanWind + turbWind; obj.MASS; accNoise], obj.dt);
                
                if(isreal(obj.X)&& obj.thisStateIsWithinLimits(obj.X) && ~obj.inCollision())
                    %if(isreal(obj.X)&& obj.thisStateIsWithinLimits(obj.X) )
                    %ababujo:brought incollision check inside, as we dont
                    %want to discard all the platforms if there is a
                    %collision, but just bring the drone down to the ground
                    if(obj.inCollision())
                        obj.eX = nan(20,1);
                        obj.valid=0;
                        
                        obj.printStateNotValidError();
                    end
                    % AHARS
                    obj.ahars.step([obj.X;obj.a]);
                    
                    estimatedAHA = obj.ahars.getMeasurement([obj.X;obj.a]);
                    
                    % GPS
                    obj.gpsreceiver.step(obj.X);
                    
                    estimatedPosNED = obj.gpsreceiver.getMeasurement(obj.X);
                    
                    %return values
                    obj.eX = [estimatedPosNED(1:3);estimatedAHA(1:3);zeros(3,1);...
                        estimatedAHA(4:6);0;estimatedAHA(7:10);estimatedPosNED(4:5);estimatedAHA(11)];
                    
                    obj.updateAdditional(U);
                    
                    % graphics
                    if(obj.graphicsOn)
                        obj.graphics.update(obj.X);
                        obj.updateAdditionalGraphics(obj.X);
                    end
                    
                    obj.valid = 1;
                else
                    obj.eX = nan(20,1);
                    obj.valid=0;
                    
                    obj.printStateNotValidError();
                end
            end
        end
    end
end