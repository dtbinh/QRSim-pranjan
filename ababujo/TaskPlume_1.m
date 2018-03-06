classdef TaskPlume_1<Task
    % ababujo: Creating a scenario as follows: Three drones moving towards an obstacle(wall)
    % and on the other side of the obstacle, two drones are moving towards each other.
    % One of the 3 drones does not have any detection capability.
    % The second drone can detect anything within a 5 feet distance from it, and when it does it comes to a sudden stop.
    % The third drone can detect anything within 10feet, and when it does- starts moving the other way.
    % The two drones on the other side of the wall should try to avoid the pending collision in someway.
    % in this task all the sensors are noiseless and the wind is
    % turned off.
    
    properties (Constant)
        durationInSteps = 1700;
        N1 = 0;             % drones without any obstacle sensing capability
        N2 = 0;             % drones with the ability to sense anything in the radius of 10f
        N3 = 0;             % drones with the ability to sense anything in the radius of 5f
        N4 = 9;             % drones when sensing another drone carries straight on assuming the other drones would change its route
        N5 = 0;             % drones when sensing another drone tries to fly above it.
        hfix = 15;           % fix flight altitude
        PENALTY = 1000;      % penalty reward in case of collision
    end
    
    properties (Access=public)
        initialX; % initial state of the uavs
        prngId;   % id of the prng stream used to select the initial positions
        PIDs;  % pid used to control the uavs
        f;  % flags to denote sensor picking up some obstacle on its radar
        d;  % flags to denote sensor picking up some other drone on its radar
        p; % flags to denote sensor comes in contact with some contamination
        in;
        C; % completed array
        L;  % Nodes to the left of the centroid
        R;  % nodes to the right of the centroid
        mesh = 0;
        done = 0;
        time =0;
        vt; % to store target velocities
    end
    
    methods (Sealed,Access=public)
        
        function obj = TaskPlume_1(state)
            obj = obj@Task(state);
        end
        
        function taskparams=init(obj) %#ok<MANU>
            % loads and returns the parameters for the various simulation objects
            %
            % Example:
            %   params = obj.init();
            %          params - the task parameters
            %
            fprintf("I was called");
            taskparams.dt = 0.2; % task timestep i.e. rate at which controls
            % are supplied and measurements are received
            
            taskparams.seed = 0; %set to zero to have a seed that depends on the system time
            
            %%%%% visualization %%%%%
            % 3D display parameters
            taskparams.display3d.on = 1;
            taskparams.display3d.width = 1000;
            taskparams.display3d.height = 600;
            
            %%%%% environment %%%%%
            % these need to follow the conventions of axis(), they are in m, Z down
            % note that the lowest Z limit is the refence for the computation of wind shear and turbulence effects
            taskparams.environment.area.limits = [-60 60 -60 60 -60 0];
            taskparams.environment.area.type = 'BoxWithObstaclesArea';
            
            % originutmcoords is the location of the RVC (our usual flying site)
            % generally when this is changed gpsspacesegment.orbitfile and
            % gpsspacesegment.svs need to be changed
            [E N zone h] = llaToUtm([51.71190;-0.21052;0]);
            taskparams.environment.area.originutmcoords.E = E;
            taskparams.environment.area.originutmcoords.N = N;
            taskparams.environment.area.originutmcoords.h = h;
            taskparams.environment.area.originutmcoords.zone = zone;
            taskparams.environment.area.graphics.type = 'AreaWithObstaclesGraphics';
            taskparams.environment.area.graphics.backgroundimage = 'ucl-rvc-zoom.tif';
            %ababujo:obstacles{Column - X Y Z(h) r}
            % taskparams.environment.area.obstacles = taskparams.environment.area.type.obstacles;
            taskparams.environment.area.obstacles = [ ];
            taskparams.environment.area.plume = [10 40 25 9]';  % x,y,z of center and radius.This value shall override the value in BoxWithObstaclesArea file.
            %taskparams.environment.area.plume = [10 40 25 13]';  % x,y,z of center and radius.This value shall override the value in BoxWithObstaclesArea file.            
            
            % GPS
            % The space segment of the gps system
            taskparams.environment.gpsspacesegment.on = 0; %% NO GPS NOISE!!!
            taskparams.environment.gpsspacesegment.dt = 0.2;
            % real satellite orbits from NASA JPL
            taskparams.environment.gpsspacesegment.orbitfile = 'ngs15992_16to17.sp3';
            % simulation start in GPS time, this needs to agree with the sp3 file above,
            % alternatively it can be set to 0 to have a random initialization
            %taskparams.environment.gpsspacesegment.tStart = Orbits.parseTime(2010,8,31,16,0,0);
            taskparams.environment.gpsspacesegment.tStart = 0;
            % id number of visible satellites, the one below are from a typical flight day at RVC
            % these need to match the contents of gpsspacesegment.orbitfile
            taskparams.environment.gpsspacesegment.svs = [3,5,6,7,13,16,18,19,20,22,24,29,31];
            % the following model is from [2]
            %taskparams.environment.gpsspacesegment.type = 'GPSSpaceSegmentGM';
            %taskparams.environment.gpsspacesegment.PR_BETA = 2000;     % process time constant
            %taskparams.environment.gpsspacesegment.PR_SIGMA = 0.1746;  % process standard deviation
            % the following model was instead designed to match measurements of real
            % data, it appears more relistic than the above
            taskparams.environment.gpsspacesegment.type = 'GPSSpaceSegmentGM2';
            taskparams.environment.gpsspacesegment.PR_BETA2 = 4;       % process time constant
            taskparams.environment.gpsspacesegment.PR_BETA1 =  1.005;  % process time constant
            taskparams.environment.gpsspacesegment.PR_SIGMA = 0.003;   % process standard deviation
            
            % Wind
            % i.e. a steady omogeneous wind with a direction and magnitude
            % this is common to all helicopters
            taskparams.environment.wind.on = 0;  %% NO WIND!!!
            taskparams.environment.wind.type = 'WindConstMean';
            taskparams.environment.wind.direction = degsToRads(45); %mean wind direction, rad clockwise from north set to [] to initialise it randomly
            taskparams.environment.wind.W6 = 0.5;  % velocity at 6m from ground in m/s
            
            %%%%% platforms %%%%%
            % Configuration and initial state for each of the platforms
            
            for i=1:obj.N4,
                taskparams.platforms(i).configfile = 'Detect10_config';
            end
            
            % get hold of a prng stream
            obj.prngId = obj.simState.numRStreams+1;
            obj.simState.numRStreams = obj.simState.numRStreams + 1;
        end
        
        %ababujo: reset() is called initially to iniitialize the positions
        % and in this case the PIDs
        function reset(obj)
            j=0;
            for i=1:floor(obj.N4/3),
                obj.simState.platforms{i}.setX([-10;50;-obj.hfix-10*j;0;0;0]);
                obj.PIDs{i} = VelocityPID(obj.simState.DT);
                obj.f{i} = 0;
                obj.d{i} = 0;
                obj.p{i} = 0;
                obj.in{i} = 0;
                obj.vt{i} = zeros(3,obj.durationInSteps);
                j=j+1;
            end
            j=0;
            for i=floor(obj.N4/3)+1:floor(2*obj.N4/3),
                obj.simState.platforms{i}.setX([-10;40;-obj.hfix-10*j;0;0;0]);
                obj.PIDs{i} = VelocityPID(obj.simState.DT);
                obj.f{i} = 0;
                obj.d{i} = 0;
                obj.p{i} = 0;
                obj.in{i} = 0;
                obj.vt{i} = zeros(3,obj.durationInSteps);
                j=j+1;
            end
            j=0;
            for i=floor(2*obj.N4/3)+1:floor(obj.N4),
                obj.simState.platforms{i}.setX([-10;30;-obj.hfix-10*j;0;0;0]);
                obj.PIDs{i} = VelocityPID(obj.simState.DT);
                obj.f{i} = 0;
                obj.d{i} = 0;
                obj.p{i} = 0;
                obj.in{i} = 0;
                obj.vt{i} = zeros(3,obj.durationInSteps);
                j=j+1;
            end
            for i=1:obj.N4
                my_coord = obj.simState.platforms{i}.getX(1:3);
                for j=1:obj.N4
                    peer_coord = obj.simState.platforms{j}.getX(1:3);   
                    m_dst = pdist([my_coord';peer_coord'], 'euclidean');  % Should be 0 when i==j
                    obj.simState.platforms{i}.distances = [obj.simState.platforms{i}.distances, m_dst];
                    fprintf("Me = %d peer = %d, ideal dist = %f\n", i, j, m_dst);
                end
            end
        end
        
        function acc_mag = get_acceleration_mag(obj, m_dst, me, peer)
            mass = obj.simState.platforms{me}.MASS;
            f_max = obj.simState.platforms{me}.F_MAX;
            d_ideal = obj.simState.platforms{me}.distances(:,peer);
            elastic = obj.simState.platforms{me}.elasticity;
            d_min = d_ideal*(elastic);  % TODO
            d_max = d_ideal*(1+elastic);
            d_max_ex = d_ideal * (1+2*elastic);
            %d_max = obj.simState.platforms{me}.d_max;
            %quart = (d_max - d_min)/8;
            m_base = d_min;
            p_base = d_max_ex - d_max;
            F = 0;
            if m_dst >= d_min && m_dst <= d_max
                F = 0;
            elseif m_dst < d_min
                F = -((f_max/m_base) * (d_min - m_dst));
            elseif m_dst > d_max_ex
                F = f_max;
            elseif m_dst > d_max
                F = (f_max/p_base) * (m_dst - d_max);
            end
            acc_mag = F/mass;
        end
        
        % in step(), the drone takes a step based on its relative position
        % with each other and also to other dronee, obstacles, etc..
        function UU = step(obj,U)
            obj.time = obj.time +1;
            
            N = obj.N4;
            UU = zeros(5,N);
            % ob = obj.simState.platforms{i}.ObDetect();
            for me = 1: N
                ob = obj.simState.platforms{me}.PlumeDetect(me);
                % Check the message queue for any messages.
                obj.simState.platforms{me}.read_message(me);
                %if(ob ==1 || obj.simState.platforms{me}.isValid() == 0) % If a UAV detects a plume, or it collided with another drone, it stops
                if(ob ==1)    
                    UU(:,me) = obj.PIDs{me}.computeU(obj.simState.platforms{me}.getX(),[0;0;0],0);
                    %obj.vt{i} = [obj.vt{i},[0 ;0 ;0]];  % Why this?
                elseif obj.simState.platforms{me}.isValid() == 0
                    UU(:,me) = obj.PIDs{me}.computeU(obj.simState.platforms{me}.getX(),[0;0;0],0);
                else
                    ct = 0;
                    res_coord = [0;0;0];
                    p_acc_vec = [0;0;0];
                    c_acc_vec = [0;0;0];
                    if obj.simState.send_plume_detected == 1 % If drones transmit plume detected message.
                        for peer=1:N  % Check for all the plume detected messages
                            peer_coord = obj.simState.platforms{me}.plume_coord(:,peer);
                            if norm(peer_coord) ~= 0
                                res_coord = res_coord + peer_coord;
                                ct = ct + 1;
                            end
                        end
                        if ct > 0
                            res_coord = res_coord / ct;
                        end
                        if norm(res_coord) ~= 0
                            my_coord = obj.simState.platforms{me}.getX(1:3);
                            vec = res_coord - my_coord;
                            unit_vec = vec/ norm(vec);
                            p_acc_mag = obj.simState.platforms{me}.F_PLUME / obj.simState.platforms{me}.MASS;
                            p_acc_vec = p_acc_mag * unit_vec;
                        end
                    end
                    if obj.simState.send_coordinates == 1  % If drones transmit their coordinates
                        % Check if the current drone is outside of d_min and
                        % d_max range.
                        my_coord = obj.simState.platforms{me}.getX(1:3);
                        c_ct = 0;
                        %c_acc_vec = [0;0;0];
                        for peer=1:N
                            if me ~= peer && obj.simState.platforms{peer}.isValid() == 1
                                peer_coord = obj.simState.platforms{me}.uav_coord(:,peer);
                                if norm(peer_coord) ~= 0
                                    m_dst = pdist([my_coord';peer_coord'], 'euclidean');
                                    c_acc_mag = obj.get_acceleration_mag(m_dst, me, peer);
                                    if c_acc_mag ~= 0
                                        dir_vec = (peer_coord - my_coord);
                                        dir_vec = dir_vec / norm(dir_vec);
                                        c_acc_vec = c_acc_vec + dir_vec * c_acc_mag;
                                        c_ct = c_ct + 1;
                                    end
                                end
                            end
                        end
                        if c_ct > 0
                            c_acc_vec = c_acc_vec / c_ct;
                        end
                    end
                    %res_acc_vec = [0;0;0];
                    if norm(c_acc_vec) ~= 0 && norm(p_acc_vec) ~= 0
                        res_acc_vec = (c_acc_vec + p_acc_vec)/2;
                    else
                        res_acc_vec = c_acc_vec + p_acc_vec;
                    end
                    if norm(res_acc_vec) ~= 0
                        vel_vec_initial = obj.simState.platforms{me}.getX(7:9);
                        vel_vec_target = vel_vec_initial + res_acc_vec * obj.simState.task.dt;
                        UU(:,me) = obj.PIDs{me}.computeU(obj.simState.platforms{me}.getX(), vel_vec_target, 0);
                    else
                        % If UAV_i detected a plume then move towards UAV_i
                        % Else keep moving in the original direction.
                        % UU(:,k) = obj.PIDs{k}.computeU(obj.simState.platforms{k}.getX(),U(:,k),0);
                        UU(:,me) = obj.PIDs{me}.computeU(obj.simState.platforms{me}.getX(),U(:,me),0);
                    end
                end
            end
        end
        
        
        function updateReward(~,~)
            % updates reward
            % in this task we only have a final cost
        end
        
        function r=reward(obj)
            % returns the total reward for this task
            
            valid = 1;
            for i=1:length(obj.simState.platforms)
                valid = valid &&  obj.simState.platforms{i}.isValid();
            end
            
            if(valid)
                r = obj.currentReward ;
            else
                % returning a large penalty in case the state is not valid
                % i.e. one the drones is out of the area, there was a
                % collision or one of the drones has crashed
                r = - obj.PENALTY;
            end
        end
    end
end
