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
            taskparams.environment.area.plume = [10
                40
                30
                15];  % x,y,z of center and radius. Note this is only for display. The actual plume is in BoxWithObstaclesArea file.
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
        end
        
        % in step(), the drone takes a step based on its relative position
        % with each other and also to other dronee, obstacles, etc..
        
        function UU = step(obj,U)
            obj.time = obj.time +1;
            
            N = obj.N4;
            UU = zeros(5,N);
            Cen = [2,5,8];
            U1 = [3,6,9];
            D1 = [1,4,7];
            Cen2= [4,5,6];
            L1 = [1,2,3];
            R1= [7,8,9];
            N = obj.N4;
            row = sqrt(obj.N4);
            
            % ob = obj.simState.platforms{i}.ObDetect();
            for i = 1: N,
                ob = obj.simState.platforms{i}.PlumeDetect(i);
                % If a UAV detects a plume, it stops
                if(ob ==1) 
                    UU(:,i) = obj.PIDs{i}.computeU(obj.simState.platforms{i}.getX(),[0;0;0],0);
                    
                    obj.vt{i} = [obj.vt{i},[0 ;0 ;0]];  % Why this?
                else
                    % Check the message queue for plume detect messages.
                    msg = obj.simState.platforms{i}.read_message(i, obj.simState);
                    if ~isempty(msg)  % Some message was read.
                        msg_coord = msg.origin_coord;
                        uav_coord = obj.simState.platforms{i}.getX(1:3);
                        %msg_coord(1) = msg_coord(1) + 25;
                        vec = msg_coord - uav_coord;
                        %d_z = msg_coord(3) - uav_coord(3);
                        %d_y = msg_coord(2) - uav_coord(2);
                        %d_x = msg_coord(1) - uav_coord(1);
                        %vec = [d_x, d_y, d_z];
                        vec = vec/ norm(vec);

                        vel_vec = obj.simState.platforms{i}.getX(7:9);
                        vel_mag = norm(vel_vec);
                        vec = vec * vel_mag;
                        
                        
                        %theta = atan2(d_y, d_x);
                        %v_x = obj.simState.platforms{i}.getX(7);
                        %v_z = v_x * cos(theta);
                        %v_y = v_x * sin(theta);
                        UU(:,i) = obj.PIDs{i}.computeU(obj.simState.platforms{i}.getX(), [vel_vec(1); vec(2)/2; vec(3)/3], 0);
                        obj.vt{i} = [obj.vt{i}, [vec(1); vec(2); vec(3)]];  % why this?
                    else
                        % If UAV_i detected a plume then move towards UAV_i
                        % Else keep moving in the original direction.
                        % UU(:,k) = obj.PIDs{k}.computeU(obj.simState.platforms{k}.getX(),U(:,k),0);
                        UU(:,i) = obj.PIDs{i}.computeU(obj.simState.platforms{i}.getX(),U(:,i),0);
                        obj.vt{i} = [obj.vt{i},U(:,i)];
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