classdef TaskPlume_2<Task
    % ababujo: Single point of contact
    
    properties (Constant)
        durationInSteps = 1700;
        N1 = 0;             % drones without any obstacle sensing capability
        N2 = 0;             % drones with the ability to sense anything in the radius of 10f
        N3 = 0;             % drones with the ability to sense anything in the radius of 5f
        N4 = 16;             % drones when sensing another drone carries straight on assuming the other drones would change its route
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
        
        function obj = TaskPlume_2(state)
            obj = obj@Task(state);
        end
        
        function taskparams=init(obj) %#ok<MANU>
            % loads and returns the parameters for the various simulation objects
            %
            % Example:
            %   params = obj.init();
            %          params - the task parameters
            %
            
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
            taskparams.environment.area.limits = [-120 120 -120 120 -120 0];
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
                15];
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
            r = 0;
            N = floor(sqrt(obj.N4));
            i = 1;
            for tr=1:N
                for j=0:N-1
                    obj.simState.platforms{i}.setX([-10;50-10*j;-obj.hfix-10*r;0;0;0]);
                    obj.PIDs{i} = VelocityPID(obj.simState.DT);
                    obj.f{i} = 0;
                    obj.d{i} = 0;
                    obj.p{i} = 0;
                    obj.in{i} = 0;
                    obj.vt{i} = zeros(3,obj.durationInSteps);
                    i=i+1;
                end
                r =r+1;
            end
            
        end
        
        % in step(), the drone takes a step based on its relative position
        % with each other and also to other dronee, obstacles, etc..
        
        function UU = step(obj,U)
            obj.time = obj.time +1;
            
            N = obj.N4;
            UU = zeros(5,N);
            Cen = [5,6,7,8];
            U1 = [13,14,15,16];
            D1 = [1,2,3,4];
            Cen2= [2,6,10,14];
            L1 = [1,5,9,13];
            R1= [4,8,12,16];
            row = sqrt(obj.N4);
            
            % ob = obj.simState.platforms{i}.ObDetect();
            for i = 1: N,
                ob = obj.simState.platforms{i}.PlumeDetect();
                % If a UAV detects a plume, it stops
                if(obj.p{i}==1)
                    UU(:,i) = obj.PIDs{i}.computeU(obj.simState.platforms{i}.getX(),[0;0;0],0);
                    
                    obj.vt{i} = [obj.vt{i},[0 ;0 ;0]];
                else
                    for j=1:N,
                        temp =0;
                        if((i~=j)&&(obj.p{j}==1))
                            x1 = obj.simState.platforms{i}.getX(1);
                            y1 = obj.simState.platforms{i}.getX(2);
                            z1 = obj.simState.platforms{i}.getX(3);
                            
                            x2 = obj.simState.platforms{j}.getX(1);
                            y2 = obj.simState.platforms{j}.getX(2);
                            z2 = obj.simState.platforms{j}.getX(3);
                            
                            %dist  = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
                            %step = dist/5;
                            
                            u2 = obj.simState.platforms{j}.getX(7);
                            v2 = obj.simState.platforms{j}.getX(8);
                            w2 = obj.simState.platforms{j}.getX(9);
                            
                            %heading angle
                            psi2 = obj.simState.platforms{j}.getX(6);
                            % rotationg the wp to body coordinates
                            dr = ((x2-x1)^2+(y2-y1)^2)^0.5;
                            a = (atan2((y2-y1),(x2-x1)) - psi2);
                            
                            %bx = dr * cos(a);
                            %by = dr * sin(a);
                            %desu = 2*bx;
                            %desv = 2*by;
                            
                            %Sensing angle
                            dotprod = x1*x2 + y1*y2 + z1*z2;
                            mod_denom = sqrt(x1^2 + y1^2 + z1^2 )* sqrt(x2^2 + y2^2 + z2^2 ) ;
                            cosnum = dotprod/mod_denom;
                            Psi = acos(cosnum);
                            
                            %desired Heading angle(new)
                            desPsi = Psi - psi2;
                            
                            UU(:,j) = obj.PIDs{j}.computeU(obj.simState.platforms{j}.getX(),[2*u2;2*v2;-w2],desPsi);
                            temp = 1;
                        end
                        
                        if((temp ==0)&&(obj.p{j}==0))
                            UU(:,j) = obj.PIDs{j}.computeU(obj.simState.platforms{j}.getX(),U(:,j),0);
                        end
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