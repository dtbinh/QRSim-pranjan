classdef TaskPlume<Task
    % ababujo: Creating a scenario as follows: Three drones moving towards an obstacle(wall) 
    % and on the other side of the obstacle, two drones are moving towards each other. 
    % One of the 3 drones does not have any detection capability. 
    % The second drone can detect anything within a 5 feet distance from it, and when it does it comes to a sudden stop. 
    % The third drone can detect anything within 10feet, and when it does- starts moving the other way. 
    % The two drones on the other side of the wall should try to avoid the pending collision in someway.
    % in this task all the sensors are noiseless and the wind is
    % turned off.
    
    properties (Constant)
        durationInSteps = 4000;
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
    end
    
    methods (Sealed,Access=public)
        
        function obj = TaskPlume(state)
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
           taskparams.environment.area.plume = [30  
                 50
                 30
                 25];
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
            j =1;
            for i=1:obj.N1,
                taskparams.platforms(j).configfile = 'Detect0_config';
                j = j+1;
            end
            for i=1:obj.N2,
                taskparams.platforms(j).configfile = 'Detect10_config';
                j=j+1;
            end
            for i=1:obj.N3,
                taskparams.platforms(j).configfile = 'Detect5_config';
                j =j+1;
            end
            for i=1:obj.N4,
                taskparams.platforms(j).configfile = 'Detect10_config';
                j =j+1;
            end
            for i=1:obj.N5,
                taskparams.platforms(j).configfile = 'Detect5_config';
                j =j+1;
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
                obj.simState.platforms{i}.setX([-30;50;-obj.hfix-10*j;0;0;0]);
                obj.PIDs{i} = WaypointPID(obj.simState.DT);
                obj.f{i} = 0;
                obj.d{i} = 0;
                obj.p{i} = 0;
                obj.in{i} = 0;
                j=j+1;
            end
            j=0;
            for i=floor(obj.N4/3)+1:floor(2*obj.N4/3),
                obj.simState.platforms{i}.setX([-30;40;-obj.hfix-10*j;0;0;0]);
                obj.PIDs{i} = WaypointPID(obj.simState.DT);
                obj.f{i} = 0;
                obj.d{i} = 0;
                obj.p{i} = 0;
                obj.in{i} = 0;
                j=j+1;
            end
            j=0;
            for i=floor(2*obj.N4/3)+1:floor(obj.N4),
                obj.simState.platforms{i}.setX([-30;30;-obj.hfix-10*j;0;0;0]);
                obj.PIDs{i} = WaypointPID(obj.simState.DT);
                obj.f{i} = 0;
                obj.d{i} = 0;
                obj.p{i} = 0;
                obj.in{i} = 0;
                j=j+1;
            end
        end
        
        function meshform(obj,U)
            N = obj.N4;
            row = sqrt(obj.N4);
            hRow = floor(row/2);
            cen = ceil(N/2);
            UU = zeros(5,N); 
            obj.C = CQueue();
            obj.L = CQueue();
            % filling the Left and Right queue around the centroid
            for j=cen-1:-1:1
                obj.L.push(j);
            end
            obj.R = CQueue();
            for j=cen+1:obj.N4
                obj.R.push(j);
            end
            for j=1:hRow
                x = obj.simState.platforms{cen-(j-1)}.getX(1);
                y = obj.simState.platforms{cen-(j-1)}.getX(2);
                z = obj.simState.platforms{cen-(j-1)}.getX(3);
                UU(:,cen-j) = obj.PIDs{cen-j}.computeU(obj.simState.platforms{cen-j}.getX(),[x;y;z],0);
                % obj.simState.platforms{cen-j}.setX([x;y;z;0;0;0]);
                x = obj.simState.platforms{cen+(j-1)}.getX(1);
                y = obj.simState.platforms{cen+(j-1)}.getX(2);
                z = obj.simState.platforms{cen+(j-1)}.getX(3);
                UU(:,cen+j) = obj.PIDs{cen-j}.computeU(obj.simState.platforms{cen-j}.getX(),[x;y;z],0);
%                obj.L.pop(); 
%                obj.R.pop();
                i=1;
                % Move the other UAvs colser to the centroid
                while (~obj.L.empty()&&(i<obj.L.size()))
                    r = obj.L.pop();
                    q = obj.L.pop();
                    x = obj.simState.platforms{r}.getX(1);
                    y = obj.simState.platforms{r}.getX(2);
                    z = obj.simState.platforms{r}.getX(3);
                    UU(:,q) = obj.PIDs{q}.computeU(obj.simState.platforms{q}.getX(),[x;y;z],0);
                    obj.L.push(r);
                    obj.L.push(q);
                    i = i+1;
                end
                while (~obj.R.empty()&&(i<obj.R.size()))
                    r = obj.R.pop();
                    q = obj.R.pop();
                    x = obj.simState.platforms{r}.getX(1);
                    y = obj.simState.platforms{r}.getX(2);
                    z = obj.simState.platforms{r}.getX(3);
                    UU(:,q) = obj.PIDs{q}.computeU(obj.simState.platforms{q}.getX(),[x;y;z],0);
                    obj.L.push(r);
                    obj.L.push(q);
                    i = i+1;
                end
            end     
            
        end
        
        % in step(), the drone takes a step based on its relative position
        % with each other and also to other dronee, obstacles, etc..
       
        function UU = step(obj,U)
            obj.time = obj.time +1;
            
            N = obj.N4;
            UU = zeros(5,N); 
            
           % ob = obj.simState.platforms{i}.ObDetect();
           for i = 1: N,
               ob = obj.simState.platforms{i}.PlumeDetect();
               for k = 1:N
                   count = 0;
                   if(obj.p{k}==1)
                        for j=1:N
                           if(obj.p{j}==1)
                               count =count+1;
                               fprintf('j: %d\n',j);
                           end
                        end
                        if(count<N)
                            x = obj.simState.platforms{k}.getX(1);
                            y = obj.simState.platforms{k}.getX(2);
                            z = obj.simState.platforms{k}.getX(3);
                       
                            UU(:,k) = obj.PIDs{k}.computeU(obj.simState.platforms{k}.getX(),[x;y;z],0);
                        end
                   else
                       
                            UU(:,k) = obj.PIDs{k}.computeU(obj.simState.platforms{k}.getX(),U(:,k),0);
                      % end
                   end
                   if (count == N)
                       %[latGridInDegrees, longGridInDegrees] = GridSphere(12+25);
                       %[lat,lon,alt] = ecef2lla(obj.simState.platforms{k}.getX(1),obj.simState.platforms{k}.getX(2),obj.simState.platforms{k}.getX(3));
                       %nnIndices = FindNearestNeighbors(latGridInDegrees, ...
                       %longGridInDegrees, neighborLatsInDegrees, neighborLongsInDegrees);
                       [x,y,z] = sphere;
                       s = surf(obj.simState.environment.area.plume(4,1)*x+obj.simState.environment.area.plume(1,1),...
                                obj.simState.environment.area.plume(4,1)*y+obj.simState.environment.area.plume(2,1),...
                                obj.simState.environment.area.plume(4,1)*z-obj.simState.environment.area.plume(3,1),...
                                'FaceAlpha',0.5,'EdgeColor','none','FaceColor','y');
                                    obj.in{i} =  0;
                                    x = s.XData;
                                    y = s.YData;    
                                    z = s.ZData;
                                    co = size(x);
                                    co = co(1)*co(2);
                                    co = floor(co/N) ;
                                    for j=1:N,
                                        rt = j*co;
                                        U(:,j) = [x(rt);y(rt);z(rt)] ;
                                        UU(:,j) = obj.PIDs{j}.computeU(obj.simState.platforms{j}.getX(),U(:,j),0);
                                       % UU(:,j) = obj.PIDs{j}.computeU(obj.simState.platforms{j}.getX(),[x;y;z],0);
                                   % for j=1:obj.N4,
                                      %  rt = k*count;
                                   %     pz = z(randi(numel(z)));
                                     
                                    %    if(pz<0)
                                    %        U(:,j) = [ x(randi(numel(x))); y(randi(numel(y))); pz] ;
                                        
                                    %        UU(:,j) = obj.PIDs{j}.computeU(obj.simState.platforms{j}.getX(),U(:,j),0);
                                    %    end
                                    end
                   end
                end
            end
        end

        
        
        
        

        
        function UU = step1(obj,U)
            obj.time = obj.time +1;
            if(obj.mesh == 0)
                meshform(obj,U);
                obj.mesh = 1;
            end
            N = obj.N4;
            UU = zeros(5,N); 
           % ob = obj.simState.platforms{i}.ObDetect();
           for i = 1: obj.N4,
               ob = obj.simState.platforms{i}.PlumeDetect();
               ob1 = obj.simState.platforms{i}.ObDetect();
           end
            for i=1:obj.N4,
                if(obj.simState.platforms{i}.isValid())
                        if(obj.p{i}==1)
                            [x,y,z] = sphere;
                        
                             s = surf(obj.simState.environment.area.plume(4,1)*x+obj.simState.environment.area.plume(1,1),...
                                obj.simState.environment.area.plume(4,1)*y+obj.simState.environment.area.plume(2,1),...
                                obj.simState.environment.area.plume(4,1)*z-obj.simState.environment.area.plume(3,1),...
                                'FaceAlpha',0.5,'EdgeColor','none','FaceColor','y');
                        
                            x = s.XData;
                            y = s.YData;    
                            z = s.ZData;
                            count = size(x);
                            count = count(1)*count(2);
                            count = floor(count/obj.N4) ;
                            for k=1:obj.N4,
                                rt = k*count;
                               U(:,k) = [x(rt);y(rt);z(rt)] ;
                               UU(:,k) = obj.PIDs{k}.computeU(obj.simState.platforms{k}.getX(),U(:,k),0);
                            end  
                        
                       
                        else
                            if(obj.in{i} == 1)
                                x = obj.simState.platforms{i}.getX(1);
                                y = obj.simState.platforms{i}.getX(2);
                                z = obj.simState.platforms{i}.getX(3);
                                UU(:,i) = obj.PIDs{i}.computeU(obj.simState.platforms{i}.getX(),[x;y;z],0);
                            end
                        end
                        y = obj.simState.platforms{i}.getX(2);
                        if(y>100)
                            obj.done = 1;
                            for k=1:obj.N4,
                               U(:,k) = [15*k;-120;-10] ;
                                UU(:,k) = obj.PIDs{k}.computeU(obj.simState.platforms{k}.getX(),U(:,k),0);
                            end   
                            break;
                        end
                        if(obj.in{i} == 0)
                            for j=1:obj.N4
                                if(obj.in{j} ==  1)
                                   [x,y,z] = sphere;
                         s = surf(obj.simState.environment.area.plume(4,1)*x+obj.simState.environment.area.plume(1,1),...
                                obj.simState.environment.area.plume(4,1)*y+obj.simState.environment.area.plume(2,1),...
                                obj.simState.environment.area.plume(4,1)*z-obj.simState.environment.area.plume(3,1),...
                                'FaceAlpha',0.5,'EdgeColor','none','FaceColor','y');
                                    
                              %      s = surf(obj.simState.environment.area.plume(1,1)*x+obj.simState.environment.area.plume(4,1)/1.1,...
                               %         obj.simState.environment.area.plume(2,1)*y+obj.simState.environment.area.plume(4,1)/1.1,...
                                %        obj.simState.environment.area.plume(3,1)*z-obj.simState.environment.area.plume(4,1)/1,1,...
                                 %       'FaceAlpha',0.5,'EdgeColor','none','FaceColor','y');
                                   
                                    
                                    obj.in{i} =  0;
                                    x = s.XData;
                                    y = s.YData;    
                                    z = s.ZData;
                                   % count = size(x);
                                   % count = count(1)*count(2);
                                   % count = floor(count/obj.N4) ;
                                    for k=1:obj.N4,
                                      %  rt = k*count;
                                      pz = z(randi(numel(z)));
                                        if(pz<0)
                                            U(:,k) = [ x(randi(numel(x))); y(randi(numel(y))); pz] ;
                                        
                                            UU(:,k) = obj.PIDs{k}.computeU(obj.simState.platforms{k}.getX(),U(:,k),0);
                                        end
                                    end
                                 else      
                                    UU(:,i) = obj.PIDs{i}.computeU(obj.simState.platforms{i}.getX(),U(:,i),0);
                                end
                            end
                          
                        end    
                    
                 end
            end
        end
        
        
        
           function UU = step2(obj,U)
            obj.time = obj.time +1;
            
            N = obj.N4;
            UU = zeros(5,N); 
            
           % ob = obj.simState.platforms{i}.ObDetect();
           for i = 1: N,
               ob = obj.simState.platforms{i}.PlumeDetect();
               for k = 1:N
                   count = 0;
                   if(obj.p{k}==1)
                        for j=1:N
                           if(obj.p{j}==1)
                               count =count+1;
                               fprintf('j: %d\n',j);
                           end
                        end
                        if(count<4)
                            x = obj.simState.platforms{k}.getX(1);
                            y = obj.simState.platforms{k}.getX(2);
                            z = obj.simState.platforms{k}.getX(3);
                       
                            UU(:,k) = obj.PIDs{k}.computeU(obj.simState.platforms{k}.getX(),[x;y;z],0);
                        end
                   else
                       
                            UU(:,k) = obj.PIDs{k}.computeU(obj.simState.platforms{k}.getX(),U(:,k),0);
                      % end
                   end
                   if (count == 4)
                       %[latGridInDegrees, longGridInDegrees] = GridSphere(12+25);
                       %[lat,lon,alt] = ecef2lla(obj.simState.platforms{k}.getX(1),obj.simState.platforms{k}.getX(2),obj.simState.platforms{k}.getX(3));
                       %nnIndices = FindNearestNeighbors(latGridInDegrees, ...
                       %longGridInDegrees, neighborLatsInDegrees, neighborLongsInDegrees);
                       [x,y,z] = sphere;
                       s = surf(obj.simState.environment.area.plume(4,1)*x+obj.simState.environment.area.plume(1,1),...
                                obj.simState.environment.area.plume(4,1)*y+obj.simState.environment.area.plume(2,1),...
                                obj.simState.environment.area.plume(4,1)*z-obj.simState.environment.area.plume(3,1),...
                                'FaceAlpha',0.5,'EdgeColor','none','FaceColor','y');
                                    obj.in{i} =  0;
                                    x = s.XData;
                                    y = s.YData;    
                                    z = s.ZData;
                                    co = size(x);
                                    co = co(1)*co(2);
                                    co = floor(co/N) ;
                                   % for j=1:N,
                                   %     rt = j*co;
                                  %      U(:,j) = [x(rt);y(rt);z(rt)] ;
                                  %     UU(:,j) = obj.PIDs{j}.computeU(obj.simState.platforms{j}.getX(),U(:,j),0);
                                       % UU(:,j) = obj.PIDs{j}.computeU(obj.simState.platforms{j}.getX(),[x;y;z],0);
                                    for j=1:obj.N4,
                                      %  rt = k*count;
                                        %pz = z(randi(numel(z)));
                                        X(j) = obj.simState.platforms{j}.getX(1);
                                        Y(j) = obj.simState.platforms{j}.getX(2);
                                        Z(j) = obj.simState.platforms{j}.getX(3);
                                        
                                       % if(pz<0)
                                        %    U(:,j) = [ x(randi(numel(x))); y(randi(numel(y))); pz] ;
                                        
                                        %    UU(:,j) = obj.PIDs{j}.computeU(obj.simState.platforms{j}.getX(),U(:,j),0);
                                       % end
                                    end
                                    [center,radius] = sphereFit([X,YZ]');
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