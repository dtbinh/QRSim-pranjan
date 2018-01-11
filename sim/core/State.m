classdef State<handle
    % Data structure that holds the full state of a qrsim simulation
    % The main purpouse of this structure is to to provide a handle to the 
    % simulator state so that ist can be referenced whenever needed
    % withouth wasteful copying of data
    
    properties (Access=public)
        numRStreams;  % number of random streams in teh simulation        
        rStreams;     % the random streams used throughout the simualtion
        DT;           % simulation time step
        t;            % current simulation time
        display3d;    % handle to the 3D graphic figure
        display3dOn;  % true if the display is on
        environment;  % handle to all the environment objects
        environment_; % handle to hidden environment objects
        platforms;    % cell array containing handles to all the platforms
        task;         % handle to the current task
        camerascnt_;  % cameras counter
        obstacles;      %ababujo
        message_loss = 0  % pranjan. Message shall be lost or not? 0 = NO, 1 = YES.
        repeat_plume_msg = 0 % pranjan. Repeatedly send plumeDetected message. 0= NO, 1 = YES.
        send_plume_detected = 0 % pranjan. Should a drone Send a plume detected message? 0=NO, 1= Yes
        send_coordinates = 1 % pranjan. Each drone shall advertise it's x,y,z coordinates
    end
end

