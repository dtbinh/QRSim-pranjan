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
        message_loss = 1  % pranjan. Message shall be lost or not? 0 = NO, 1 = YES.
        repeat_plume_msg = 0 % pranjan. DEPRECATED. Repeatedly send plumeDetected message. 0= NO, 1 = YES.
        send_plume_detected = 0 % pranjan. DEPRECATED. Should a drone Send a plume detected message? 0=NO, 1= Yes
        send_coordinates = 1 % pranjan. Each drone shall advertise it's x,y,z coordinates
        dist_scale = 8   % 1 unit on the 3D display is 'dist_scale' meters in actual.
        number_of_drones = 0;  % Let it be 0 and SHOULD NOT be used at any new place. It was a quick workaround to change the number of drones at run time.
        radio_propagation_cons_sigma = 2; % Standard deviation for the gaussian random variable N in path loss calculation.
        radio_propagation_cons_mean = -20; %  Mean for the gaussian radom variable N in path loss calculation.
    end
end

