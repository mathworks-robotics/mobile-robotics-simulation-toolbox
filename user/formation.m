%% EXAMPLE: Multi-Robot Swarm Behavior
% Copyright 2018 The MathWorks, Inc.

%% Create a multi-robot environment
numRobots = 6;
env = MultiRobotEnv(numRobots);
env.robotRadius = 0.15;
env.showTrajectory = true;

%% Initialization
% Number of robot teams
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:25;        % Time array                

% Initialize poses randomly, and add bias to each "team"
poses = [10*(rand(2,numRobots) - 0.5); ...
         pi*rand(1,numRobots)];

%% Simulation loop
vel = zeros(3,numRobots);
for idx = 2:numel(tVec)
    
    % Update the environment
    env(1:numRobots, poses);
    xlim([-8 8]);   % Without this, axis resizing can slow things down
    ylim([-8 8]); 
    
    % Read the sensor and execute the controller for each robot
    for rIdx = 1:numRobots
       vel(:,rIdx) = swarmTeamController(poses,rIdx);
    end
    
    % Discrete integration of pose
    poses = poses + vel*sampleTime;

end

%% Helper function: Robot Controller Logic
function vel = swarmTeamController(poses,rIdx)
    
    % Unpack the robot's pose and team index
    pose = poses(:,rIdx);
    % If there are no detections, turn in place
    v = 0;
    w = 2;
    
    % Convert to global velocity
    vel = bodyToWorld([v;0;w],pose);

end