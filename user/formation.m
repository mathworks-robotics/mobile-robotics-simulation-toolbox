%% EXAMPLE: Multi-Robot Swarm Behavior
% Copyright 2018 The MathWorks, Inc.
clear;
%% Create a multi-robot environment
numRobots = 6;
env = MultiRobotEnv(numRobots);
env.robotRadius = 0.15;
env.showTrajectory = false;
graph_matrix = [1,1,0,1,0,1;
                0,1,1,0,1,0;
                0,0,1,1,1,1;
                0,0,0,1,1,0;
                0,0,0,0,1,1;
                0,0,0,0,0,1];
%% Initialization
% Number of robot teams
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:10000;        % Time array                

% Initialize poses randomly, and add bias to each "team"
poses = [10*(rand(2,numRobots) - 0.5); ...
         pi*rand(1,numRobots)];
original_poses = poses;
%% Simulation loop
vel = zeros(3,numRobots);
Control_Matrix = Cal_Control_Matrix(6,original_poses);
for idx = 2:numel(tVec)
    % Update the environment
    env(1:numRobots, poses);
%     xlim([-8 8]);   % Without this, axis resizing can slow things down
%     ylim([-8 8]); 
    axis equal;
    % Read the sensor and execute the controller for each robot
    for rIdx = 1:numRobots
       vel(1:2,rIdx) = swarmTeamController(poses,rIdx,Control_Matrix,original_poses);
    end
    
    % Discrete integration of pose
    poses = poses + vel*sampleTime;
%     if idx>100
%         plot_connection(graph_matrix,poses)
%     end
end

%% Helper function: Robot Controller Logic
function vel = swarmTeamController(poses,rIdx,control_m,original_poses)
    
    % Unpack the robot's pose and team index
    % If there are no detections, turn in place
    % v = 0;
    % w = 2;
    vel_xy = [0;0];
    [row,col]=size(poses);
    
    for neighbor_index = 1:col
        if neighbor_index ~= rIdx
            control_m(2*rIdx-1:2*rIdx,2*neighbor_index-1:2*neighbor_index)
            pose_local = [cos(original_poses(3,rIdx)),sin(original_poses(3,rIdx));-sin(original_poses(3,rIdx)),cos(original_poses(3,rIdx))]*(poses(1:2,neighbor_index) - poses(1:2,rIdx));
%             pose_local =(poses(1:2,neighbor_index) - poses(1:2,rIdx));
            vel_xy = vel_xy + control_m(2*rIdx-1:2*rIdx,2*neighbor_index-1:2*neighbor_index)*-pose_local;
        end
    end
    rIdx
    vel = vel_xy
    norm_vel = norm(vel);
%     if norm_vel<1
%         vel=[0;0];
%     end
    vel = vel ./ norm_vel;
    % Convert to global velocity
    % vel = bodyToWorld([v;0;w],pose);

end

function plot_connection(graph_matrix,poses)
    [row,col]=size(poses);
    for index_1=1:col
        for index_2=index_1:col
            if graph_matrix(index_1,index_2)==1
                x1=poses(1,index_1);
                y1=poses(2,index_1);
                x2=poses(1,index_2);
                y2=poses(2,index_2);
                line([x1,x2],[y1,y2]);
            end
        end
    end

end
