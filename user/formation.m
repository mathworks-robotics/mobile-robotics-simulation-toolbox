clear;
%% Initialization
numRobots = 6;
env = MultiRobotEnv(numRobots);
env.robotRadius = 0.15;
env.showTrajectory = true;
graph_matrix_semi = [1,1,0,1,0,1;
                0,1,1,0,1,0;
                0,0,1,1,1,1;
                0,0,0,1,1,0;
                0,0,0,0,1,1;
                0,0,0,0,0,1];
sampleTime = 0.01;              % Sample time [s]
tVec = 0:sampleTime:10000;        % Time array                

% Initialize poses randomly, and add bias to each "team"
poses = [10*(rand(2,numRobots) - 0.5); ...
         pi*rand(1,numRobots)];
vel_xy = zeros(2,numRobots);
vel_vw = zeros(2,numRobots);
d_poses = zeros(3,numRobots);
h_matrix= zeros(2,2);

Control_Matrix = Cal_Control_Matrix(6,graph_matrix_semi);

%% Simulation loop
for idx = 2:numel(tVec)
    % Update the environment
    env(1:numRobots, poses);
    xlim([-8 8]);   % Without this, axis resizing can slow things down
    ylim([-8 8]);

    for rIdx = 1:numRobots
        h_matrix(:,:) = [   cos(poses(3,rIdx)),...
                            sin(poses(3,rIdx));
                            -sin(poses(3,rIdx)),...
                            cos(poses(3,rIdx)) 
                        ];
        vel_xy(:,rIdx) = swarmTeamController(poses,rIdx,Control_Matrix);
        vel_vw(:,rIdx) = h_matrix  * vel_xy(:,rIdx);
        d_poses(:,rIdx) = [cos(poses(3,rIdx)),0;sin(poses(3,rIdx)),0;0,1] * vel_vw(:,rIdx);
    end
    
    % Discrete integration of pose
    poses = poses + d_poses*sampleTime;

    if idx>1000
        plot_connection(graph_matrix_semi,poses)
    end
end

%% Helper function: Robot Controller Logic
function [vel_xy] = swarmTeamController(poses,rIdx,control_m)
    vel_xy = [0;0];
    [row,col]=size(poses);
    for neighbor_index = 1:col
        if neighbor_index ~= rIdx
            pose_diff =(poses(1:2,neighbor_index) - poses(1:2,rIdx));
            vel_xy = vel_xy + control_m(2*rIdx-1:2*rIdx,2*neighbor_index-1:2*neighbor_index)*-pose_diff;
        end
    end
    norm_vel = norm(vel_xy);
    vel_xy = vel_xy ./ norm_vel;
end

%% plot the connection 
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
