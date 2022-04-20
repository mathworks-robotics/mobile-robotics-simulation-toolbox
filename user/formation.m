clear;
%% Initialization
numRobots = 6;
max_translational_v = 0.22;  % m/s
max_rotational_w = 2.84;     % rad/s
env = MultiRobotEnv(numRobots);
env.robotRadius = 0.15;

env.showTrajectory = false;
env.showDesired = true;
env.showConnection = true;
randomized_matrix= randn(numRobots,numRobots);
graph_matrix_semi = zeros(numRobots,numRobots);
graph_matrix_semi(randomized_matrix>0.5) = 0.5;
graph_matrix_semi = ones(numRobots,numRobots);
% graph_matrix_semi = [1,1,0,1,0,1;
%                 0,1,1,0,1,0;
%                 0,0,1,0,1,1;
%                 0,0,0,1,1,0;
%                 0,0,0,0,1,0;
%                 0,0,0,0,0,1];
graph_matrix_semi = [1,1,0,0,0,1;
                0,1,1,0,0,0;
                0,0,1,1,0,0;
                0,0,0,1,1,0;
                0,0,0,0,1,1;
                0,0,0,0,0,1];
sampleTime = 0.01;              % Sample time [s]
tVec = 0:sampleTime:10000;        % Time array                

% Initialize poses randomly, and add bias to each "team"
poses = [5*(rand(2,numRobots) - 0.5); ...
         pi*rand(1,numRobots)];
vel_xy = zeros(2,numRobots);
vel_vw = zeros(2,numRobots);
d_poses = zeros(3,numRobots);

distance_u = zeros(2,1);
distance_control_id_pair = [1;4];
Control_Matrix = Cal_Control_Matrix(numRobots,graph_matrix_semi);

fid= fopen('formation_data.txt', 'r');    % clear the data in txt
q_desire =fscanf(fid, '%f', [numRobots*2,1]);

%% Simulation loop
for idx = 2:numel(tVec)
    % Update the environment
    env(1:numRobots,poses,q_desire,graph_matrix_semi);
    xlim([-8 8]);   % Without this, axis resizing can slow things down
    ylim([-8 8]);

    for rIdx = 1:numRobots              
        vel_vw(:,rIdx) = swarmTeamController(poses,rIdx,Control_Matrix);
        vel_vw(:,rIdx) = vel_vw(:,rIdx) .* [max_translational_v;max_rotational_w];
        d_poses(:,rIdx) = [cos(poses(3,rIdx)),0;sin(poses(3,rIdx)),0;0,1] * vel_vw(:,rIdx);
    end

    % leader control - distance based 
    pose1 = poses(:,distance_control_id_pair(1));
    pose2 = poses(:,distance_control_id_pair(2));
    desired_pose1 = q_desire(distance_control_id_pair(1)*2-1:distance_control_id_pair(1)*2);
    desired_pose2 = q_desire(distance_control_id_pair(2)*2-1:distance_control_id_pair(2)*2);
    u1 = Calculate_Target_U(desired_pose1,pose1);
    u2 = Calculate_Target_U(desired_pose2,pose2);
    u1 = u1 .* [max_translational_v;max_rotational_w];
    u2 = u2 .* [max_translational_v;max_rotational_w];
    vel_vw(:,distance_control_id_pair(1)) = u1;
    vel_vw(:,distance_control_id_pair(2)) = u2;
    d_poses(:,distance_control_id_pair(1)) = [cos(poses(3,distance_control_id_pair(1))),0;sin(poses(3,distance_control_id_pair(1))),0;0,1] * vel_vw(:,distance_control_id_pair(1));
    d_poses(:,distance_control_id_pair(2)) = [cos(poses(3,distance_control_id_pair(2))),0;sin(poses(3,distance_control_id_pair(2))),0;0,1] * vel_vw(:,distance_control_id_pair(2));

    poses = poses + d_poses*sampleTime;
end

%% Helper function: Robot Controller Logic
function [vel_vw] = swarmTeamController(poses,rIdx,control_m)
    vel_xy = [0;0];
    [~,col]=size(poses);
    for neighbor_index = 1:col
        if neighbor_index ~= rIdx
            pose_diff =(poses(1:2,neighbor_index) - poses(1:2,rIdx));
            vel_xy = vel_xy + control_m(2*rIdx-1:2*rIdx,2*neighbor_index-1:2*neighbor_index)*-pose_diff;
        end
    end
    norm_vel = norm(vel_xy);
    if norm_vel ~=0
        vel_xy = vel_xy ./ norm_vel;
    end
    h_matrix= zeros(2,2);
    h_matrix(:,:) = [   cos(poses(3,rIdx)),...
                        sin(poses(3,rIdx));
                        -sin(poses(3,rIdx)),...
                        cos(poses(3,rIdx)) 
                    ];
    vel_vw = h_matrix  * vel_xy;
end

%% distance controller for leader
function distance_u = Calculate_Distance_U(pose1,pose2,desired_distance)   % return [v,w]
    K_Gain  =  10;
    pose_xy_diff = pose1(1:2) - pose2(1:2);
    co_matrix = [cos(pose1(3)),sin(pose1(3));-sin(pose1(3)),cos(pose1(3))];
    distance_u =  - K_Gain .* (1-desired_distance/norm(pose_xy_diff)) * co_matrix * pose_xy_diff;
    distance_u = distance_u./norm(distance_u);
end

%% target controller for leader
function target_u = Calculate_Target_U(target_pose,pose_now)  % linear control law ;return [v,w]
    pose_xy_diff = target_pose(1:2) - pose_now(1:2);
    if norm(pose_xy_diff) ~=0
        pose_xy_diff = pose_xy_diff./norm(pose_xy_diff);
    end
    co_matrix = [cos(pose_now(3)),sin(pose_now(3));-sin(pose_now(3)),cos(pose_now(3))];
    target_u =   co_matrix * pose_xy_diff;
end
