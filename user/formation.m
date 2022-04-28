clear;
%% Initialization
% important hyperparas
numRobots = 10;
max_translational_v = 0.5;  % m/s
max_rotational_w = 2.84;     % rad/s
sampleTime = 0.01;              % Sample time [s]
tVec = 0:sampleTime:10000;        % Time array
distance_control_id_pair = [1;5];
rand_id = 2;
nearest_seleted_num = 2;

% multi_env settings
env = MultiRobotEnv(numRobots);
env.robotRadius = 0.15;
env.showTrajectory = false;
env.showDesired = true;
env.showConnection = true;
env.showRealTime = true;

% variable initialization
graph_matrix_semi = zeros(numRobots,numRobots);
vel_xy = zeros(2,numRobots);
vel_vw = zeros(2,numRobots);
d_poses = zeros(3,numRobots);
distance_u = zeros(2,1);
neighbor_list_ordered = zeros(nearest_seleted_num,1);
% calculate base graph_matrix_semi.
for row = 1:numRobots-2
    graph_matrix_semi(row,row) = 1;
    graph_matrix_semi(row,row+1) = 1;                                                                                                                                                                                            
    graph_matrix_semi(row,row+2) = 1;
end
graph_matrix_semi(numRobots-1,numRobots-1) = 1;
graph_matrix_semi(numRobots-1,numRobots) = 1;
graph_matrix_semi(numRobots,numRobots) = 1;
graph_matrix_semi(1,numRobots) = 1;

% read q_desire from files
fid= fopen('formation_data.txt', 'r');    % clear the data in txt
q_desire =fscanf(fid, '%f', [numRobots*2,1]);

% calculate initial pose

% for i=1:numRobots
%     poses(1,i) = q_desire(i*2-1) + randn;
%     poses(2,i) = q_desire(i*2) + randn;
% end
poses = [5*(rand(2,numRobots) - 0.5); ...
         pi*rand(1,numRobots)];

%% Calculate Control Matrix
[matrix_value,connection_list,graph_matrix_semi_value,Solve_Status_list]=Cal_Control_Matrix_For_All(numRobots,graph_matrix_semi,rand_id);
[~,connection_list_col] = size(connection_list);

%% Simulation loop
for idx = 2:numel(tVec)
    % Update the environment
    real_time_now = idx * sampleTime;
    Control_Matrix_Rand =matrix_value(:,:,1);
    Control_Matrix = matrix_value(:,:,1);
    env(1:numRobots,poses,q_desire,graph_matrix_semi,real_time_now);
    xlim([-8 8]);   % Without this, axis resizing can slow things down
    ylim([-8 8]);

    for rIdx = 1:numRobots
        if rIdx == rand_id  % enable rand connection
            neighbor_list=Nearest_Detection(rand_id,poses,nearest_seleted_num);
            neighbor_list_ordered(1,1) = min(neighbor_list(1),neighbor_list(2));
            neighbor_list_ordered(2,1) = max(neighbor_list(1),neighbor_list(2));

            % find the nearest 2 for controlling.
            for neighbor_index=1:connection_list_col
                if connection_list(:,neighbor_index) == neighbor_list_ordered
                    break;
                end
            end
            Control_Matrix_Rand = matrix_value(:,:,neighbor_index);
            graph_matrix_semi = graph_matrix_semi_value(:,:,neighbor_index);
            vel_vw(:,rIdx) = swarmTeamController(poses,rIdx,Control_Matrix_Rand(2*rIdx-1:2*rIdx,:));
        else        
            vel_vw(:,rIdx) = swarmTeamController(poses,rIdx,Control_Matrix(2*rIdx-1:2*rIdx,:));
        end
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

    % manuever control
    if 0
        vel_vw(:,distance_control_id_pair(1)) = [0.1,0]
        vel_vw(:,distance_control_id_pair(2)) = [0.1;0];
    else
        vel_vw(:,distance_control_id_pair(1)) = u1;
        vel_vw(:,distance_control_id_pair(2)) = u2;
    end
    

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
            vel_xy = vel_xy + control_m(1:2,2*neighbor_index-1:2*neighbor_index)*-pose_diff;
        end
    end
    norm_vel = norm(vel_xy);

    % only normalize when the norm of control law is bigger than 1.
    if norm_vel ~=0 && norm_vel >1
        vel_xy = vel_xy ./ norm_vel;
    end

    % try to stop when approaching the target position.
    if norm(vel_xy)<=0.2
        vel_xy(1)=0;
        vel_xy(2)=0;
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

%% Detect the nearest {nearest_num} agents for agent_{self_id}
function result = Nearest_Detection(self_id,poses,nearest_num)
    [~,col] = size(poses);
    pose_self = repmat(poses(:,self_id),[1,col]);
    poses_diff = poses - pose_self;
    for i=1:col
        poses_diff(3,i) = norm(poses_diff(1:2,i));
    end
    p_v=linspace(1,col,col);

    for i =col:-1:col-nearest_num-1
        for j=col:-1:col-i+2
            pose_diff_i = norm(poses_diff(1:2,p_v(j)));
            pose_diff_j = norm(poses_diff(1:2,p_v(j-1)));
            if pose_diff_i<pose_diff_j
                temp = p_v(j);
                p_v(j) = p_v(j-1);
                p_v(j-1) = temp;
            end
        end
    end
    result = [p_v(2);p_v(3)];
end