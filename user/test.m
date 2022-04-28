numRobots = 10;
% graph_matrix_semi = zeros(numRobots,numRobots);
% for row = 1:numRobots-2
%     graph_matrix_semi(row,row) = 1;
%     graph_matrix_semi(row,row+1) = 1;                                                                                                                                                                                            
%     graph_matrix_semi(row,row+2) = 1;
% end
% graph_matrix_semi(numRobots-1,numRobots-1) = 1;
% graph_matrix_semi(numRobots-1,numRobots) = 1;
% graph_matrix_semi(numRobots,numRobots) = 1;
% graph_matrix_semi(1,numRobots) = 1;
% try_total = 100;
% c_old = Cal_Control_Matrix(numRobots,graph_matrix_semi);
% for try_index=1:try_total
%     graph_matrix_semi_2 = graph_matrix_semi;
%     for i=1:numRobots
%         graph_matrix_semi_2(1,i)=0;
%     end
%     rand_n = fix(numRobots * rand());
%     graph_matrix_semi_2(1,rand_n+1) = 1;
%     rand_n = round(numRobots * rand());
%     graph_matrix_semi_2(1,rand_n+1) = 1;
%     graph_matrix_semi_2(1,1) = 1;
%     c_new = Cal_Control_Matrix(numRobots,graph_matrix_semi_2);
%     c_tested(1:2,:)= c_new(1:2,:);
%     c_tested(3:2*numRobots,:) = c_old(3:2*numRobots,:);
%     eigs(c_tested)
%     c_old = c_new;
% end
poses = [5*(rand(2,numRobots) - 0.5); ...
         pi*rand(1,numRobots)];
  Nearest_Detection(3,poses,2)
  
function result = Nearest_Detection(self_id,poses,nearest_num)
    [~,col] = size(poses);
    pose_self = repmat(poses(:,self_id),[1,col]);
    poses_diff = poses - pose_self;
    for i=1:col
        poses_diff(3,i) = norm(poses_diff(1:2,i));
    end
    p_v=linspace(1,col,col);
    poses_diff
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
    result = p_v(2:3);
end