
function A_final = Cal_Control_Matrix(agent_number,original_poses)
n = agent_number;  % number of agent
E1 = zeros(2*n,1);  % core base 1
E2 = zeros(2*n,1);  % core base 1
for i=1:n
    E1(2*i-1) = 1;
    E2(2*i) = 1;
end

q_star = [0;(3^0.5);1;(3^0.5);1.5;(3^0.5)/2;1;0;0;0;-0.5;(3^0.5)/2];
rotate_theta = zeros(2,2,n);
for i=1:n
    rotate_theta(:,:,i) = [cos(original_poses(3,i)),-sin(original_poses(3,i));sin(original_poses(3,i)),cos(original_poses(3,i))];
end
world_2_local = blkdiag(rotate_theta(:,:,1),rotate_theta(:,:,2),rotate_theta(:,:,3),rotate_theta(:,:,4),rotate_theta(:,:,5),rotate_theta(:,:,6));
% q_star_local = world_2_local*q_star;
rotate_90 = [0,1;-1,0];
rotate_90_block = blkdiag(rotate_90,rotate_90,rotate_90,rotate_90,rotate_90,rotate_90);
q_star_perpendicular = rotate_90_block*q_star;

N = [q_star,q_star_perpendicular,E1,E2];
[U,S,V] = svd(N);
U_last = U(:,5:end);

graph_matrix = [1,1,0,1,0,1;
                0,1,1,0,1,0;
                0,0,1,1,1,1;
                0,0,0,1,1,0;
                0,0,0,0,1,1;
                0,0,0,0,0,1];
            % graph_matrix = [1,1,1,1,1,1;
            %     0,1,1,1,1,1;
            %     0,0,1,1,1,1;
            %     0,0,0,1,1,1;
            %     0,0,0,0,1,1;
            %     0,0,0,0,0,1];
graph_matrix = graph_matrix.'+graph_matrix;
cvx_begin
    variable A(2*n,2*n) symmetric 
    maximize(lambda_min(U_last' * A * U_last))
    subject to
        A * N == 0;
        A(graph_matrix==0) == 0;
        for row_index=1:n
            for col_index=1:n
                A(2*row_index-1,2*col_index-1) == A(2*row_index,2*col_index);
                A(2*row_index,2*col_index-1) == -1*A(2*row_index-1,2*col_index);
            end
        end
cvx_end
eig(A)
A_final = A;
end
