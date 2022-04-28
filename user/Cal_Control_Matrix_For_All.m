
function [matrix_value,connection_list,graph_matrix_semi_value,Solve_Status_list] = Cal_Control_Matrix_For_All(agent_number,graph_matrix_semi,agent_to_be_trans)
%% calculate kernalspace base vector.
    n = agent_number;  % number of agent
    E1 = zeros(2*n,1);  % core base 1
    E2 = zeros(2*n,1);  % core base 2
    for i=1:n
        E1(2*i-1) = 1;
        E2(2*i) = 1;
    end
    fid= fopen('formation_data.txt', 'r');    % clear the data in txt
    q_star =fscanf(fid, '%f', [n*2,1]);  
    rotate_90 = [0,1;-1,0];
    rotate_90_block = rotate_90;
    for i =1:(n-1)
        rotate_90_block = blkdiag(rotate_90_block,rotate_90);  
    end

    q_star_perpendicular = rotate_90_block*q_star;  % core base 4

    N = [q_star,q_star_perpendicular,E1,E2];  % kernal space
    [U,~,~] = svd(N);
    U_last = U(:,5:end);

%% calculate graph_matrix     C(2,n-1)
p_v=linspace(1,n,n);
if agent_to_be_trans ~=1
    p_v(agent_to_be_trans) = p_v(1);
    p_v(1) = agent_to_be_trans;
end
connection_list=zeros(2,(n-1)*(n-2)/2);
Solve_Status_list = zeros(1,(n-1)*(n-2)/2);
index = 1;
for i=1:n-2
    for j=i+1:n-1
        connection_list(1,index) = min(p_v(i+1),p_v(j+1));
        connection_list(2,index) = max(p_v(i+1),p_v(j+1));
        index = index+1;
    end
end
matrix_value = zeros(2*n,2*n,(n-1)*(n-2)/2);
graph_matrix_semi_value = zeros(n,n,(n-1)*(n-2)/2);
for i=1:n
    if i~=agent_to_be_trans
        if agent_to_be_trans<i
            graph_matrix_semi(agent_to_be_trans,i) = 0;
        else
            graph_matrix_semi(i,agent_to_be_trans) = 0;
        end
    end
end
%% convex optimization
    for matrix_num=1:(n-1)*(n-2)/2

        % make graph_matrix
        graph_matrix_semi_modified = graph_matrix_semi;
        if connection_list(1,matrix_num) <agent_to_be_trans
            graph_matrix_semi_modified(connection_list(1,matrix_num),agent_to_be_trans) = 1;
        else
            graph_matrix_semi_modified(agent_to_be_trans,connection_list(1,matrix_num)) = 1;
        end

        if connection_list(2,matrix_num) <agent_to_be_trans
            graph_matrix_semi_modified(connection_list(2,matrix_num),agent_to_be_trans) = 1;
        else
            graph_matrix_semi_modified(agent_to_be_trans,connection_list(2,matrix_num)) = 1;
        end
        graph_matrix = graph_matrix_semi_modified.' + graph_matrix_semi_modified;
        graph_matrix_semi_value(:,:,matrix_num) = graph_matrix_semi_modified;
        % cvx calculation
        cvx_begin
            variable A(2*n,2*n) symmetric
            maximize(lambda_min(U_last' *A * U_last))
            subject to
                A * N == 0;
                lambda_max(U_last' *A * U_last) <= 10000;
                for row_index=1:n
                    for col_index=1:n
                        A(2*row_index-1,2*col_index-1) == A(2*row_index,2*col_index);
                        A(2*row_index,2*col_index-1) == -1*A(2*row_index-1,2*col_index);
                        if(graph_matrix(row_index,col_index)==0)
                            A(2*row_index-1,2*col_index-1) == 0;
                            A(2*row_index-1,2*col_index) == 0;
                            A(2*row_index,2*col_index-1) == 0;
                            A(2*row_index,2*col_index) == 0;
                        end
                    end

                end
        cvx_end
        eigs(A)
        matrix_value(:,:,matrix_num) = A;
        if cvx_status == "Solved"
            Solve_Status_list(1,matrix_num) = 1;
        end
    end
end
