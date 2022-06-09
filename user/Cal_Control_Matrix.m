
function A_final = Cal_Control_Matrix(agent_number,graph_matrix_semi)
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

%% calculate graph_matrix

    graph_matrix = graph_matrix_semi.' + graph_matrix_semi;

%% convex optimization
    cvx_begin
        variable A(2*n,2*n) symmetric
        maximize(lambda_min(U_last' *A * U_last))
        subject to
            A * N == 0;
            lambda_max(U_last' *A * U_last) <= 1000;
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
    
    A_final = A;
end
