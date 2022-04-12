clear
n = 6;  % number of agent
E1 = zeros(2*n,1);  % core base 1
E2 = zeros(2*n,1);  % core base 1
for i=1:n
    E1(i) = 1;
    E2(i) = 1i;
end

q_star = [0+(3^0.5)*1i,1+(3^0.5)*1i,1.5+1.5*1i,1,0,-0.5+0.5*(3^0.5)*1i].';
rotate_90 = 1i;
rotate_90_block = blkdiag(rotate_90,rotate_90,rotate_90,rotate_90,rotate_90,rotate_90);
q_star_perpendicular = rotate_90_block*q_star;

N = [q_star,q_star_perpendicular,E1,E2];
[U,S,V] = svd(N);
U_last = U(:,3:end);
% A = ones(4,4);
% eig(U_last.'*A*U_last)

graph_matrix = [1,1,1,1,1,1;
                0,1,1,1,1,0;
                0,0,1,1,0,0;
                0,0,0,1,1,0;
                0,0,0,0,1,1;
                0,0,0,0,0,1];
%             graph_matrix = [1,1,1,1,1,1;
%                 0,1,1,1,1,1;
%                 0,0,1,1,1,1;
%                 0,0,0,1,1,1;
%                 0,0,0,0,1,1;
%                 0,0,0,0,0,1];
graph_matrix = graph_matrix.'+graph_matrix;
% % l = min( bnds, [], 2 );
% % u = max( bnds, [], 2 );
% % A  = randn(n,n);
% % b = randn(n,1);
cvx_begin
    variable A(n,n) complex symmetric 
    maximize(lambda_min(U_last' * A * U_last))
    subject to
        A * N == 0;
        A(graph_matrix==0) ==0;
        A(1,1) == A(2,1);
cvx_end
eigs(A)