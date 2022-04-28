n=6;
agent_to_be_trans=6;
p_v=linspace(1,n,n);
if agent_to_be_trans ~=1
    p_v(agent_to_be_trans) = p_v(1);
    p_v(1) = agent_to_be_trans;
end
connection_list=zeros(2,(n-1)*(n-2)/2);
index = 1;
for i=1:n-2
    for j=i+1:n-1
        connection_list(1,index) = p_v(i+1);
        connection_list(2,index) = p_v(j+1);
        index = index+1;
    end
end
connection_list