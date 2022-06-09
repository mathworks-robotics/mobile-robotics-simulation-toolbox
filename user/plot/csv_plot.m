clear;
close all;
folder_name = '2022-05-30-10-34-59';
end_flag = 12800;
fid_q_real = importdata(strcat([folder_name,'/q_real.csv']));
fid_q_desired = importdata(strcat([folder_name,'/q_desired.csv']));
fid_grad_data = importdata(strcat([folder_name,'/Grad_data_record.csv']));
fid_vector =  importdata(strcat([folder_name,'/Vector_data_record.csv']));
fid_leader = importdata(strcat([folder_name,'/Leader_data_record.csv']));

total_row = size(fid_q_real,1);
num_agent = 4;
time1 = zeros(total_row-1,1);
q_real_data = zeros(total_row-1,num_agent*3);
q_desired_data = zeros(total_row-1,num_agent*2);
grad_data = zeros(total_row-1,num_agent*5);
vector_data = zeros(total_row-1,num_agent*4);
leader_data =  zeros(total_row-1,num_agent*4);
S= regexp(fid_q_real{2,1},',','split');
start_time = str2double(S{1,1});
for row=2:total_row
    S= strsplit(fid_q_real{row,1},{',','[',']','"'});
    time1(row-1,1)  = (str2double(S{1,1})-start_time)/10^9;
    for id=1:num_agent*3
        q_real_data(row-1,id) = str2double(S{1,3+id-1});
    end
end
for row=2:12835
    S= strsplit(fid_q_desired{row,1},{',','[',']','"'});
    for id=1:num_agent*2
        q_desired_data(row-1,id) = str2double(S{1,3+id-1});
    end
end
for row=2:total_row-3
    S= strsplit(fid_grad_data{row,1},{',','[',']','"'});
    for id=1:num_agent*5
        grad_data(row-1,id) = str2double(S{1,3+id-1});
    end
end

for row=2:total_row-3
    S0 = strsplit(fid_grad_data{row,1},{',','[',']','"'});
    S1= strsplit(fid_leader{row,1},{',','[',']','"'});
    S2 = strsplit(fid_vector{row,1},{',','[',']','"'});
    for id=1:num_agent*5
        grad_data(row-1,id) = str2double(S{1,3+id-1});
        
    end
end

for rId=3:4
    plot(time1(1:end_flag),q_real_data(1:end_flag,rId*3-2),'LineWidth',1.5);
    hold on;
end

for rId=3:4
    plot(time1(1:end_flag),q_desired_data(1:end_flag,rId*2-1),'--r');
end
legend("x3 real","x4 real","x desired");
xlabel("time/sec");
ylabel("xpos/m");
title("x pos of formation");
xlim([0 128]);
figure(2)
for rId=3:4
    plot(time1(1:end_flag),q_real_data(1:end_flag,rId*3-1),'LineWidth',1.5);
    hold on;
end

for rId=3:4
    plot(time1(1:end_flag),q_desired_data(1:end_flag,rId*2),'--r');
end
legend("y3 real","y4 real","y desired");
xlabel("time/sec");
ylabel("ypos/m");
xlim([0 128]);
ylim([-1.5 1.5]);
title("y pos of formation");
figure(3)
plot(time1(1:end),grad_data(1:end,3*5),'LineWidth',1.2);
xlabel("time/sec");
ylabel("Cost");
legend("Cost Value");
title("Cost Function for agent 2");


