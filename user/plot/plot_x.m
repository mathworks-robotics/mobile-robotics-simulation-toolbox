load data_log/data20220512T165818;
num_agent = length(trajx);
sample_time = 0.01;
time_x = linspace(0.01,0.01*length(trajx{1}),length(trajx{1}));
for rId=1:num_agent
   time_index = 1;
   plot(time_x,trajx{rId});
   hold on;
end
figure(2);
time_y = linspace(0.01,0.01*length(trajy{1}),length(trajy{1}));
for rId=1:num_agent
   time_index = 1;
   plot(time_y,trajy{rId});
   hold on;
end
figure(3);
time_cmdv = linspace(0.01,0.01*length(cmd_v{1}),length(cmd_v{1}));
for rId=1:num_agent
   time_index = 1;
   plot(time_cmdv,cmd_v{rId});
   hold on;
end

figure(4);
time_cmdw = linspace(0.01,0.01*length(cmd_w{1}),length(cmd_w{1}));
for rId=1:num_agent
   time_index = 1;
   plot(time_cmdw,cmd_w{rId});
   hold on;
end