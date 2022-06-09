clc;
car_num = 6;
% time = zeros(car_num,10000);
% posx = zeros(car_num,10000);
% posy = zeros(car_num,10000);
name = 'D:\浙江大学\毕业设计\中期审查\pose';
end_index  = 860;
figure(1);
for i=1:car_num
    sub_name = strcat([name,num2str(i-1)],".csv");
    rawTable = readtable(sub_name);
    x = rawTable.field_x(1:end_index);
    y = rawTable.field_y(1:end_index);
    time = rawTable.x_time(1:end_index);
    RG = ones(size(time));
    plot(x,y);
    scatter(x(end),y(end));
    scatter(x(1),y(1));
    hold on;
end
grid on;
xlabel("xpos/m");
ylabel("ypos/m");
axis([-2 3 -2 2]); 
% for i=1:car_num
%     rawTable = readtable('D:\浙江大学\毕业设计\中期审查\pose1.csv');
%     sheet_all(i) = sheet;
% end