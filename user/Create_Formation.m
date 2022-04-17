
function Create_Formation(xlimit,ylimit,agent_num)
close all;
figure;
fid= fopen('test.txt', 'w');    % clear the data in txt
set(gcf,'WindowButtonDownFcn',{@ButttonDownFcn,xlimit,ylimit,agent_num});
xlim(xlimit);   % Without this, axis resizing can slow things down
ylim(ylimit);
% while 1
%     data = fscanf(fid_2, ' %f ', [agent_num*2,1]);
%     [row,col] = size(data);
%     if row == agent_num*2
%         disp("yes");
%         break
%     end
% end
end

function ButttonDownFcn(src,event,xlimit,ylimit,agent_num)
pt = get(gca,'CurrentPoint');
x = pt(1,1);
y = pt(1,2);
fid= fopen('test.txt', 'a');   
fprintf(fid,'%.3f %.3f ',x,y);
scatter(x,y);
xlim(xlimit);   % Without this, axis resizing can slow things down
ylim(ylimit);
hold on;
end