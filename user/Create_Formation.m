
function Create_Formation(xlimit,ylimit,agent_num)
close all;
figure;
[~] = fopen('formation_data.txt', 'w');    % clear the data in txt
set(gcf,'WindowButtonDownFcn',{@ButttonDownFcn,xlimit,ylimit});
xlim(xlimit);   % Without this, axis resizing can slow things down
ylim(ylimit);
end

function ButttonDownFcn(~,~,xlimit,ylimit)
pt = get(gca,'CurrentPoint');
x = pt(1,1);
y = pt(1,2);
fid= fopen('formation_data.txt', 'a');   
fprintf(fid,'%.3f %.3f ',x,y);
plot(x,y,'rx','LineWidth',2,'MarkerSize',10);
xlim(xlimit);   % Without this, axis resizing can slow things down
ylim(ylimit);
hold on;
end