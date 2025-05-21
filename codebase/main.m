clear all
figure

global RRT_ans;
RRT_time=[];
RRT_ftimes=0;
RRT_f=0;
num_ob=40;
xlim=[-40,40];   %[-40,40]
ylim=[-20,20];   %[-20,20]
start=[20,-15]; %[-20,15];  
goal=[-25,15]; %[25,15];
ob_size=[3,3];
M=3;

%% 初始化

%障碍物信息
obs=[
    %5
    5,0,4.0; 5,5,4.0;
    
    -7,5,4.0;  -10,10,4.0;  -13,-5,4.0;             %7,5,4.0;  10,10,4.0;  13,-5,4.0; 
    %4
    30,0,4.0;  25,10,4.0;   45,-10,4.0;  20,-5,4.0;         %30,0,4.0;  25,10,4.0;   45,-10,4.0;  20,-5,4.0;
    
    %4
   -20,-10,4.0;  -15,-5,4.0;   -11,-5,4.0; -1,-10,4.0;

    %3
    -6,10,4.0; -18,17,4.0;-10,15,4.0;
    
    %4
    -33,-3,4.0; -35,-5,4.0;  
    -35,13,4.0; -40,5,4.0 ];

% xx = [];
% yy = [];
% x1 = [10 -7.171 -15.3171];
% y1 = [10 2.20 -11.79730];
% 
% x = -13:0.25:10;
% y = spline(x1,y1,x);                                                       
% % plot(x,y,'linewidth',1.5);
% 
% x1=[x1,zeros(1,M-length(x1))];                                             
% y1=[y1,zeros(1,M-length(y1))];                                             
% 
% xx=[xx;x1];                                                                
% yy=[yy;y1]; 

%rrt*的起点、终点、轨迹
%障碍物 颜色+形状
for i=1:num_ob      %循环次数
    while true
        tmp_x = rand*(xlim(2)-xlim(1))+xlim(1);          
        tmp_y = rand*(ylim(2)-ylim(1))+ylim(1);
        tmp_s = rand*(ob_size(2)-ob_size(1))+ob_size(1);
        if sqrt( (tmp_x-start(1,1) )^2+ ( tmp_y-start(1,2) )^2) <= 4
            continue;
        elseif sqrt( (tmp_x-goal(1,1))^2 + (tmp_y-goal(1,2))^2 ) <= 4
            continue;
        else
            obs(i,1) = tmp_x;
            obs(i,2) = tmp_y;
            obs(i,3) = tmp_s;
%规则圆
        rectangle('Position',[obs(i,1)-obs(i,3) obs(i,2)-obs(i,3) obs(i,3)*1.75 obs(i,3)*1.5],'Curvature',[1 1],'FaceColor',[0 0.6 0.6]);    %障碍物
%不规则物体
% while true
%     if rem(i,3)==0
%     draw_x = [obs(i,1)+obs(i,3)*rand,obs(i,1)+obs(i,3)*rand,obs(i,1)-obs(i,3)*rand,obs(i,1)-obs(i,3)*rand];
%     draw_y = [obs(i,2)+obs(i,3)*rand,obs(i,2)-obs(i,3)*rand,obs(i,2)-obs(i,3)*rand,obs(i,2)+obs(i,3)*rand];
%     patch(draw_x,draw_y,'black');
%     elseif rem(i,3)==1
%     draw_x = [obs(i,1),obs(i,1)+obs(i,3)*rand,obs(i,1)+obs(i,3)*rand,obs(i,1)-obs(i,3)*rand,obs(i,1)-obs(i,3)*rand];
%     draw_y = [obs(i,2),obs(i,2)+obs(i,3)*rand,obs(i,2)-obs(i,3)*rand,obs(i,2)-obs(i,3)*rand,obs(i,2)+obs(i,3)*rand];
%     patch(draw_x,draw_y,'black');
%     else
%     draw_x = [obs(i,1),obs(i,1)+obs(i,2)*rand,obs(i,1)+obs(i,3)*rand,obs(i,1)+obs(i,3)*rand,obs(i,1)-obs(i,3)*rand,obs(i,1)-obs(i,3)*rand];
%     draw_y = [obs(i,2),obs(i,2),obs(i,2)+obs(i,3)*rand,obs(i,2)-obs(i,3)*rand,obs(i,2)-obs(i,3)*rand,obs(i,2)+obs(i,3)*rand];
%     patch(draw_x,draw_y,'black');  
        end
    break;
    end
end

%动态避障
% for i=1:num_ob 
%     x_move = obs(i,1):0.01:5; % 函数变量取值范围
%     y_move = 5*sin(x_move)+5; % 函数
%     % plot(x,y);
%     hold on
%     h = plot(x_move,y_move,'or', 'MarkerSize', 16);% 调节markersize后的数值调节圆的大小
%     for ii = -10:0.01:5
%     data = 5*sin(ii)+5;
%     axis([-40 40 -20 20]);
%     set(h,'Xdata',ii,'Ydata',data);
%     pause(0.1); % 调整速度
%     end
% end


%起点、终点
hold on
    plot(start(1),start(2),'rh','MarkerSize',16,'MarkerFaceColor','y');    
    plot(goal(1),goal(2),'g^','MarkerSize',16,'MarkerFaceColor','c');
param.threshold = 0.001;
param.maxNodes = 2000;      % 迭代次数
param.step_size = 0.3;       % 机器人每次行进步数
param.neighbourhood = 0.5;   % 寻找子节点的距离
param.random_seed = randi(101,1)-1;          %返回一个在0到100内的标量
param.obstacles=obs;
hold off
hold on
RRT_path = PlanPathRRTstar(param,start',goal',RRT_ftimes);
hold on
% APF_path= path_plan(start',goal',obs);  % 人工势场  计算并绘制出路径
pso(start,goal,obs);
legend('starting point','target point');
set(gca,'XLim',[-23,33]);    %坐标轴删除  
set(gca,'Ylim',[-20,20]);

