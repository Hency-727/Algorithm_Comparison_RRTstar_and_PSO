clear all
figure

global RRT_ans;
global Gau_ans;
RRT_time=[];
RRT_ftimes=0;
RRT_f=0;
num_ob=20;
xlim=[-40,40];
ylim=[-20,20];
start=[-20,-15]; %[-20,15];  
goal=[25,15];%[25,15];
ob_size=[3,3];
M=3;
%% 初始化
obs=[15,10,1;
     5,5,1.5;
    10,5,1.3;
    10,10,2;
    15,0,2;
    5,0,1.5;
    20,-5,1.5;
    15,-10,2;
    10,-5,2];
xx=[];yy=[];
x1 = [10 -7.171 -15.3171];
y1 = [10 2.20 -11.79730];

x=-13:0.25:10;
y=spline(x1,y1,x);
% plot(x,y,'linewidth',1.5);
x1=[x1,zeros(1,M-length(x1))];
y1=[y1,zeros(1,M-length(y1))];
xx=[xx;x1];
yy=[yy;y1];


hold on
% x2=[15,4.791,-3.86];
% y2=[10,6.62,0.01];
% x=-13:0.25:15;
% y=spline(x2,y2,x);
% % plot(x,y,'linewidth',1.5);
% x2=[x2,zeros(1,M-length(x2))];
% y2=[y2,zeros(1,M-length(y2))];
% xx=[xx;x2];
% yy=[yy;y2];
% 
% 
% hold on
% x3=[5,-4.791,-9.463];
% y3=[5,-2.62,-8.4882];
% x=-13:0.25:5;
% y=spline(x3,y3,x);
% % plot(x,y,'linewidth',1.5);
% x3=[x3,zeros(1,M-length(x3))];
% y3=[y3,zeros(1,M-length(y3))];
% xx=[xx;x3];
% yy=[yy;y3];
% 
% hold on
% x4=[10,2.88,-6.4];
% y4=[5,-4.67,-13.361];
% x=-13:0.25:10;
% y=spline(x4,y4,x);
% % plot(x,y,'linewidth',1.5);
% x4=[x4,zeros(1,M-length(x4))];
% y4=[y4,zeros(1,M-length(y4))];
% xx=[xx;x4];
% yy=[yy;y4];
% 
% 
% hold on
% x5=[5,-4.332,-8.84];
% y5=[0,-5.0, -6.8];
% x=-13:0.25:5;
% y=spline(x5,y5,x);
% % plot(x,y,'linewidth',1.5);
% x5=[x5,zeros(1,M-length(x5))];
% y5=[y5,zeros(1,M-length(y5))];
% xx=[xx;x5];
% yy=[yy;y5];
% 
% hold on
% x6=[15,3.52,-7.299];
% y6=[0,-4.16,-9.76];
% x=-13:0.25:15;
% y=spline(x6,y6,x);
% % plot(x,y,'linewidth',1.5);
% x6=[x6,zeros(1,M-length(x6))];
% y6=[y6,zeros(1,M-length(y6))];
% xx=[xx;x6];
% yy=[yy;y6];
% 
% hold on
% x7=[10,0,-10.6];
% y7=[-5,-10.16,-7.979];
% x=-13:0.25:10;
% y=spline(x7,y7,x);
% % plot(x,y,'linewidth',1.5);
% x7=[x7,zeros(1,M-length(x7))];
% y7=[y7,zeros(1,M-length(y7))];
% xx=[xx;x7];
% yy=[yy;y7];
% 
% hold on
% x8=[15,1.73697,-11.2];
% y8=[-10,-14.7245,-11.8];
% x=-13:0.25:15;
% y=spline(x8,y8,x);
% % plot(x,y,'linewidth',1.5);
% x8=[x8,zeros(1,M-length(x8))];
% y8=[y8,zeros(1,M-length(y8))];
% xx=[xx;x8];
% yy=[yy;y8];
% 
% hold on
% x9=[20,6.06,-11.7];
% y9=[-5,-11.41,-10.78];
% x=-13:0.25:20;
% y=spline(x9,y9,x);
% % plot(x,y,'linewidth',1.5);
% x9=[x9,zeros(1,M-length(x9))];
% y9=[y9,zeros(1,M-length(y9))];
% xx=[xx;x9];
% yy=[yy;y9];

for i=1:10
    
end
for i=1:num_ob
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
            rectangle('Position',[obs(i,1)-obs(i,3) obs(i,2)-obs(i,3) obs(i,3)*2 obs(i,3)*2],'Curvature',[1 1],'FaceColor',[0.5 0.7 0.8]);%[36/255,41/255,47/255]);

            break;
        end
     end
end


% hold on %画图前加hold on
% % % RRTstar初始化参数
% param.threshold = 1;
% param.maxNodes = 50;      % 迭代次数
% param.step_size = 2;       % 机器人每次行进步数
% param.neighbourhood = 2;   % 寻找子节点的距离
% param.random_seed = randi(101,1)-1;


hold on
% for j=1:size(xx,2)
%     hold on
    plot(start(1),start(2),'bs','MarkerSize',12,'MarkerFaceColor','y');
    plot(goal(1),goal(2),'kp','MarkerSize',16,'MarkerFaceColor','g');
%     for i=1:size(obs,1)
%           obs(i,1)=xx(i,j);
%           obs(i,2)=yy(i,j);
%           rectangle('Position',[obs(i,1)-obs(i,3) obs(i,2)-obs(i,3) obs(i,3)*2 obs(i,3)*2],'Curvature',[1 1],'FaceColor',[0.5 0.7 0.8]);%[36/255,41/255,47/255]);
%     end
%     hold on
% 
param.threshold = 1;
param.maxNodes = 1200;      % 迭代次数
param.step_size = 2;       % 机器人每次行进步数
param.neighbourhood = 2;   % 寻找子节点的距离
param.random_seed = randi(101,1)-1;
param.obstacles=obs;
RRT_path = PlanPathRRTstar(param,start',goal',RRT_ftimes)
hold on
pso(start,goal,obs);
set(gca,'XLim',[-23,33]);
set(gca,'Ylim',[-20,20]);
legend('start','goal','RRT*','PSO');

%     break;
% end




% hold on 
% grid on
% RRT_f=RRT_f+faile;
% figure(2)
% for i=1:size(obs,1)
%      rectangle('Position',[obs(i,1)-obs(i,3) obs(i,2)-obs(i,3) obs(i,3)*2 obs(i,3)*2],'Curvature',[1 1],'FaceColor',[0.5 0.7 0.8]);%[36/255,41/255,47/255]);
% end
% axis equal
% grid on
% hold on
% plot(start(1),start(2),'bs','MarkerSize',12,'MarkerFaceColor','y');
% plot(goal(1),goal(2),'kp','MarkerSize',16,'MarkerFaceColor','g');
% 
% hold on
%legend('start','goal','original path','optimized path');
%legend('start','goal','RRT*','Gaussian')


% figure(3)
% axis equal
% grid on
% hold on 
% plot(start(1),start(2),'bs','MarkerSize',12,'MarkerFaceColor','y');
% plot(goal(1),goal(2),'kp','MarkerSize',16,'MarkerFaceColor','g');
% % 

    function circle(x,y,r,c)
        ang=0:0.01:2*pi; 
        xp=r*cos(ang);
        yp=r*sin(ang);
        plot(x+xp,y+yp, c);
    end