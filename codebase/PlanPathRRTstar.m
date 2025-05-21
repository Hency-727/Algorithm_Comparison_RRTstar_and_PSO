function result = PlanPathRRTstar(param, p_start, p_goal,ftime)

field1 = 'p';
field2 = 'iPrev';  %  借助该参数，可将整个路径搜索出来。
field3 = 'cost';
field4 = 'goalReached';

rng(param.random_seed);   % 用指定的 randomseed 初始化随机数生成器
tic;       % tic开始计时，常与toc配合使用，toc停止计时  （算法运行的时间计时）
start();   % 执行路径规划功能函数

    function start()      
        % s = struct(field1,value1,...,fieldN,valueN) 创建一个包含多个字段的结构体数组
        rrt(1) = struct(field1, p_start, field2, 0, field3, 0, field4, 0);  
        N = param.maxNodes; % 迭代次数 iterations
        j = 1;

%         while endcondition>param.threshold %&& j<=N    
%       每走一次循环，j++，一共循环N次
        while j<=N   
            % 1）随机采样一个点
            sample_node = getSample();  
%             plot(sample_node(1), sample_node(2), '.g');
%             text(sample_node(1), sample_node(2), strcat('random',num2str(j)))
            % 2）找到现有节点中，距离该采样点最近的点
            nearest_node_ind = findNearest(rrt, sample_node); 
            second_nearest_node_ind = findsecondNearest(rrt, sample_node); 
%             plot(rrt(nearest_node_ind).p(1), rrt(nearest_node_ind).p(2), '.g');
%             text(rrt(nearest_node_ind).p(1), rrt(nearest_node_ind).p(2), strcat('nearest', num2str(j)));
            % 3）沿最近点到采样点方向，按照步长前进，得到新的节点
            new_node = steering(rrt(nearest_node_ind).p, sample_node);
            second_new_node = steering(rrt(second_nearest_node_ind).p, sample_node);
            if (isObstacleFree(new_node,second_new_node)==1)        % 4.1） 新节点的障碍物检测
%                 plot(new_node(1), new_node(2), '.g');
%                 text(new_node(1), new_node(2)+3, strcat('steered: new node', num2str(j)))
                % 获取新节点附近可到达的节点索引  neighbourhood
                neighbors_ind = getNeighbors(rrt, new_node);
                if(~isempty(neighbors_ind))        % 4.2） 判断附近可达到的节点 存在与否 （找该新节点的父节点）
                    % 存在— 从根节点为新子节点选择成本最低的父节点  【难理解1】
                    parent_node_ind = chooseParent(rrt, neighbors_ind, nearest_node_ind,new_node);
%                     plot(rrt(parent_node_ind).p(1), rrt(parent_node_ind).p(2), '.g');
%                     text(rrt(parent_node_ind).p(1), rrt(parent_node_ind).p(2)+3, strcat('parent', num2str(j)));
                else   
                    % 不存在— 选取距离该采样点最近的点为父节点
                    parent_node_ind = nearest_node_ind;
                end
                % 5）将新节点插入到 rrt搜索树中
                rrt = insertNode(rrt, parent_node_ind, new_node);
                if (~isempty(neighbors_ind))    % 存在可到达节点时，进行重连操作  优化路径  【难理解2】
                    rrt = reWire(rrt, neighbors_ind, parent_node_ind, length(rrt));
                end
                % 满足距离阈值条件，记该节点为到达目标 （但依旧会持续嵌套搜索）
                if norm(new_node-p_goal) <= param.threshold
                    rrt = setReachGoal(rrt);
                end
            end
            j = j + 1;
        end
        setPath(rrt);   % 上述while结束后，绘制寻找的路径
%         text1 = strcat('Total number of generated nodes:', num2str(j-1))
%         text1 = strcat('Total number of nodes in tree:', length(rrt))
    end

%% 一系列功能函数（start中调用）

    % 在rrt树中标记该节点为“到达目标点”（第四个参数）  ok
    function rrt=setReachGoal(rrt)
        rrt(end).goalReached = 1;
    end
    
    % 绘制出 rrt中各个节点的关系，并标识出最终得到的路径  ok
    function setPath(rrt)    
        %hold on
        % 1）绘制rrt树 ：各个父节点与子节点之间的连线  【父节点→子节点】
        for i = 1: length(rrt)-1
            p1 = rrt(i).p;   % 遍历rrt中各个节点
            rob.x = p1(1); rob.y=p1(2);
            plot(rob.x,rob.y,'.g')  % 绘制各个节点                                         %点的颜色
            child_ind = find([rrt.iPrev]==i);  % 寻找rrt中父节点索引为i的一堆子节点
            for j = 1: length(child_ind)       % 遍历上述一堆子节点
                p2 = rrt(child_ind(j)).p;      % 找到并绘制 上述父节点到子节点 的连线
%                 pause(0.01);            % 程序暂停一会再继续运行 -- 体现出路径搜索的过程
                plot([p1(1),p2(1)], [p1(2),p2(2)], 'g', 'LineWidth', 1);                 %树状分支的颜色
            end
        end 
        % 2）找到最短路径，并绘制
        [cost,i] = getFinalResult(rrt);
        result.cost = cost;
        result.rrt = [rrt.p];  % result输出的节点坐标
        xx=[];yy=[];
        %绘制最终得到的最短路径
        while i ~= 0
            p11 = rrt(i).p;
            %plot(p11(1),p11(2),'r', 'Marker','.', 'MarkerSize', 10);
            i = rrt(i).iPrev;
            
            xx=[xx,p11(1)];yy=[yy,p11(2)];

            if i ~= 0
                p22 = rrt(i).p;  % 依次画出该节点，其父节点，其父节点的父节点（从终点画到起点）     
                plot([p11(1),p22(1)],[p11(2),p22(2)],'-b','Linewidth',1.5);
                plot(p22(1),p22(2),'b', 'Marker', '.', 'MarkerSize', 10);
%                 pause(0.02);            % 程序暂停一会再继续运行 -- 体现出路径搜索的过程
                plot([p11(1),p22(1)],[p11(2),p22(2)], 'b', 'LineWidth', 3);
            end 
        end
        plot(xx,yy,'-b','linewidth',1.5);
        result.time_taken = toc;     % 返回程序总的运行时间
        ft=ftime;
    end


    % 找最终路径 （在setPath中调用）           ok
    % 参    数：rrt树
    % 返 回 值：代价cost 最接近终点的最优节点索引
    % 寻找方法：寻找“到达目标点”的节点中，代价最小的节点
    % 由该代价最小的节点，rrt树中的第二个参数iPrev，可再找到其父节点，由此可得到一条代价最小的路径
    function [value,min_node_ind] = getFinalResult(rrt)
        % 找到所有“到目标点”的节点索引   rrt第四个参数
        goal_ind = find([rrt.goalReached]==1);
        if ~(isempty(goal_ind))   % 判断是否达到目标点（goal是否存在）
            %disp('Goal has been reached!');
            rrt_goal = rrt(goal_ind);  % 储存“到目标点”节点
            value = min([rrt_goal.cost]);  % 取得上述节点中 cost最小值
            min_node_ind = find([rrt.cost]==value);  % 找到 cost最小值节点的索引
            if length(min_node_ind)>1    % 如果cost最小值对应的索引有多个，取第一个即可
                min_node_ind = min_node_ind(1);
            end
        else   % goal_ind不存在
           % disp('Goal has not been reached!');
            ftime=ftime+1;
            % 计算rrt中各个节点到目标点的距离，求出最小的一个节点，充当“终点” 
            for i =1:length(rrt)
                norm_rrt(i) = norm(p_goal-rrt(i).p);
            end
            [value,min_node_ind]= min(norm_rrt);   % 求取其中距离目标点最近的节点
            value = rrt(min_node_ind).cost;        % 以求得的该点信息作为返回值
        end
    end
    

    % 新节点的障碍物判断函数 （针对此程序的矩形障碍物判断）
    % 参    数：新节点的坐标信息
    % 返 回 值： 0-有障碍物   1-无障碍物
    % 判断方法：节点坐标是否在矩形障碍物四个端点的约束内（可更改该函数，实现障碍物的不同变换）
    % param.obstacles =[130,70,20,60; 70,135,60,20;]; 
    function free=isObstacleFree(node_free,second_node_free)
        free = 1;
        p1=node_free';
        p2=second_node_free';
        for i = 1: size(param.obstacles(:,1), 1)

            center = param.obstacles(i,1:2);
            radio = param.obstacles(i, 3);
            if(radio==0)continue;end
            % 点在圆内
            if norm(center - p1) < radio || norm(center - p2) < radio
                free = 0;
                return;
            end
            % 直线 Ax + By + C = 0;  (y1 - y2) x + (x2 - x1) y + x1y2 - y1x2 = 0;
            if p1(1) == p2(1)
                A = 1;
                B = 0;
                C = -p1(1);
            elseif p1(2) == p2(2)
                A = 0;
                B = 1;
                C = -p1(2);
            else
                A = p1(2) - p2(2);
                B = p2(1) - p1(1);
                C = p1(1) * p2(2) - p1(2) * p2(1);
            end
            dist1 = (A * center(1) + B * center(2) + C) ^ 2;
            dist2 = (A * A + B * B) * radio * radio;
            if dist1 > dist2
%                 disp(dist2)
%                 disp(dist1)
                continue;
            end
            angle1 = (center(1) - p1(1)) * (p2(1) - p1(1)) + (center(2) - p1(2)) * (p2(2) - p1(2));
            angle2 = (center(1) - p2(1)) * (p1(1) - p2(1)) + (center(2) - p2(2)) * (p1(2) - p2(2));
            if angle1 > 0 && angle2 > 0
                free = 0;
                return;
            end
        end
    end
    
    % 【step_size】    ok
    % 沿最近点到采样点方向，按照机器人步长前进，得到新的节点
    % 参    数： 离采样点最近的节点  采样点
    % 返 回 值： 新节点的坐标
    function new_node=steering(nearest_node, random_node)
       dist = norm(random_node-nearest_node);   % 两点距离
       ratio_distance = param.step_size/dist;   
       % 计算新节点的xy坐标  （这里也可使用其他方式进行计算）
       x = (1-ratio_distance).* nearest_node(1)+ratio_distance .* random_node(1);
       y = (1-ratio_distance).* nearest_node(2)+ratio_distance .* random_node(2);
       
       new_node = [x;y];
    end
    
    % 范围内的可到达的节点重新连接，以得到更优的路径 
    % 参    数： rrt树   临近节点索引   父节点索引   新节点的索引--rrt节点总数（前面传入）
    % 返 回 值： 更新 新节点附近的临近节点的 父节点 【建立了更短的路径联系】
    function rrt = reWire(rrt, neighbors, parent, new)
        for i=1:length(neighbors)      % 遍历每个可达到的临近节点
            cost = rrt(new).cost + norm(rrt(neighbors(i)).p - rrt(new).p);   % 求以新节点作为父节点时，临近节点的代价
            
            if (cost<rrt(neighbors(i)).cost)  % 如果 上述新代价 小于 临近节点现有代价
%                 if norm(rrt(new).p-rrt(neighbors(i)).p)<param.step_size
% %                     plot(rrt(neighbors(i)).p(1), rrt(neighbors(i)).p(2), '.b');
%                     rrt(neighbors(i)).p = steering(rrt(new).p, rrt(neighbors(i)).p);
%                 end
%                 plot(rrt(neighbors(i)).p(1), rrt(neighbors(i)).p(2), '.m');
                rrt(neighbors(i)).iPrev = new;    % 将 新节点作为 该临近节点的 父节点
                rrt(neighbors(i)).cost = cost;
            end
        end
    end
    
    % 将新的节点插入rrt树末尾      ok
    % 参    数： rrt  父节点索引  新节点坐标
    % 返 回 值： 更新rrt
    % 其中第三个参数 cost = 父节点代价+新节点到父节点的代价
    function rrt = insertNode(rrt, parent, new_node)
        rrt(end+1) = struct(field1, new_node, field2, parent, field3, rrt(parent).cost + norm(rrt(parent).p-new_node), field4, 0);
    end
    
    % 从根节点为新子节点选择成本最低的父节点    ok  父节点？
    % 参    数： rrt  临近的节点  最近的节点  新节点
    % 返 回 值： 父节点索引
    function parent = chooseParent(rrt, neighbors, nearest, new_node)
        min_cost = getCostFromRoot(rrt, nearest, new_node);   % 求以最近的节点为父节点时，新节点的代价
        parent = nearest;            %  暂取最近的节点，作为父节点
        for i=1:length(neighbors)    % neighbors - 下一步可到达的节点
            cost = getCostFromRoot(rrt, neighbors(i), new_node);   % 求以可到达的节点为父节点时，新节点的代价
            if (cost<min_cost)    % 最终取代价最小的那个节点，作为父节点
               min_cost = cost;
               parent = neighbors(i);
            end
        end
    end
    
    % 父节点的cost + 子节点到父节点的距离代价    ok
    % 参    数： rrt  父节点索引  子节点
    % 返 回 值： 子节点的代价
    function cost = getCostFromRoot(rrt, parent, child_node)       
       cost =  rrt(parent).cost + norm(child_node - rrt(parent).p);
    end

    % 【neighbourhood】         ok
    % 获取指定节点周围可以到达的节点索引
    % 参    数： rrt  node -new_node（上面调用时传的参数）
    % 返 回 值： 保存可到达节点的索引的neighbors
    function neighbors = getNeighbors(rrt, node)
        neighbors = [];
        for i = 1:length(rrt)    % 遍历rrt树中所有的节点
            dist = norm(rrt(i).p-node);      % 计算rrt中各个节点到node的距离代价
            if (dist<=param.neighbourhood)   % 找到 距离代价＜单次行进步长的节点索引
               neighbors = [neighbors i];    % 使用 neighbors 将节点索引保存
            end
        end        
    end
    
    % 在坐标图上生成随机的采样点       ok
    function node = getSample()
        x = 0;
        y = 0;
        a = -40;
        b = 40;
        c=20;
        d=-20;
        node = [x;y];
        node(1) = (b-a) * rand(1) + a;
        node(2) = (c-d) * rand(1) + d;  
    end
    
    % 寻找rrt树中距离采样点最近的节点   ok
    function indx = findNearest(rrt, n)
        mindist = norm(rrt(1).p - n);
        indx = 1;
        for i = 2:length(rrt)
            dist = norm(rrt(i).p - n);
            if (dist<mindist)
               mindist = dist;
               indx = i;
            end
        end
    end 

    % 寻找rrt树中距离采样点次最近的节点   ok
    function second_indx = findsecondNearest(rrt, n)
        mindist = norm(rrt(1).p - n);
        indx = 1;
        for i = 2:length(rrt)
            dist = norm(rrt(i).p - n);
            if (dist<mindist)
               mindist = dist;
               indx = i;

            end
        end
        disp(indx)
        second_mindist = norm(rrt(1).p - n);
        second_indx = 1;
         for j = 2:length(rrt)
            dist = norm(rrt(j).p - n);
            if dist<second_mindist&&j~=indx&&dist~=mindist
                    second_mindist = dist;
                    second_indx = j;
            end
         end      
         %disp(second_indx)
    end    
end