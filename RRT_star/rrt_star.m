clear tree* v* x* y* path tFindPath tEnd point;
%% 参数初始化
% point.init=[592,436];%frame
% point.goal=[1000,436];%frame
% point.init=[307,436];%squares
% point.goal=[875,436];%squares
point.init=[100,700];%maze
point.goal=[1000,100];%maze
point.goalThreshold=30;   % 设置目标点阈值
delta=30;           % 拓展步长
rNear=80;           % rewire半径范围
maxIterations=5000; % 最大迭代次数
% optimalPathCost=1461;%frame
% optimalPathCost=691;%squares
% optimalPathCost=1923;%maze


%   全局变量，方便记录数据
global countTime;
countTime=0;
global tFindPath;
global path;
path.findPath = 0;
path.pos = [];
path.cost=0;

global tEnd;
tEnd=0;

%% 树初始化：T是树，v是节点
tree.v(1).x=point.init(1);     % 起始节点加入tree中
tree.v(1).y=point.init(2);
tree.v(1).xPre=point.init(1);  % 起点的父节点是本身
tree.v(1).yPre=point.init(2);
tree.v(1).totalCost=0;
tree.v(1).indexPre=0;
countV=1;                           %tree中节点的数量
% optimalPathCost=1870;%maze
% optimalPathCost=1100;%simple
optimalPathCost=1460;

%% 导入地图

mapRgb = imread('maze.bmp');
map = rgb2gray(mapRgb);
CheckParameters(point.init,point.goal,map);
figure(1);
imshow(map);
% title('Quick RRT Star (Rapidly-Exploring Random Trees)');
[yLength,xLength]=size(map);        % 地图的高和宽
hold on
plot(point.init(1), point.init(2), 'mo', 'MarkerSize',10, 'MarkerFaceColor','m');   % 绘制起点和目标点
plot(point.goal(1), point.goal(2), 'go', 'MarkerSize',10, 'MarkerFaceColor','g');

tic;
tic;
%% 构建树
for iter=1:maxIterations
    % Step 1: 在地图中随机采样一个点vRand (Sample)
    vRand=[unifrnd(0,xLength),unifrnd(0,yLength)];
    
    % Step 2: 遍历树，从树中找到最近邻近点vNear (Near)
    minDist=sqrt((vRand(1) - tree.v(1).x)^2 + (vRand(2) - tree.v(1).y)^2);
    minIndex=1;
    for i=2:size(tree.v,2)          % tree.v按行向量存储，size(tree.v,2)获得节点总数
        tmpDist=sqrt((vRand(1) - tree.v(i).x)^2 + (vRand(2) - tree.v(i).y)^2);   %两节点间距离
        if tmpDist<minDist
            minDist=tmpDist;
            minIndex=i;
        end
    end
    
    % 找到当前树中离vRand最近的节点
    vNear=[tree.v(minIndex).x, tree.v(minIndex).y];
    % 临时父节点的索引
    tmpParent=minIndex;
    % 临时累计代价
    tmpCost = delta + tree.v(minIndex).totalCost;
    
    % Step 3: 扩展得到vNew节点 (Steer)
    theta = atan2((vRand(2) - vNear(2)),(vRand(1) - vNear(1)));
    vNew=[vNear(1)+cos(theta)*delta,vNear(2)+sin(theta)*delta];
    
    % 检查节点是否是collision-free
    if ~CollisionCheck(vNear,vNew,map)
        continue;   %有障碍物
    end
    
    % Step 4: 在以vNew为圆心,半径为R的圆内搜索节点 (rNear)
    disToNewList = [];    % 每次循环要把队列清空
    nearIndexList = [];
    for index_near = 1:countV
        disToNew = sqrt((vNew(1) - tree.v(index_near).x)^2 + (vNew(2) - tree.v(index_near).y)^2);
        if(disToNew < rNear)    % 满足条件:欧氏距离小于rNear
            disToNewList = [disToNewList disToNew];     % 满足条件的所有节点到vNew的cost
            nearIndexList = [nearIndexList index_near];     % 满足条件的所有节点基于树T的索引
        end
    end
    
    % Step 5: 选择vNew的父节点,使vNew的累计cost最小 (ChooseParent)
    for cost_index = 1:length(nearIndexList)    % cost_index是基于disToNewList的索引,不是整棵树的索引
        costToNew = disToNewList(cost_index) + tree.v(nearIndexList(cost_index)).totalCost;
        if(costToNew < tmpCost)    % tmpCost为通过minDist节点的路径的cost
            x_mincost(1) = tree.v(nearIndexList(cost_index)).x;     % 符合剪枝条件节点的坐标
            x_mincost(2) = tree.v(nearIndexList(cost_index)).y;
            if ~CollisionCheck(x_mincost,vNew,map) 
            	continue;   %有障碍物
            end
        	tmpCost = costToNew;
        	tmpParent = nearIndexList(cost_index);
        end
    end
    
    %Step 6: 将vNew插入树tree (AddNodeEdge)
    countV = countV+1;    %最新节点的索引
    
    tree.v(countV).x = vNew(1);
    tree.v(countV).y = vNew(2);
    tree.v(countV).xPre = tree.v(tmpParent).x;
    tree.v(countV).yPre = tree.v(tmpParent).y;
    tree.v(countV).totalCost = tmpCost;
    tree.v(countV).indexPre = tmpParent;     %其父节点xNear的index
   
    % Step 7: 剪枝 (rewire)
    for rewire_index = 1:length(nearIndexList)
        if(nearIndexList(rewire_index) ~= tmpParent)    % 若不是之前计算的最小cost的节点
            newCost = tmpCost + disToNewList(rewire_index);    % 计算vNearby经过vNear再到起点的代价
            if(newCost < tree.v(nearIndexList(rewire_index)).totalCost)    % 需要剪枝
                x_neib(1) = tree.v(nearIndexList(rewire_index)).x;     % 符合剪枝条件节点的坐标
                x_neib(2) = tree.v(nearIndexList(rewire_index)).y;
                if ~CollisionCheck(x_neib,vNew,map) 
                    continue;   %有障碍物
                end
                tree.v(nearIndexList(rewire_index)).xPre = vNew(1);      % 对该neighbor信息进行更新
                tree.v(nearIndexList(rewire_index)).yPre = vNew(2);
                tree.v(nearIndexList(rewire_index)).totalCost = newCost;
                tree.v(nearIndexList(rewire_index)).indexPre = countV;       % vNew的索引
            end
        end
    end
    
    %Step 8:检查是否到达目标点附近
    disToGoal = sqrt((vNew(1) - point.goal(1))^2 + (vNew(2) - point.goal(2))^2);
    if(disToGoal < point.goalThreshold&&~path.findPath )    % 找到目标点，此条件只进入一次
        path.findPath = 1;
        tFindPath=toc;
        countV = countV+1;    %手动将Goal加入到树中
        indexGoal = countV;
        tree.v(countV).x = point.goal(1);
        tree.v(countV).y = point.goal(2);
        tree.v(countV).xPre = vNew(1);
        tree.v(countV).yPre = vNew(2);
        tree.v(countV).totalCost = tree.v(countV - 1).totalCost + disToGoal;
        tree.v(countV).indexPre = countV - 1;     %其父节点x_near的index
        path.cost=tree.v(indexGoal).totalCost;
    end
    
%     if path.findPath==1&&tree.v(indexGoal).totalCost<=optimalPathCost
    if path.findPath==1
        path.pos=FillPath(point.goal,indexGoal,tree);
    end
%         break;
%     end
end
tEnd=toc;
%%  显示树和路径
for index_tree=length(tree.v):-1:1
    plot([tree.v(index_tree).xPre, tree.v(index_tree).x], [tree.v(index_tree).yPre, tree.v(index_tree).y], 'b', 'Linewidth', 1);
%     plot(tree.v(index_tree).x, tree.v(index_tree).y, 'ko', 'MarkerSize', 4, 'MarkerFaceColor','k');
end

tmpPath=FillPath(point.goal,indexGoal,tree);
path.pos=tmpPath.pos;
for j = 2:length(path.pos)
    plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'r', 'Linewidth', 6);
end

%%  子函数
function CheckParameters(start,goal,map)
    if ~(start(1)>=1 && start(1)<=size(map,2) && start(2)>=1 ...
        && start(2)<=size(map,1) && map(start(2),start(1))==255)
            error('start point should be in the map');
    end
    if ~(goal(1)>=1 && goal(1)<=size(map,2) && goal(2)>=1 ...
        && goal(2)<=size(map,1) && map(goal(2),goal(1))==255)
            error('goal point should be in the map');
    end
end

