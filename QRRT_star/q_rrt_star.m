clear all;close all;clc;
%% 参数初始化
% point.init=[592,436];%frame
% point.goal=[1000,436];%frame
point.init=[307,436];%squares
point.goal=[875,436];%squares
% point.init=[100,700];%maze
% point.goal=[1000,100];%maze
point.goalThreshold=30;   % 设置目标点阈值
point.deltaStep=30;           % 拓展步长
point.rNear=80;           % rewire半径范围
point.depth=1;
maxIterations=3000; % 最大迭代次数
path.findPath = 0;
path.cost=[];
path.ratioToLast=[];
path.pos = [];

%% 树初始化：T是树，v是节点
tree.v(1).x=point.init(1);     % 起始节点加入tree中
tree.v(1).y=point.init(2);
tree.v(1).xPre=point.init(1);  % 起点的父节点是本身
tree.v(1).yPre=point.init(2);
tree.v(1).totalCost=0;
tree.v(1).indexPre=0;
tree.countV=1;                           %tree中节点的数量

%% 导入地图
mapRgb = imread('squares.bmp');
map = rgb2gray(mapRgb);
% 检测起点终点
CheckParameters([point.init(1),point.init(2)],[point.goal(1),point.goal(2)],map);
figure(1);
imshow(map)
% title('Quick RRT* (Rapidly-Exploring Random Trees)');
[yLength,xLength]=size(map);        % 地图的高和宽
hold on
plot(point.init(1), point.init(2), 'mo', 'MarkerSize',10, 'MarkerFaceColor','m');   % 绘制起点和目标点
plot(point.goal(1), point.goal(2), 'go', 'MarkerSize',10, 'MarkerFaceColor','g');


%% 构建树
tic;
tic;
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
    vNearest=[tree.v(minIndex).x, tree.v(minIndex).y];
    % 临时父节点的索引
    tmpParent=minIndex;
    
    % Step 3: 扩展得到vNew节点 (Steer)
    theta = atan2((vRand(2) - vNearest(2)),(vRand(1) - vNearest(1)));
    vNew=[vNearest(1)+cos(theta)*point.deltaStep,vNearest(2)+sin(theta)*point.deltaStep];
    
    % 检查节点是否是collision-free
    if ~CollisionCheck(vNearest,vNew,map)
        continue;   %有障碍物
    end
    
    [tree]=Extend(vNew,tmpParent,point,tree,map);
    
    %Step 9:检查是否到达目标点附近 
    disToGoal = sqrt((vNew(1) - point.goal(1))^2 + (vNew(2) - point.goal(2))^2);
    if(disToGoal < point.goalThreshold && ~path.findPath)    % 找到目标点，此条件只进入一次
        tFindPath=toc;
        path.findPath = 1;
        tree.countV = tree.countV+1;    %手动将Goal加入到树中
        indexGoal = tree.countV;
        tree.v(tree.countV).x = point.goal(1);          
        tree.v(tree.countV).y = point.goal(2); 
        tree.v(tree.countV).xPre = vNew(1);     
        tree.v(tree.countV).yPre = vNew(2);
        tree.v(tree.countV).totalCost = tree.v(tree.countV - 1).totalCost + disToGoal;
        tree.v(tree.countV).indexPre = tree.countV - 1;     %其父节点x_near的index
        
        path.cost=[path.cost,tree.v(indexGoal).totalCost];
        path.ratioToLast=[path.ratioToLast,0];
    end
    
    if path.findPath == 1&&path.cost(end)~=tree.v(indexGoal).totalCost
        path.cost=[path.cost,tree.v(indexGoal).totalCost];
%         path.ratioToLast=[path.ratioToLast,path.cost(end)/path.cost(end-1)];
%         if path.ratioToLast(end)>0.999
%             break;
%         end
    end
end
disp (iter);
tEnd=toc;

%%  显示树和路径
j = 2;
path.pos(1).x = point.goal(1);
path.pos(1).y = point.goal(2);
indexPath = tree.v(indexGoal).indexPre;
while 1
    path.pos(j).x = tree.v(indexPath).x;
    path.pos(j).y = tree.v(indexPath).y;
    indexPath = tree.v(indexPath).indexPre;    % 沿终点回溯到起点
    if indexPath == 0
        break;
    end
    j=j+1;
end

for index_tree=length(tree.v):-1:1
    plot([tree.v(index_tree).xPre, tree.v(index_tree).x], [tree.v(index_tree).yPre, tree.v(index_tree).y], 'b', 'Linewidth', 1);
%     plot(tree.v(index_tree).x, tree.v(index_tree).y, 'ko', 'MarkerSize', 4, 'MarkerFaceColor','k');
end

for j = 2:length(path.pos)
    plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'r', 'Linewidth', 6);
end

%%  检查参数
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
