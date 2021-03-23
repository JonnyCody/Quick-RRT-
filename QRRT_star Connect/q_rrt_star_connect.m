clear all;close all;clc;
%% 参数初始化
    % 为了减少函数的入口参数，用结构体合并多个变量
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
maxIterations=5000; % 最大迭代次数

path.findPath = 0;
path.pos = [];
path.cost=[];
path.ratioToLast=[];

%% 树初始化：T是树，v是节点
% treeA初始化
treeA.v(1).x=point.init(1);     % 起始节点加入tree中
treeA.v(1).y=point.init(2);
treeA.v(1).xPre=point.init(1);  % 起点的父节点是本身
treeA.v(1).yPre=point.init(2);
treeA.v(1).totalCost=0;
treeA.v(1).indexPre=0;
treeA.countV=1;                           %treeA中节点的数量

% treeB初始化
treeB.v(1).x=point.goal(1);     % 起始节点加入tree中
treeB.v(1).y=point.goal(2);
treeB.v(1).xPre=point.goal(1);  % 起点的父节点是本身
treeB.v(1).yPre=point.goal(2);
treeB.v(1).totalCost=0;
treeB.v(1).indexPre=0;
treeB.countV=1;                           %treeB中节点的数量

%% 导入地图
mapRgb = imread('squares.bmp');
map = rgb2gray(mapRgb);
% 检测起点终点
CheckParameters(point.init,point.goal,map);
figure(1);
imshow(map)
% title('Quick RRT* Connect(Rapidly-Exploring Random Trees)');
[yLength,xLength]=size(map);        % 地图的高和宽
hold on;
plot(point.init(1), point.init(2), 'mo', 'MarkerSize',10, 'MarkerFaceColor','m');   % 绘制起点和目标点
plot(point.goal(1), point.goal(2), 'go', 'MarkerSize',10, 'MarkerFaceColor','g');

tic;
tic;
%% 构建树
for iter=1:maxIterations
    % Step 1: 在地图中随机采样一个点vRand (Sample)
    vRand=[unifrnd(0,xLength),unifrnd(0,yLength)];
    % Step 2: 根据随机点，找出treeA的vNew
    [vNearestA,vNewA,tmpParentA]=FindNewPoint(vRand,point,treeA);
    
    % 检查节点是否是collision-free
    if ~CollisionCheck(vNearestA,vNewA,map)     %有障碍物
        continue;
    end
    
    %  Step 3: 将treeA拓展到vNewA
    [treeA]=Extend(tmpParentA,vNewA,point,treeA,map);
    
    % Step 4: 将treeB向vNewA拓展，直到遇到障碍物或者接近vNewA
    [vNewB,treeB]=Connect(vNewA,point,treeB,map);
    %Step 5:检查两棵树是否接近
    disNewBToNewA = sqrt((vNewB(1) - vNewA(1))^2 + (vNewB(2) - vNewA(2))^2);
    if disNewBToNewA < point.goalThreshold &&CollisionCheck(vNewB,vNewA,map)==true    % 
        tmpPath=FillPath(path,point,treeA,treeB);
        tmpPath=PathSmooth(tmpPath,map);
        if ~path.findPath
            path.findPath=1;
            tFindPath=toc;
            path.cost=[path.cost,tmpPath.cost];
            path.ratioToLast=[path.ratioToLast,0];
        end
        
        if path.cost(end)>tmpPath.cost
           path.cost=[path.cost,tmpPath.cost];
%            path.ratioToLast=[path.ratioToLast,path.cost(end)/path.cost(end-1)];
           path.pos=tmpPath.pos;
        end
    end
    
    % 交换两棵树
    [treeA,treeB]=Swap(treeA,treeB);
end
tEnd=toc;

%%  显示路径和树

    %   显示树
 for i=length(treeA.v):-1:1
     plot([treeA.v(i).x; treeA.v(i).xPre;], [treeA.v(i).y; treeA.v(i).yPre], 'g', 'Linewidth', 1);
 end
 
  for i=length(treeB.v):-1:1
     plot([treeB.v(i).x; treeB.v(i).xPre;], [treeB.v(i).y; treeB.v(i).yPre], 'b', 'Linewidth', 1);
 end
 

    %   显示路径
for j = 2:length(path.pos)
    plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'r', 'Linewidth', 6);
end

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

function [a,b]=Swap(a,b)
    c=a;
    a=b;
    b=c;
end
