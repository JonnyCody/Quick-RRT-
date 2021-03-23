function [vNearest,vNew,tmpParent]=FindNew(target,point,tree)
    % Step 2: 遍历树，从树中找到最近邻近点vNear (Near)
    minDist=sqrt((target(1) - tree.v(1).x)^2 + (target(2) - tree.v(1).y)^2);
    minIndex=1;
    for i=2:size(tree.v,2)          % tree.v按行向量存储，size(tree.v,2)获得节点总数
        tmpDist=sqrt((target(1) - tree.v(i).x)^2 + (target(2) - tree.v(i).y)^2);   %两节点间距离
        if tmpDist<minDist
            minDist=tmpDist;
            minIndex=i;
        end
    end
    
    % 找到当前树中离vRand最近的节点
    vNearest=[tree.v(minIndex).x, tree.v(minIndex).y];
    % 临时父节点的索引
    tmpParent=minIndex;
    % 临时累计代价
    tmpCost = point.deltaStep + tree.v(minIndex).totalCost;
    
    % Step 3: 扩展得到vNew节点 (Steer)
    theta = atan2((target(2) - vNearest(2)),(target(1) - vNearest(1)));
    vNew=[vNearest(1)+cos(theta)*point.deltaStep ,vNearest(2)+sin(theta)*point.deltaStep ];
end