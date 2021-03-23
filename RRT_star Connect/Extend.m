function [tree]=Extend(tmpParent,vNew,point,tree,map)
    tmpCost=point.deltaStep+tree.v(tmpParent).totalCost;
    % Step 4: 在以vNew为圆心,半径为R的圆内搜索节点 (rNear)
    
    disToNewList = [];    % 每次循环要把队列清空
    nearIndexList = [];
    for index_near = 1:tree.countV
        disToNew = sqrt((vNew(1) - tree.v(index_near).x)^2 + (vNew(2) - tree.v(index_near).y)^2);
        if(disToNew < point.rNear)    % 满足条件:欧氏距离小于rNear
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
    tree.countV = tree.countV+1;    %最新节点的索引
    
    tree.v(tree.countV).x = vNew(1);
    tree.v(tree.countV).y = vNew(2);
    tree.v(tree.countV).xPre = tree.v(tmpParent).x;
    tree.v(tree.countV).yPre = tree.v(tmpParent).y;
    tree.v(tree.countV).totalCost = tmpCost;
    tree.v(tree.countV).indexPre = tmpParent;     %其父节点xNear的index
   
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
                tree.v(nearIndexList(rewire_index)).indexPre = tree.countV;       % vNew的索引
            end
        end
    end
end