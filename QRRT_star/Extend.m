function [tree]=Extend(vNew,tmpParent,point,tree,map)

    tmpCost = point.deltaStep + tree.v(tmpParent).totalCost;
    % Step 1: 在以vNew为圆心,半径为point.rNear的圆内搜索节点 (vNear)
    disNearToNewList = [];    % 每次循环要把队列清空
    indexNearList = [];
    for index_near = 1:tree.countV
        disToNew = sqrt((vNew(1) - tree.v(index_near).x)^2 + (vNew(2) - tree.v(index_near).y)^2);
        if(disToNew < point.rNear)    % 满足条件:欧氏距离小于point.rNear
            disNearToNewList = [disNearToNewList disToNew];     % 满足条件的所有节点到vNew的cost
            indexNearList = [indexNearList index_near];     % 满足条件的所有节点基于树T的索引
        end
    end
    
    % Step 2: 找出vNear的父节点
    indexParentList=[];
    disParToNewList=[];
   for index_near=1:length(indexNearList)
        tmpIndexPre=tree.v(indexNearList(index_near)).indexPre;
        depthNear=point.depth;
        while tmpIndexPre~=0&&depthNear>0
            if ~ismember(tmpIndexPre,indexParentList)
                indexParentList=[indexParentList tmpIndexPre];
                disParToNewList=[disParToNewList sqrt((vNew(1) - tree.v(tmpIndexPre).x)^2 + (vNew(2) - tree.v(tmpIndexPre).y)^2)];
            end
            tmpIndexPre=tree.v(tmpIndexPre).indexPre;
            depthNear=depthNear-1;
        end
    end
    % 合并vNear和父节点
    indexParAndNearList=[indexNearList indexParentList];
    disToNewList=[disNearToNewList disParToNewList];
    
    % Step 3: 选择vNew的父节点,使vNew的累计cost最小 (ChooseParent)
    for cost_index = 1:length(indexParAndNearList)    % cost_index是基于disToNewList的索引,不是整棵树的索引
        costToNew = disToNewList(cost_index) + tree.v(indexParAndNearList(cost_index)).totalCost;
        if(costToNew < tmpCost)    % tmpCost为通过minDist节点的路径的cost
            x_mincost(1) = tree.v(indexParAndNearList(cost_index)).x;     % 符合剪枝条件节点的坐标
            x_mincost(2) = tree.v(indexParAndNearList(cost_index)).y;
            if ~CollisionCheck(x_mincost,vNew,map) 
            	continue;   %有障碍物
            end
        	tmpCost = costToNew;
        	tmpParent = indexParAndNearList(cost_index);
        end
    end
    
    %Step 4: 将vNew插入树tree (AddNodeEdge)
    tree.countV = tree.countV+1;    %最新节点的索引
    tree.v(tree.countV).x = vNew(1);
    tree.v(tree.countV).y = vNew(2);
    tree.v(tree.countV).xPre = tree.v(tmpParent).x;
    tree.v(tree.countV).yPre = tree.v(tmpParent).y;
    tree.v(tree.countV).totalCost = tmpCost;
    tree.v(tree.countV).indexPre = tmpParent;     %其父节点xNear的index
   
    % Step 5: 剪枝 (rewire)
    
    % 存储vNew及其所有父节点
    indexNewAndParList=[];
    indexNewAndParList=[indexNewAndParList tree.countV];
    tmpIndexPre=tree.v(tree.countV).indexPre;
    depthNew=point.depth;
    while tmpIndexPre~=0&&depthNew>0
        indexNewAndParList=[indexNewAndParList tmpIndexPre];
        tmpIndexPre=tree.v(tmpIndexPre).indexPre;
        depthNew=depthNew-1;
    end
    
    for rewire_index = 1:length(indexNearList)
        tmpVNear=[tree.v(indexNearList(rewire_index)).x,tree.v(indexNearList(rewire_index)).y];
       for from_index = 1:length(indexNewAndParList)
           tmpVFrom=[tree.v(indexNewAndParList(from_index)).x,tree.v(indexNewAndParList(from_index)).y];
           costFromToNear=sqrt((tmpVFrom(1)-tmpVNear(1))^2 + (tmpVFrom(2)-tmpVNear(2))^2);
           if tree.v(indexNewAndParList(from_index)).totalCost+costFromToNear<tree.v(indexNearList(rewire_index)).totalCost
               if ~CollisionCheck(tmpVFrom,tmpVNear,map)
                   continue;
               end
                   tree.v(indexNearList(rewire_index)).xPre=tmpVFrom(1);
                   tree.v(indexNearList(rewire_index)).yPre=tmpVFrom(2);
                   tree.v(indexNearList(rewire_index)).indexPre=indexNewAndParList(from_index);
                   tree.v(indexNearList(rewire_index)).totalCost=tree.v(indexNewAndParList(from_index)).totalCost+costFromToNear;
                   
                   
           end
       end
    end
end