function [vNewB,treeB]=Connect(vNewA,point,treeB,map)
    while 1
        [vNearestB,vNewB,tmpParentB]=FindNewPoint(vNewA,point,treeB);
        disNewBToNewA = sqrt((vNewB(1) - vNewA(1))^2 + (vNewB(2) - vNewA(2))^2);
        if ~CollisionCheck(vNearestB,vNewB,map)||disNewBToNewA < point.goalThreshold
            % vNearestB和vNewB有存在障碍物，所以要放弃这个vNewB，防止vNewB和vNewA没有障碍，导致产生错误路径
            if ~CollisionCheck(vNearestB,vNewB,map)
                vNewB=[treeB.v(treeB.countV).x,treeB.v(treeB.countV).y];
                break;
            end
            % 将vNewB加入树treeB中
            [treeB]=Extend(tmpParentB,vNewB,point,treeB,map);
            break;   %有障碍物或者两棵树接近
        end
        [treeB]=Extend(tmpParentB,vNewB,point,treeB,map);
    end
end