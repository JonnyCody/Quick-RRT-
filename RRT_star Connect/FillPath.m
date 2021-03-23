function [path]=FillPath(point,treeA,treeB)
    path.findPath=1;
    j=2;
    pathA(1).x=treeA.v(treeA.countV).x;
    pathA(1).y=treeA.v(treeA.countV).y;
    indexPathA=treeA.v(treeA.countV).indexPre;
    while 1
        pathA(j).x=treeA.v(indexPathA).x;
        pathA(j).y=treeA.v(indexPathA).y;
        indexPathA=treeA.v(indexPathA).indexPre;
        if indexPathA==0
            break;
        end
        j=j+1;
    end
    
    j=2;
    pathB(1).x=treeB.v(treeB.countV).x;
    pathB(1).y=treeB.v(treeB.countV).y;
    indexPathB=treeB.v(treeB.countV).indexPre;
    while 1
        pathB(j).x=treeB.v(indexPathB).x;
        pathB(j).y=treeB.v(indexPathB).y;
        indexPathB=treeB.v(indexPathB).indexPre;
        if indexPathB==0
            break;
        end
        j=j+1;
    end
    
    if pathA(end).x==point.init(1)
        pathA=fliplr(pathA);
        path.pos=[pathA,pathB];
    else
        pathB=fliplr(pathB);
        path.pos=[pathB,pathA];
    end
    
    path.cost=0;
    for index_path=2:length(path.pos)
        path.cost=path.cost+sqrt((path.pos(index_path).x-path.pos(index_path-1).x)^2 ...
        + (path.pos(index_path).y-path.pos(index_path-1).y)^2);
    end
    
end