function [path]=PathSmooth(pathRow,map)
    path.findPath=1;
    path.pos(1).x=pathRow.pos(1).x;
    path.pos(1).y=pathRow.pos(1).y;
    path.pos=[path.pos,path.pos];
    for index_path=2:length(pathRow.pos)
        if CollisionCheck([path.pos(end-1).x,path.pos(end-1).y], ... 
                [pathRow.pos(index_path).x,pathRow.pos(index_path).y],map)
            path.pos(end)=pathRow.pos(index_path);
        else
            path.pos=[path.pos,pathRow.pos(index_path)];
        end
    end
    path.cost=0;
    for index_path=2:length(path.pos)
        path.cost=path.cost+sqrt((path.pos(index_path).x-path.pos(index_path-1).x)^2 ...
        + (path.pos(index_path).y-path.pos(index_path-1).y)^2);
    end
end