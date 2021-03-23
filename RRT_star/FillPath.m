function [path]=FillPath(goal,indexGoal,tree)
     j = 2;
    path.pos(1).x = goal(1);
    path.pos(1).y = goal(2);
    indexPath = tree.v(indexGoal).indexPre;
    while 1
        path.pos(j).x = tree.v(indexPath).x;
        path.pos(j).y = tree.v(indexPath).y;
        indexPath = tree.v(indexPath).indexPre;    % 沿终点回溯到起点
        if indexPath == 0
            break
        end
        j=j+1;
    end
end