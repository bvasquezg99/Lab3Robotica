function move_RTB(motorSVC,ids,q,time,movePX_RTB)
    if length(ids) == length(q)
        for i = 1:length(q)
            value = round(map_range(q(i),-5*pi/6,5*pi/6,0,1023));
            goal_pos(motorSVC,ids(i),value,time,movePX_RTB);
        end
    else    
        disp('Revise los parametros');
    end
end