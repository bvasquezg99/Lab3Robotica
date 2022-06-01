%% Funcion principal para calcular y mover Robot con CTRAJ
function [mth_f,q] = calc_RTB(motorSVC,p_rbt,eslabones,matrices,q_prev,id_m,time,movePX_RTB)
    mth = matrices(:,:,1);
    q = invKinPxC(mth,eslabones);
    rta = 1;
    if abs(q(rta,1)-q_prev(1))>pi/4
        rta = 3;
    end
    if abs(q(rta,3)+q_prev(3)) < 0.1
        rta = rta+1;
    end    
    for iteration=2:length(matrices)    
        mth = matrices(:,:,iteration);
        q = invKinPxC(mth,eslabones);

        if abs(q(rta,4)-q_prev(4)) > pi/4
            if q(rta,4)> 0
                q(rta,4) = q(rta,4)-pi;
            else
                q(rta,4) = q(rta,4)+pi;
            end
        end
        q = q(rta,:);
        mth_f = p_rbt.fkine(q);
        if abs(mth_f(1:3,4)-mth(1:3,4))<0.1
            %mover el robot
            move_RTB(motorSVC,id_m,q,time,movePX_RTB)
            pause(0.2);
            p_rbt.plot(q,'notiles','noname');
        else
            disp('Fuera de rangos articulares')
            q = q_prev;
            mth_f = p_rbt.fkine(q);
            break;
        end
    end
end