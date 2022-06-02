%% Funcion principal para calcular y mover Robot con CTRAJ
function [mth_f,q] = calc_RTB(motorSVC,p_rbt,eslabones,matrices,q_prev,id_m,time,movePX_RTB)
    % Evalua la primera matriz para determinar tipo de trajectoria Codo Arriba o Codo Abajo
    mth = matrices(:,:,1);
    q = invKinPxC(mth,eslabones);
    % Se inicia asumiendo codo abajo sin desfase de PI
    rta = 1;
    % Si la articulacion uno tiene un desfase se cambia a codo abajo con desfase de PI
    if abs(q(rta,1)-q_prev(1))>pi/4
        rta = 3;
    end
    % Si la articulacion 3 tiene un desfase se cambia a codo arriba sumando 1
    if abs(q(rta,3)+q_prev(3)) < 0.1
        rta = rta+1;
    end
    % Ciclo iterativo para cada matriz de posicion de CTRAJ
    for iteration=2:length(matrices)
        % Calculo de cinematica invera de cada matriz
        mth = matrices(:,:,iteration);
        q = invKinPxC(mth,eslabones);
        % Verificacion de la orientacion de la herramienta
        if abs(q(rta,4)-q_prev(4)) > pi/4
            if q(rta,4)> 0
                q(rta,4) = q(rta,4)-pi;
            else
                q(rta,4) = q(rta,4)+pi;
            end
        end
        % se establece q para realizar el movimiento
        q = q(rta,:);
        % Calculo de cinematica directa con q obtenidos
        mth_f = p_rbt.fkine(q);
        % Si la matriz original y la obtenida tienen desfases menores a .1 se ejecuta el movimiento
        if abs(mth_f(1:3,4)-mth(1:3,4))<0.1
            % Mover el robot
            move_RTB(motorSVC,id_m,q,time,movePX_RTB)
            % Graficar el robot
            pause(0.2);
            p_rbt.plot(q,'notiles','noname');
        else
            % No se puede calcular correctamente se devuelve a posicion inicial
            disp('Fuera de rangos articulares')
            q = q_prev;
            mth_f = p_rbt.fkine(q);
            break;
        end
    end
end
