% Limpiar espacio de trabajo
close all
clear
clc
 % Longitudes eslabones en cm
eslabones = [14.5, 10.7, 10.7, 9.0];
% Establecer avance de cada eje
feed_x = 1.5 ; feed_y = 1.5 ; feed_z = 1.5 ; % cm
feed_pitch = 0.2 ; % rad
% Variables auxiliares de proceso
mov = [0 0 0 0]';
id = 1;
change = 0 ;
dir = 0;
% Variables de movimiento del robot
time_m = 0.2;
id_m = [1 2 3 4];
name = ["trax","tray","traz","rot"];
feeds = [feed_x, feed_y, feed_z, 0]';
% Inicializar RTB Toolbox
p_rbt = init_RTB(eslabones);
% Establecer posicion inicial
q = deg2rad([0 40 60 40]);
% Bandera para habilitar el movimiento del robot PhantomX 
movePX_RTB = 0;
% Inicializar Ros y establecer torques limite
if movePX_RTB
    rosinit;
    %Creacion del servicio para controlar Dynamixel
    motorSVC = rossvcclient('dynamixel_workbench/dynamixel_command');
    torque_limit(motorSVC,1,800,0.2,movePX_RTB)
    for it = 2:4
        torque_limit(motorSVC,it,650,time_m,movePX_RTB)
    end
else
    motorSVC = 'test';
end

%Posicionar robot en home
move_RTB(motorSVC,id_m,q,time_m,movePX_RTB);

% Programa principal
while 1
    disp("Eje de movimiento: " + name(id))
    txt = "Ingrese comando (Presione 'f' para salir): ";
    key = input(txt,'s');
    switch key
        % Cambio de eje de movimiento
        case 'w'
            if id ~= 4
                id =  id + 1;
            else
                id = 1;
            end
        case 's'
            if id ~= 1
                id = id - 1;
            else
                id = 4;
            end
        % Cambio de dirección de movimiento
        case 'd'
            dir = 1;
            change = 1;
        case 'a'
            dir = -1;
            change = 1;
        % Terminar Programa
        case 'f'
            if movePX_RTB
                disp("Finalizando ROS");
                rosshutdown;
            end
            disp("Programa terminado")
            break;
    end
    % Establecer direccion y eje de movimiento
    mov(id) = dir;
    % Calcular cinematica directa de la posicion actual
    mth_home = p_rbt.fkine(q);
    % Graficar RTB
    p_rbt.plot(q,'notiles','noname');
    hold on
    ws = [-50 50];
    trplot(eye(4),'rgb','arrow','length',15,'frame','0')
    axis([repmat(ws,1,2) 0 60])
    view([25.4 34.2]);
    % Logica para movimientos en ejes x, y, z
    if change & id<4
        % Establecer matriz Objetivo
        mth_obj = mth_home;
        mth_obj(:,4) = mth_obj(:,4)+mov.*feeds
        % Calcular matrices de movimiento
        matrices = ctraj(mth_home,mth_obj,5);
        % Guardar posicion anterior
        q_prev = q;
        % Calcular y mover Robot a las poses ctraj
        [mth_f,q] = calc_RTB(motorSVC,p_rbt,eslabones,matrices,q_prev,id_m,time_m,movePX_RTB);
        % Graficar posicion actual
        p_rbt.plot(q,'notiles','noname');
        % Mostrar posicion final en X, Y y Z
        pos_final = mth_f(1:3,4)
        % Reset variables aux
        change = 0; dir = 0; mov = [0,0,0,0]';
    % Logica para movimiento en eje pitch
    elseif change & id==4
        % Guardar posicion anterior
        q_prev = q;
        % Verificar limites articulares
        if q(4)+dir*feed_pitch<5*pi/6 & q(4)+dir*feed_pitch>-5*pi/6
            % Establecer movimiento
            q(4) = q(4)+dir*feed_pitch;
            % Graficar
            p_rbt.plot(q,'notiles','noname');
            % Reset variables aux
            dir = 0; change = 0; mov = [0,0,0,0]';
            % Mover el robot
            move_RTB(motorSVC,id_m,q,time_m,movePX_RTB);
        else
            % Volver a la posicion anterior
            q = q_prev;
            disp('Fuera de rangos articulares');
        end
    end 
end
