close all
clear
clc
eslabones = [14.5, 10.7, 10.7, 9.0]; % Longitudes eslabones en cm
feed_x = 1.5 ; % cm
feed_y = 1.5 ; % cm
feed_z = 1.5 ; % cm
feed_pitch = 0.2 ; % rad
mov = [0 0 0 0]';
id = 1;
change = 0 ;
dir = 0;

time_m = 0.2;
id_m = [1 2 3 4];
name = ["trax","tray","traz","rot"];
feeds = [feed_x, feed_y, feed_z, 0]';
p_rbt = init_RTB(eslabones);

q = deg2rad([0 40 60 40]);

movePX_RTB = 0;

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
        case 'd'
            dir = 1;
            change = 1;
        case 'a'
            dir = -1;
            change = 1;
        case 'f'
            if movePX_RTB
                disp("Finalizando ROS");
                rosshutdown;
            end
            disp("Programa terminado")
            break;
    end
    mov(id) = dir;
    mth_home = p_rbt.fkine(q);
    p_rbt.plot(q,'notiles','noname');
    hold on
    ws = [-50 50];
    trplot(eye(4),'rgb','arrow','length',15,'frame','0')
    axis([repmat(ws,1,2) 0 60])
    view([25.4 34.2]);
    
    if change & id<4
        mth_obj = mth_home;
        mth_obj(:,4) = mth_obj(:,4)+mov.*feeds
        matrices = ctraj(mth_home,mth_obj,5);
        q_prev = q;
        % Calcular y mover Robot a las poses ctraj
        [mth_f,q] = calc_RTB(motorSVC,p_rbt,eslabones,matrices,q_prev,id_m,time_m,movePX_RTB);
        
        p_rbt.plot(q,'notiles','noname');
        pos_final = mth_f(1:3,4)
        change = 0; dir = 0; mov = [0,0,0,0]';
    elseif change & id==4
        q_prev = q;
        if q(4)+dir*feed_pitch<5*pi/6 & q(4)+dir*feed_pitch>-5*pi/6
            q(4) = q(4)+dir*feed_pitch;
            p_rbt.plot(q,'notiles','noname');
            dir = 0; change = 0; mov = [0,0,0,0]';
            %mover el robot
            move_RTB(motorSVC,id_m,q,time_m,movePX_RTB);
        else
            q = q_prev;
            disp('Fuera de rangos articulares');
        end
    end 
end