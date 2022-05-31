close all
clear
clc
eslabones = [14.5, 10.7, 10.7, 9.0]; % Longitudes eslabones en cm
feed_x = 1.5 ; % cm
feed_y = 1.5 ; % cm
feed_z = 1.5 ; % cm
feed_pitch = 0.5 ; % rad
mov = [0 0 0 0]' ;
id = 1;
change = 0 ;
dir = 0;

name = ["trax","tray","traz","rot"];
feeds = [feed_x, feed_y, feed_z, 0]';
p_rbt = init_RTB(eslabones);

q = deg2rad([20 40 60 40]);

movePX_RTB = 0;

if movePX_RTB
    rosinit;
    %Creacion del servicio para controlar Dynamixel
    motorSVC = rossvcclient('dynamixel_workbench/dynamixel_command');
else
    motorSVC = 'test';
end

%Posicionar robot en home
for i = 1:length(q)
    value = round(map_range(q(i),-5*pi/6,5*pi/6,0,1023));
    goal_pos(motorSVC,i,value,0.2,movePX_RTB)
end

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
                roshutdown;
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

        mth = matrices(:,:,1);
        q_prev = q;
        q = invKinPxC(mth,eslabones);
        rta = 1;
        if abs(q(rta,1)-q_prev(1))>pi/4
            rta = 3;
        end
        if abs(q(rta,3)+q_prev(3)) < 0.1
            rta = rta+1;
        end

        for iteration=2:5
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
                for i = 1:length(q)
                    value = round(map_range(q(i),-5*pi/6,5*pi/6,0,1023))
                    goal_pos(motorSVC,i,value,0.2,movePX_RTB)
                end
                pause(0.2);
                p_rbt.plot(q,'notiles','noname');
            else
               disp('Fuera de rangos articulares')
               q = q_prev;
               break;
            end
        end
        %p_rbt.plot(q,'notiles','noname');
        pos_final = mth_f(1:3,4)
        change = 0; dir = 0; mov = [0,0,0,0]';
    elseif change & id==4
        if q(4)+dir*feed_pitch<5*pi/6 & q(4)+dir*feed_pitch>-5*pi/6
            q(4) = q(4)+dir*feed_pitch
            p_rbt.plot(q,'notiles','noname');
            dir = 0; change = 0; mov = [0,0,0,0]';
            %mover el robot
            value = round(map_range(q(4),-5*pi/6,5*pi/6,0,1023))
            goal_pos(motorSVC,4,value,0.2,movePX_RTB)
        else
            disp('Fuera de rangos articulares');
        end
    end 
end
%%
function output = map_range(value,fromLow,fromHigh,toLow,toHigh)
    narginchk(5,5)
    nargoutchk(0,1)
    output = (value - fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow) + toLow;
end

function PhantomX = init_RTB(l)
    % Definicion del robot RTB
    L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
    L(2) = Link('revolute','alpha',0,   'a',l(2),'d',0,   'offset',pi/2,'qlim',[-3*pi/4 3*pi/4]);
    L(3) = Link('revolute','alpha',0,   'a',l(3),'d',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
    L(4) = Link('revolute','alpha',0,   'a',0,   'd',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
    PhantomX = SerialLink(L,'name','Px');
    % roty(pi/2)*rotz(-pi/2)
    PhantomX.tool = [0 0 1 l(4); -1 0 0 0; 0 -1 0 0; 0 0 0 1];
end

function goal_pos(service,id,value,time,movePX_RTB)
    if value < 1023 & value > 0         
        if movePX_RTB
            msg_svc = rosmessage(service);
            msg_svc.AddrName = "Goal_Position";
            msg_svc.Id = id;
            msg_svc.Value = value;
            call(service,msg_svc);
            pause(time);
        else
            disp("Comando no habilitado")
        end
    else
        disp("Fuera de rangos articulares");
    end
end