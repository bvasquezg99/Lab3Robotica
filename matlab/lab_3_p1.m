close all
clear
clc
eslabones = [14.5, 10.7, 10.7, 9.0]; % Longitudes eslabones en cm
% Ids de los motores sin gripper
id_m = [1 2 3 4];
% Inicializar RTB Toolbox
p_rbt = init_RTB(eslabones);
% Bandera para habilitar el movimiento del robot PhantomX
movePX_RTB = 0;
%set tiempo de pausa entre movimientos
time_m = 0.2;

if movePX_RTB
    % Iniciar ROS
    rosinit;
    % Creacion del servicio para controlar Dynamixel
    motorSVC = rossvcclient('dynamixel_workbench/dynamixel_command');
    for it = 1:4
        torque_limit(motorSVC,it,850,0.2,movePX_RTB)
    end
else
    motorSVC = 'test';
end

% Programa principal

mth_t1_a =      [0   1    0     0;
                1   0    0  15.5;
                0   0   -1  14.0;
                0   0    0    1];
             
mth_t1_p =      [0   1    0     0;
                1   0    0  15.5;
                0   0   -1   6.0;
                0   0    0    1];

mth_t2_p_a =    [0  -1    0     0;
                -1   0    0 -15.5;
                 0   0   -1    14;
                 0   0    0    1]; 
             
mth_t2_p =      [0  -1   0      0;
                -1   0   0  -15.5;
                 0   0  -1    4.5;
                 0   0   0     1];             

mth_t1_a_pl =   [-1   0   0 -15.5;
                  0   1   0     0;
                  0   0  -1    14;
                  0   0   0    1];
              
mth_t1_pl =     [-1   0   0 -15.5;
                  0   1   0     0;
                  0   0  -1     6;
                  0   0   0    1];
% Solucion Codo Arriba
rta = 4;
% gripper abierto
gripper_open = 512;
% gripper cerrado 666 para t2
gripper_close_2 = 666;
% gripper cerrado para t1
gripper_close_1 = 40;
% ID gripper
gripper_id = 5;
% Abrir Gripper
goal_pos(motorSVC,gripper_id,gripper_open,0.5,movePX_RTB);
% Establece posicion Home
mth_home = mth_t1_a_pl;
q = invKinPxC(mth_home,eslabones);
q = q(rta,:);
q(4) = q(4)+pi;
% Posicionar RTB
move_RTB(motorSVC,id_m,q,time_m,movePX_RTB)
% Graficar el Movimiento
p_rbt.plot(q,'notiles','noname');
hold on
ws = [-50 50];
trplot(eye(4),'rgb','arrow','length',15,'frame','0')
axis([repmat(ws,1,2) 0 60])
view([25.4 34.2]);
% set cantidad de matrices ctraj
mth_quantity = 6;

% Primer Movimiento Horizontal
q(1) = q(1)-pi/2;
move_RTB(motorSVC,id_m,q,time_m,movePX_RTB);
% Pick pieza tipo 1
mth_home = p_rbt.fkine(q);
mth_obj = mth_t1_p;
matrices = ctraj(mth_home,mth_obj,mth_quantity);
% Calcular y mover Robot a las poses ctraj
calc_RTB(motorSVC,p_rbt,eslabones,matrices,q,id_m,time_m,movePX_RTB);
% Cerrar Gripper
goal_pos(motorSVC,gripper_id,gripper_close_1,time_m,movePX_RTB);

% Movimiento vertical pieza tipo 1
mth_home = p_rbt.fkine(q);
mth_obj = mth_t1_a;
matrices = ctraj(mth_home,mth_obj,mth_quantity);
% Calcular y mover Robot a las poses ctraj
calc_RTB(motorSVC,p_rbt,eslabones,matrices,q,id_m,time_m,movePX_RTB);

% Movimiento horizontal
q(1) = q(1)+pi/2;
move_RTB(motorSVC,id_m,q,time_m,movePX_RTB);

% Movimiento para dejar Pieza tipo 1 en el centro
mth_home = p_rbt.fkine(q);
mth_obj = mth_t1_pl;
matrices = ctraj(mth_home,mth_obj,mth_quantity);
% Calcular y mover Robot a las poses ctraj
calc_RTB(motorSVC,p_rbt,eslabones,matrices,q,id_m,time_m,movePX_RTB);
% Abrir Gripper
goal_pos(motorSVC,gripper_id,gripper_open,time_m,movePX_RTB);

% Movimiento para ir a home vertical
mth_home = p_rbt.fkine(q);
mth_obj = mth_t1_a_pl;
matrices = ctraj(mth_home,mth_obj,mth_quantity);
% Calcular y mover Robot a las poses ctraj
calc_RTB(motorSVC,p_rbt,eslabones,matrices,q,id_m,time_m,movePX_RTB);

% Movimiento horizontal para tomar pieza tipo 2
q(1) = q(1)+pi/2;
move_RTB(motorSVC,id_m,q,time_m,movePX_RTB);

% Movimiento vertical para tomar pieza tipo 2
mth_home = p_rbt.fkine(q);
mth_obj = mth_t2_p;
matrices = ctraj(mth_home,mth_obj,mth_quantity);
% Calcular y mover Robot a las poses ctraj
calc_RTB(motorSVC,p_rbt,eslabones,matrices,q,id_m,time_m,movePX_RTB);
% Cerrar Gripper para tomar pieza tipo 2
goal_pos(motorSVC,gripper_id,gripper_close_2,time_m,movePX_RTB);

% Movimiento vertical para llevar pieza tipo 2
mth_home = p_rbt.fkine(q);
mth_obj = mth_t2_p_a;
matrices = ctraj(mth_home,mth_obj,mth_quantity);
% Calcular y mover Robot a las poses ctraj
calc_RTB(motorSVC,p_rbt,eslabones,matrices,q,id_m,time_m,movePX_RTB);

% Movimiento horizontal para llevar pieza tipo 2
q(1) = q(1)-pi/2;
move_RTB(motorSVC,id_m,q,time_m,movePX_RTB);

% Movimiento vertical para dejar pieza tipo 2
mth_home = p_rbt.fkine(q);
mth_obj = mth_t1_pl;
matrices = ctraj(mth_home,mth_obj,mth_quantity);
% Calcular y mover Robot a las poses ctraj
calc_RTB(motorSVC,p_rbt,eslabones,matrices,q,id_m,time_m,movePX_RTB);
% Abrir gripper
goal_pos(motorSVC,gripper_id,gripper_open,time_m,movePX_RTB);

% Regresar a home vertical
mth_home = p_rbt.fkine(q);
mth_obj = mth_t1_a_pl;
matrices = ctraj(mth_home,mth_obj,mth_quantity);

% Calcular y mover Robot a las poses ctraj
calc_RTB(motorSVC,p_rbt,eslabones,matrices,q,id_m,time_m,movePX_RTB);

if movePX_RTB
    rosshutdown
end

