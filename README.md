# Lab 3 - Robótica
Laboratorio 3 Robótica - Universidad Nacional de Colombia - 2022-1
# Integrantes:

Brian Alejandro Vásquez González  
William Arturo Sierra Díaz  

# MATLAB + Toolbox
 ### Modelo cinemática inversa Phantom X
 Basados en el ejemplo desarrollado en el laboratorio, y siguiendo la metodología analizada en clase, se construyó un modelo de cinemática inversa del manipulador en MATLAB. 
 
Lo primero, entonces, es definir el manipulador, para lo cual, se recupera la definición del manipulador, realizada en el laboratorio pasado. Se usa el comando SerialLink, se especifican distancias y parámetros DH y se crea la matriz de la herramienta con respecto al último marco de referencia. 
 
```matlab
l = [4.7, 10.65, 10.65, 6.97]

L_1(1) = Link('revolute','alpha',pi/2,'a',0,'d',l(1),'offset',0, 'qlim',[-3*pi/4 3*pi/4]);
L_1(2) = Link('revolute','alpha',0,   'a',l(2),'d',0,'offset',pi/2, 'qlim',[-3*pi/4 3*pi/4]);
L_1(3) = Link('revolute','alpha',0,   'a',l(3),'d',0,'offset',0, 'qlim',[-3*pi/4 3*pi/4]);
L_1(4) = Link('revolute','alpha',0,'a',l(4),'d',0,'offset',0, 'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L_1,'name','Px');
PhantomX.tool = [0 0 1 l(4); -1 0 0 0; 0 -1 0 0; 0 0 0 1];
```
[![PosIni.png](https://i.postimg.cc/2816B9rM/PosIni.png)](https://postimg.cc/CzVY3NZs)

Con el robot ya construido, el cual para verificar se muestra graficado en la imagen anterior, se empieza con el proceso de cinemática inversa. De acuerdo a lo sugerido en clase, lo primero es el desacople, así se separa el problema, ignorando incialmente el problema de orientación en la muñeca. Así, lo primero es obtener la MTH de la herramienta con respecto a la base. Luego se hace el desacople, reduciendo solo al problema de posición.
```matlab
qt = deg2rad([20, -30, 45, 30]);
Tt = PhantomX.fkine(qt);
%%
% Desacople
T = Tt
Posw = T(1:3,4) - l(4)*T(1:3,3) %Posición final herramienta
```
Para la primera articulación, su valor articular, solo está determinado por la posición final de la herramienta, por lo tanto:
```matlab
q1 = atan2(Posw(2),Posw(1));
rad2deg(q1)
```
Ahora, para determinar los valores de q2 y de q3, el problema se ve reducido a la solución de un mecanismo 2R, con dos posibles soluciones, codo arriba y codo abajo. Para el 2R, su posición final viene en términos los valores de posición final de la muñeca.
```matlab
h = Posw(3) - l(1);
r = sqrt(Posw(1)^2 + Posw(2)^2);
```
Ahora, con esas dos distancias determinadas, se plantean las soluciones para ambos casos. Codo arriba y codo abajo, en donde el valor articular dependera de un angulo theta_3 y theta_2, que están en terminos de las distnacias r yh y de las longitudes de eslabon.
```matlab
% Codo abajo
the3 = acos((r^2+h^2-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
the2 = atan2(h,r) - atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));

q2d = -(pi/2-the2);
q3d = the3;

% Codo arriba
the2 = atan2(h,r) + atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));
q2u = -(pi/2-the2);
q3u = -the3;

disp(rad2deg([q2d q3d; q2u q3u]))
```
Ahora para el valor de q4, se conocen los valores de q1, q2 y q3, por lo que se puede calcular la MTH de la base a la  muñeca, y teniendo la MTH de la base a la herramienta, se puede despejar para hallar la matriz de rotación de 3 a la herramienta, y así q4. Esto también se hace para codo arriba y codo abajo.
```matlab
% Codo arriba
T03d = PhantomX.A([1 2 3],[q1 q2d q3d])
R_3Td = (T03d(1:3,1:3))'*T(1:3,1:3)
q4d = atan2(R_3Td(1,1),-R_3Td(2,1));
rad2deg(q4d)

% Codo abajo
T03u = PhantomX.A([1 2 3],[q1 q2u q3u]);
R_3Tu = (T03u(1:3,1:3))'*T(1:3,1:3);
q4u = atan2(R_3Tu(1,1),-R_3Tu(2,1));
rad2deg(q4u)
```
Y finalmente, se obtiene todos los valores articulares, obteniendo el resultado de la cinemática inversa.
```matlab
qinv(1,1:4) = [q1 q2u q3u q4u]
qinv(2,1:4) = [q1 q2d q3d q4d];
disp(rad2deg(qinv))
```
Para verificar mediante el toolbox de Peter Corke, se utilizo el comando ikine, que calcula la cinematica inversa por métodos numéricos. Con esto se comprueba una respuesta que coincide con la solución calculada en codo arriba.
```matlab
Mask = [1 1 1 0 0 0]
qinv_t=PhantomX.ikine(Tt,qinv(1,1:4),Mask)
disp(rad2deg(qinv_t))
%Sol ikine:  -20.0000   15.0000  -45.0000   75.0000
```
### Espacio de trabajo Phantom X

En la siguiente imagen, se puede observar el espacio de trabajo para  un PhantomX, con una vista planar:

[![Workspace.png](https://i.postimg.cc/BvHyrdCz/Workspace.png)](https://postimg.cc/8FktWX9d)

Tomada de: Toquica, H (2018). PhantomX Pincher Specifications. Disponible en: https://www.researchgate.net/publication/322222351_PhantomX_Pincher_Specifications

### Métodos para el cálculo de la cinemática inversa
Tras realizar una consulta sobre las funciones disponibles en el toolbox de Peter Corke, se encontraron los siguientes métodos que se pueden usar para hallar la cinemática inversa de algún manipulador:



1. ikine6s: permite calcular la cinemática inversa de un manipulador de 6 grados de libertad.
2. ikine: permite calcular la cinemática de cualquier manipulador, usando un metodo numérico iterativo. 
3. ikunc: permite el cálculo de la cinemática inversa mediante optimización.
4. ikcon: similar a la anterior, realiza el cálculo nmediante optimización, pero teniendo en cuenta los límites articulares.
5. ikine_sym: realiza el cálculo de la cinemática inversa utilizando el meotodo análitica y variables simbólicas.
6. ikinem: Hace el cálculo numérico de la cinemática inversa mediante minimización.
7. ikine3: permite calcular la cinemática inversa de un manipulador de 3 ejes sin muñeca esférica.

# Análisis

1. Sabiendo que el robot Phantom X posee 4 GDL, de los cuales 3 corresponden a posición, el GDL restante
proporciona una medida independiente para un  ́angulo de orientación (asuma orientación en  ́angulos fijos).
¿De qué  ́angulo de orientación se trata?

-Como se menciona en la pregunta, el robot puede realizar movimiento en los tres ejes cooordenados. En cuanto a la orientación, el análisis muestra que el grado de libertad restante corresponde a un movimiento alrededor del eje y, lo que en ángulo fijo, es llamado pitch.

2. ¿Cuántas soluciones posibles existen para la cinemática inversa del manipulador Phantom X ?

-Siempre que se intente realizar la solución para una pose con el manipulador, se tendrán dos alternativas de solución, las llamadas soluciones "codo arriba" y "codo abajo".

3. Consulte en qué consiste el espacio diestro de un manipulador.

-Se conoce como espacio diestro al conjunto de puntos dentro del espacio de trabajo general, que el manipulador puede alcanzar con todas las orientaciones posibles del efector final.

# Aplicaciones

Para estas las apliaciones pick and place y movimiento en el esapcio de la tarea se elegio MatLab pese a su facilidad para el manejo del toolbox de Peter Corke, se realizaron algunos ensayos previos con python pero no fueron del todo exitosos por lo cual se prefirio continuar con el desarrollo del laboratorio en MatLab.

# Aplicación de Pick and Place
El script completo de este ejercicio lo puede ver [dando clic aquí](matlab/lab_3_p1.m)

[Ver video Aplicación Pick and Place](https://youtu.be/sInh1Wx4ufA)

# Aplicación de movimiento en el espacio de la tarea

El script completo de este ejercicio lo puede ver [dando clic aquí](matlab/lab_3_p2.m)

Para comenzar se realiza la limpieza del area de trabajo, despues de esto se procede a la configuracion de las variables de longitudes de eslabon, paso que va a tener cada movimiento en sus respectivos ejes y variavles auxiliares del proceso

```matlab
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
```
Luego se procede a inicializar el robot en el toolbox de Peter Corke y crear el servicio para el movimiento de los motores Dynamixel
```matlab
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
```
Luego con los parámetros establecidos se posiciona el robot en home
```matlab
move_RTB(motorSVC,id_m,q,time_m,movePX_RTB);
```
Posteriormente en el programa principal se realiza la deteccion de la tecla presionada y se ajustan las variables auxiliares de proceso para que reflejen el cambio deseado
```matlab
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
```
Luego de esto se graficá la posición de la siguiente forma:
```matlab
% Establecer direccion y eje de movimiento
mov(id) = dir;
% Calcular cinematica directa de la posicion actual
mth_home = p_rbt.fkine(q);
% Graficar RTB
p_rbt.plot(q,'notiles','noname');
```
A continuacion se detalla el codigo para el movimiento de los ejes del robot segun las matrices obtenida por la funcion CTRAJ
```matlab
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
```

[Ver video Aplicación de movimiento en el espacio de la tarea](https://youtu.be/qoHWaeiMhog)


# Conclusiones
