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
qt = deg2rad([45, 45, -60, 30]);
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
