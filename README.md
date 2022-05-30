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

