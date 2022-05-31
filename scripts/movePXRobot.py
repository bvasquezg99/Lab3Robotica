"""
Allows to use the service dynamixel_command 
"""

from turtle import position
import rospy
import time
import termios, os, sys
from dynamixel_workbench_msgs.srv import DynamixelCommand
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb
from spatialmath import *
from spatialmath.base import *
import numpy as np

__author__ = "F Gonzalez, S Realpe, JM Fajardo"
__credits__ = ["Felipe Gonzalez", "Sebastian Realpe", "Jose Manuel Fajardo", "Robotis"]
__email__ = "fegonzalezro@unal.edu.co"
__status__ = "Test"

TERMIOS = termios
l = np.array([14.5, 10.7, 10.7, 9])
feed_x = 5 # mm
feed_y = 5 # mm
feed_z = 5 # mm
feed_pitch = 0.5 # rad
dir = 0
name = ['trax','tray','traz','rot']
feeds = [feed_x, feed_y, feed_z, feed_pitch]
home = np.array([512, 512 ,512 ,512])
pos = [512, 512 ,512 ,512]
id = 1
change = 0

def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def callback(data):
    rospy.loginfo(data.position)
    #if data.position[4]>-2:
    #    print('gripper abierto')
    #else:
    #    print('gripper cerrado')
    #pos = data.position

def listener():
    rospy.init_node('joint_states', anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, callback)
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

#Se hace uso del código brindado para la detección de las teclas oprimidas
def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c

def q_inv(Tt):
    np.set_printoptions(suppress=True)
    T = Tt.A
    Tw = T-(l[3]*T[0:4,2]).reshape(4,1)
    q1 = np.arctan2(Tw[1,3],Tw[0,3])
    # Solucion 2R
    h = Tw[2,3] - l[0]
    r = np.sqrt(Tw[0,3]**2 + Tw[1,3]**2)
    # Codo abajo
    the3 = np.arccos((r**2+h**2-l[1]**2-l[2]**2)/(2*l[1]*l[2]))
    the2 = np.arctan2(h,r) - np.arctan2(l[2]*np.sin(the3),l[1]+l[2]*np.cos(the3))
    q2d = -(np.pi/2-the2)
    q3d = the3
    # Codo arriba
    the2 = np.arctan2(h,r) + np.arctan2(l[2]*np.sin(the3),l[1]+l[2]*np.cos(the3))
    q2u = -(np.pi/2-the2)
    q3u = -the3
    # Solucion q4
    Rp = (rotz(q1).T).dot(T[0:3,0:3])
    pitch = np.arctan2(Rp[2,0],Rp[0,0])
    q4d = pitch - q2d - q3d
    q4u = pitch - q2u - q3u
    
    qinv = np.empty((2,4))
    qinv[:] =np.NaN
    qinv[0,:] = np.array([q1, q2u, q3u, q4u])
    qinv[1,:] = np.array([q1, q2d, q3d, q4d])
    return qinv

def map_range(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


if __name__ == '__main__':
    #try:
        # Goal_Position (0,1023)
        # Torque_Limit (0,1023)
        # map(0,1023,-150,150)
        #jointCommand('', 1, 'Torque_Limit', 600, 0)
        #jointCommand('', 2, 'Torque_Limit', 500, 0)
        #jointCommand('', 3, 'Torque_Limit', 400, 0)
        #jointCommand('', 4, 'Torque_Limit', 400, 0)
        ids = [5]
        #for i in ids:
            #jointCommand('', i, 'Goal_Position',home[i-1], 0.2)
        qlims = np.array([[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4],[-3*np.pi/4, 3*np.pi/4]])
        robot = rtb.DHRobot(
            [rtb.RevoluteDH(alpha=np.pi/2, d=l[0], qlim=qlims[0,:]),
            rtb.RevoluteDH(a=l[1], offset=np.pi/2, qlim=qlims[0,:]),
            rtb.RevoluteDH(a=l[2], qlim=qlims[0,:]),
            rtb.RevoluteDH(qlim=qlims[0,:])],
            name="Px_DH_std")
        robot.tool = transl(l[3],0,0).dot(troty(np.pi/2).dot(trotz(-np.pi/2)))
        print(robot)
        q = map_range(np.array([60, -75, 15, 10]),0,180,0,np.pi);
        robot.plot(q)
        # listener()
        while(1):
            key = getkey()
            if key == b'w':
                if id != 4:
                    id =  id + 1
                else:
                    id = 1
            if key == b's':
                if id != 1:
                    id = id - 1
                else:
                    id = 4
            if key == b'd':
                dir = 1
                change = 1
            if key == b'a':
                dir = -1
                change = 1
            posHome = map_range(home,0,1023,-2.6,2.6)
            Thome = robot.fkine(posHome)
            print('Eje de movimiento: ' + name[id-1])
            print(Thome)
            print('Posicion: ' + str(posHome))
            if change == 1:
                pos[id-1] = pos[id-1] + dir*feeds[id-1]
                if id == 4:
                    rot = map_range(pos[id-1],qlims[0,0],qlims[0,1],0,1023)
                    print('rot: ' + rot + ' id: ' + id)
                    #jointCommand('', id, 'Goal_Position',rot, 0.2)
                else: 
                    T = Thome
                    T[4,1] = pos[1]
                    T[4,2] = pos[2]
                    T[4,3] = pos[3]
                    print(T)
                    matrices = rtb.tools.trajectory.ctraj(Thome,T,5)
                    for mt in matrices:
                        q_i = q_inv(mt)
                        q_i = q_i[0,:]
                        print(q_i)
                        q_imap = map_range(q_i,-2.6,2.6,0,1023)
                        print(q_imap)
                        #jointCommand('', id, 'Goal_Position',pos[id-1], 0.2)
                        time.sleep(0.1)
                dir = 0
                change = 0

    #except rospy.ROSInterruptException:
     #   pass