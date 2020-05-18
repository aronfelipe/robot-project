#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda","Bruno Cury", "Felipe Aron"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan
import visao_module

velocidade1 = None

bumper = None
bridge = CvBridge()

v = 0.1 #velocidade linear
w = math.pi/4 #velocidade angular
cv_image = None
media = []
centro = []
distance = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 


def Bump(dado):
    global bumper
    bumper = dado.data
    print(bumper)

def scaneou(dado):
    global distance
    distance = dado.ranges


# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        media, centro, area =  cormodule.identifica_cor(cv_image)
        centro, result_frame, result_tuples = visao_module.processa(cv_image)

        depois = time.clock()
        cv2.imshow("Camera", cv_image)
    except CvBridgeError as e:
        print('ex', e)


if __name__=="__main__":
    rospy.init_node("projeto1")

    topico_imagem = "/kamera"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    print("Usando ", topico_imagem)
    sub = rospy.Subscriber("/bumper",UInt8,Bump)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size = 1)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3)
    velocidade = Twist(Vector3(0,0,0), Vector3(0,0,0))


    try:

        while not rospy.is_shutdown():

            vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
            if len(media) != 0 and len(centro) != 0:

                for i in range(len(distance)):


                    if distance[i] > 0 and distance[i] <= 0.2:

                        if i <= 90:

                            velocidade = Twist(Vector3(-v, 0, 0), Vector3(0, 0, w))

                        elif i <= 180:

                            velocidade = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))

                        elif i <= 270:

                            velocidade = Twist(Vector3(v, 0, 0), Vector3(0, 0,w))

                        elif i <= 360:

                            velocidade = Twist(Vector3(-v, 0, 0), Vector3(0, 0, -w))

                print(velocidade)
                velocidade_saida.publish(velocidade)
                velocidade = Twist(Vector3(0,0,0), Vector3(0,0,0))
                rospy.sleep(0.1)


                if bumper == 1:

                    if velocidade1 == True:
                        vel = Twist(Vector3(-v,0,0), Vector3(0,0,w))
                        pub.publish(vel)
                        rospy.sleep(2)
                        bumper = None
                        velocidade1 = False


                    else:
                        vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
                        pub.publish(vel)
                        rospy.sleep(2)
                        bumper = None
                        velocidade1 = True


                elif bumper == 2:

                    if velocidade1 == True:
                        vel = Twist(Vector3(-v,0,0), Vector3(0,0,w))
                        pub.publish(vel)
                        rospy.sleep(2)
                        bumper = None
                        velocidade1 = False


                    else:
                        vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
                        pub.publish(vel)
                        rospy.sleep(2)
                        bumper = None
                        velocidade1 = True


                elif bumper == 3:

                    if velocidade1 == True:
                        vel = Twist(Vector3(-v,0,0), Vector3(0,0,w))
                        pub.publish(vel)
                        rospy.sleep(2)
                        bumper = None
                        velocidade1 = False


                    else:
                        vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
                        pub.publish(vel)
                        rospy.sleep(2)
                        bumper = None
                        velocidade1 = True

                elif bumper == 4:

                    if velocidade1 == True:
                        vel = Twist(Vector3(-v,0,0), Vector3(0,0,w))
                        pub.publish(vel)
                        rospy.sleep(2)
                        bumper = None
                        velocidade1 = False


                    else:
                        vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
                        pub.publish(vel)
                        rospy.sleep(2)
                        bumper = None
                        velocidade1 = True


                else:

                    vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
                    rospy.sleep(0.1)

                if centro[0] - media[0] > 30:
                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,-0.2))
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.5)
                elif centro[0] - media[0] < -30:
                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0.2))
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.5)
                else:
                    vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.5)

            else:

                vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
                rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
