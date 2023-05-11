#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time

from std_msgs.msg import String

class RosLink(object):
  # Clase encargada en enviar y recibir comandos de ROS para comunicar las
  # partes de visión y lógica del juego con el movimiento del brazo.

  def __init__(self):
    # Creación del objeto Rate con la frecuencia de funcionamiento.
    self.rate = rospy.Rate(10)

    # Topics handlers
    self.msg_publisher = None
    self.msg_subscriber = None

    # Mensajes enviados y recibidos
    self.p_msg = String() 
    self.s_msg = String()

    # Inicialización de publicadores y subscriptores
    self.init_publishers()
    time.sleep(0.2)
    self.init_subscribers()
    time.sleep(0.2)

  def init_publishers(self):
    self.msg_publisher = rospy.Publisher('p_topic', String, queue_size=5)

  def init_subscribers(self):
    self.grid_map_subscriber = rospy.Subscriber('s_topic', String, self.handle_msg)
    
  def publish_msg(self):
    self.msg_publisher.publish(self.p_msg)
    
  def handle_msg(self, msg):
    self.msg_subscriber = msg.data

  def run(self):  
    while not rospy.is_shutdown():
      '''
      Insertar aquí el código que se quiere ejecutar cíclicamente
      '''
      self.rate.sleep()

if __name__ == '__main__':
    # Nombre del nodo : "ROS link".
    rospy.init_node("RosLink")
    print("Nodo RosLink activo")
    ROS_link = RosLink()
    
    ROS_link.run()
