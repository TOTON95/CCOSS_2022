#!/usr/bin/python3

# Basado en

# Importamos las librerias
import rospy
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState,GetModelState

# Creamos el mensaje para guardar el estado del modelo
drone_state = ModelState()

# Establecer el callback para la deteccion del drone
def get_drone_state_cb(msg):
    global drone_state
    if (msg.model_name == 'solo'):
        drone_state = msg


def main():
    # Iniciamos el nodo de ROS
    rospy.init_node('carga_util_act_node')

    # Creamos la pose para el paquete
    estado_modelo = ModelState()
    estado_modelo = drone_state
    estado_modelo.pose.position.x = drone_state.pose.position.x = 0.0
    estado_modelo.pose.position.y = drone_state.pose.position.y = 0.0
    estado_modelo.pose.position.z = drone_state.pose.position.z = 0.05

    # Esperamos por el servicio de gazebo para obtener el estado de los modelos
    rospy.wait_for_service("/gazebo/get_model_state")

    # Esperamos por el servicio de gazebo para establecer el estado de los modelos
    rospy.wait_for_service("/gazebo/set_model_state")
