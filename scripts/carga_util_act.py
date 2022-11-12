#!/usr/bin/python3

# Basado en la respuesta a https://answers.gazebosim.org//question/22125/how-to-set-a-models-position-using-gazeboset_model_state-service-in-python/

# Importamos las librerias
import rospy
import rospkg
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState,GetModelState

# Hacemos una variable donde si es verdadero, debe sujetar al paquete
sujetar = False


# Callback para cambiar si debe ser sujetado
def sujetar_paquete_cb(msg):
    global sujetar
    sujetar = msg.data

# Callback para obtener los datos del drone
def get_data_drone_cb(msg):
    global drone_estado
    print(drone)


def main():
    # Iniciamos el nodo de ROS
    rospy.init_node('carga_util_act_node')

    # Creamos el mensaje para guardar el estado del modelo
    drone_estado = ModelState()

    # Creamos la suscripcion del drone para que se sujete
    rospy.Subscriber("/solo/sujetar", Bool)

    # Aquí empezamos a crear el código del sujetador

    # Creamos la pose para el paquete
    caja_estado = ModelState()
    caja_estado = drone_estado
    caja_estado.pose.position.x = drone_estado.pose.position.x
    caja_estado.pose.position.y = drone_estado.pose.position.y
    caja_estado.pose.position.z = drone_estado.pose.position.z - 0.05



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

