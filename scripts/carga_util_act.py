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

# Establecer el callback para la deteccion del drone
def sujetar_paquete_cb(msg):
    global sujetar
    sujetar = msg.data


def main():
    # Iniciamos el nodo de ROS
    rospy.init_node('carga_util_act_node')

    # Creamos el mensaje para guardar el estado del modelo
    drone_estado = ModelState()

    # Creamos la suscripcion del drone para que se sujete
    rospy.Subscriber("/solo/sujetar", Bool)

    # Creamos la pose para el paquete
    caja_estado = ModelState()
    caja_estado = drone_estado
    caja_estado.pose.position.x = drone_estado.pose.position.x
    caja_estado.pose.position.y = drone_estado.pose.position.y
    caja_estado.pose.position.z = drone_estado.pose.position.z - 0.05

    # Esperamos por el servicio de gazebo para obtener el estado de los modelos
    rospy.wait_for_service("/gazebo/get_model_state")
    get_estado_drone = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    # Esperamos por el servicio de gazebo para establecer el estado de los modelos
    rospy.wait_for_service("/gazebo/set_model_state")
    set_estado_caja = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    try:
        res_get = get_estado_drone(drone_estado)
        res_set = set_estado_caja(caja_estado)

    except (rospy.ServiceException) as e:
        print("Service call failed: %s" % e)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

