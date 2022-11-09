#!/usr/bin/python3


# Muestra de código tomada de PX4 (https://docs.px4.io/main/en/ros/mavros_offboard_python.html)

"""
Se importan las librerías para ser usadas para interactuar
con ros y PX4
"""
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# Recipiente para el estado reportado por MAVROS
current_state = State()

# Callback para obtener cualquier nuevo mensaje de estado
def state_cb(msg):
    global current_state
    current_state = msg


# Comienzo del programa
if __name__ == '__main__':
    # Se crea un nodo de ROS
    rospy.init_node("test_offb_node_py")
    # Suscriptor para el estado reportado en MAVROS
    state_sub = rospy.Subscriber("/mavros/state", State, callback = state_cb)
    # Publicador para el punto de destino en el sistema de coordenadas local
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Se espera por el servicio que permita armar los motores
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    # Se espera por el servicio que permita cambiar entre modos de vuelo
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    # Frecuencia a la que queremos que corra nuestro nodo, debe ser mayor a 2hz
    rate = rospy.Rate(20)

    # Esperamos por la conexión con el controlador
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # Creamos una pose para que se mantenga a dos metros sobre el suelo
    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Mandamos varios puntos objetivo, para que el controlador respete el modo de vuelo
    for i in range(100):
        if(rospy.is_shutdown()):
            break
        # Publicamos la pose deseada al controlador usando MAVROS
        local_pos_pub.publish(pose)
        # Mantenemos la frecuencia del sistema
        rate.sleep()


    # Creamos un Request para cambiar de modo en el controlador
    offb_set_mode = SetModeRequest()
    # Seleccionamos el modo OFFBOARD para acciones autonomas
    offb_set_mode.custom_mode = 'OFFBOARD'

    # Creamos un request para pedir que los motores se armen
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    # Obtenemos el tiempo actual en el sistema ROS
    last_req = rospy.Time.now()

    # Ciclo while que continúa de forma perpetua hasta que el nodo o
    # el master son detenidos
    while(not rospy.is_shutdown()):
        # Se da tiempo a que se inicien los sistemas y se solicita el cambio de modo
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent is True):
                rospy.loginfo("OFFBOARD activado")

            last_req = rospy.Time.now()
        else:
            # Operación para el armado de motores
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success is True):
                    rospy.loginfo("Vehiculo armado")

                last_req = rospy.Time.now()

        # Se mandan la pose deseada
        local_pos_pub.publish(pose)

        rate.sleep()
