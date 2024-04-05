import sys
import asyncio
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

import time

from threading import Thread

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Int8

from functools import partial
from ur_msgs.srv import SetIO

from my_moveit2_py.Moveit2_resources import MoveIt2 #own resources
from my_moveit2_py import ur3e_model #own resources

from rclpy.callback_groups import ReentrantCallbackGroup

#GUI
import tkinter as tk
from PIL import Image,ImageTk
from tkinter import font

# Victor descripcion de los argumentos
#args = {'ur_type':'ur3e', 'robot_ip': '192.168.20.35','use_fake_hardware':True,'launch_rviz':True,'initial_joint_controller':'joint_trajectory_controller'}

class conexion(Node):
    def __init__(self):
        super().__init__("conexion")
        
    # Declaration of parameters

        self.declare_parameter("controller_name", "joint_trajectory_position_controller")
        self.declare_parameter("joints",["shoulder_pan_joint", "shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]) #Obligado para lanzar inputs
        self.declare_parameter("check_starting_point", False)
        self.declare_parameter("starting_point_limits")

    # Parameters for the robot
    #    self.pos_wanted = JointTrajectory()
    #    controller_name = self.get_parameter("controller_name").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}
        self.get_logger().info(
            'LETSGO'
            )
        self.inp = None

        # Victor
        print("joints: ", self.joints, "\n starting point: ", self.check_starting_point)
        
        
    #Main values for topics	
        self.pose_required = Pose()
      
   
	#Safety joints
        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        #Supervisor of starting points
        if self.check_starting_point:
            # Nested params
            for name in self.joints:
                param_name_tmp = "starting_point_limits" + "." + name
                self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param_name_tmp).value
            #Testing
            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise Exception('"starting_point" parameter is not set correctly!')
            self.joint_state_sub = self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 10
            )

            
    # Starting point status
        if not self.check_starting_point:
            self.starting_point_ok = True
        else:
            self.starting_point_ok = False

        self.joint_state_msg_received = False
            
    #USE OF GRIPPER
    
    #Client for the service of the gripper
    def call_gripper(self, fun, pin, state):
            client = self.create_client(SetIO, "/io_and_status_controller/set_io") #servicio de la pinza ya creado.
        
            while not client.wait_for_service(1.0):
	            self.get_logger().warn("Waiting for Server SetIO for the gripper to be opened...")
        
            request = SetIO.Request()
            request.fun = fun
            request.pin = pin
            request.state = state
        
            future = client.call_async(request)
            future.add_done_callback(
		        partial(self.callback_gripper, fun=fun, pin=pin, state=state))
		
    def callback_gripper(self, future, fun, pin, state):
    	try:
      	    response = future.result()
            #self.get_logger().info('The response of the gripper is:' + str(response.success) + '.')

    	except Exception as e:
	        self.get_logger().error("Service reset call failed %r" % (e,))

    #The pin can varies with the robot

    #Function for closing the gripper
    def close_gripper(self):
        self.call_gripper(1, 17, 24.0)
        time.sleep(0.5)
        self.call_gripper(1,17,0.0) #Hay que descargar la pinza.
    
    #Function for opening the gripper
    def open_gripper(self):
        self.call_gripper(1, 16, 24.0)
        time.sleep(0.5)
        self.call_gripper(1,16,0.0) #Hay que descargar la pinza.
        
    #Joints state supervisor
    def joint_state_callback(self, msg):

        if not self.joint_state_msg_received:

            # Checking start state not to exceeded
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):
                    self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
                    limit_exceeded[idx] = True

            if any(limit_exceeded):
                self.starting_point_ok = False
            else:
                self.starting_point_ok = True

            self.joint_state_msg_received = True
        else:
            return

    # Victor funcion para comprobar si esta conectado
    def robot_connected(self):

        try:
            # Inicializar ROS
            #rclpy.init()

            # Intentar crear un nodo
            conex_test = rclpy.create_node("connection_test_node")

            # Si llegamos hasta aquí sin errores, la conexión al robot es exitosa
            conex_test.destroy_node()
            #rclpy.shutdown()
            return True
        except Exception as e:
            # Si ocurre algún error durante la inicialización de ROS o la creación del nodo, asumimos que la conexión falla
            return False

def main(args = None):
    
    rclpy.init(args = args)

    control_node = conexion() #Object of the class created for the control of the robot    
    
    # Victor comprobacion de conexion
    if control_node.robot_connected():
        print("Conexión al robot establecida con éxito.")      
    else:
        print("No se pudo conectar al robot. Verifique la configuración de ROS y vuelva a intentarlo.")
        sys.exit(1)

    #Instatiation of Moveit
    moveit2 = MoveIt2(
        node=control_node,
        joint_names=ur3e_model.joint_names(),
        base_link_name=ur3e_model.base_link_name(),
        end_effector_name=ur3e_model.end_effector_name(),
        group_name=ur3e_model.MOVE_GROUP_ARM,
        #callback_group=self.callback_group, #Optional
        )
    
    #Multithread programming
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(control_node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    control_node.close_gripper() 
    time.sleep(2)

    #joint_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    #control_node.send_joint_commands(joint_values)

    control_node.open_gripper() 
    time.sleep(2)

    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
