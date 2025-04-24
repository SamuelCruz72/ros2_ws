# Laboratorio No. 01 - 2025-I - Robótica de Desarrollo, Intro a ROS 2 Humble - Turtlesim
El objetivo primordial de esta práctica de laboratorio es familiarizarse con el entorno básico de ROS2, en particular con el nodo Turtlesim; para ello, este nodo se conectó con Python, en primera instancia mediante la creación de un workspace de ROS2 a través de los siguientes comandos en la terminal de Linux:

```bash
mkdir -p ~/Laboratorio_Robotica/lab_1/ros2_ws/src
cd ..
colcon build
source /opt/ros/humble/setup.bash
```

Una vez creado el workspace, con ROS2 se creó un paquete que genera el código en Python que permite efectuar la conexión con el nodo Turtlesim:

```bash
cd ~/Laboratorio_Robotica/lab_1/ros2_ws/src
ros2 pkg create --build-type ament_python my_turtle_controller
```

A partir de este paquete, se creó un archivo ``move_turtle.py`` en la carpeta ``my_turtle_controller/my_turtle_controller/`` con el cual se inicializó la comunicación con el nodo Turtlesim mediante la sección del código ``rclpy.spin(node)``. Consiguientemente, se implementaron funciones en la clase ``TurtleController`` que regulan el movimiento de la tortuga mediante la interacción con el teclado.

Una vez se completaron todas las funciones del archivo ``move_turtle.py``, se guardaron los cambios y se retornó a la raíz del workspace para compilar el código y ejecutarlo:

```bash
cd ~/Laboratorio_Robotica/lab_1/ros2_ws
colcon build
source install/setup.bash
ros2 run my_turtle_controller move_turtle
```

A continuación se explica con mayor detalle cada una de las funciones implementadas para el control de movimiento de la tortuga.

## Control del nodo Turtlesim con flechas

Para el control del nodo con las flechas del teclado, primero se implementó un hilo adicional que se encargara unicamente de escuchar el teclado, luego se creó la función ``listen_keyboard``

## Generación de Trayectorias

### Trayectoria de la S

<p align="center">
   <img src="Figuras/Traj_S.png" alt="S generada con Matlab" width="400"><br> 

<p align="center">
   <img src="Figuras/SRos2.png" alt="S generada con ROS2" width="500"><br> 

### Trayectoria de la A

<p align="center">
   <img src="Figuras/Traj_A.png" alt="A generada con Matlab" width="400"><br> 

<p align="center">
   <img src="Figuras/ARos2.png" alt="A generada con ROS2" width="500"><br> 

### Trayectoria de la C

<p align="center">
   <img src="Figuras/Traj_C.png" alt="C generada con Matlab" width="500"><br> 

<p align="center">
   <img src="Figuras/CRos2.png" alt="C generada con ROS2" width="500"><br> 
