# Turtlebot hardware

Este paquete esta dedicado para la variacion del hardware del turtlebot 3 perteneciente al laboratorio.

### Primeros pasos
* Instalar Ubuntu Desktop 22.04 en la Raspberry Pi 4.
* Instalar ROS 2 Humble Base.
* Habilitar los puertos SSH para comunicacion con la raspberry.



### Configuracion del ws 
Aqui simplemente creamos una carpeta mediante el comando 
~~~
mkdir turtlebot_ws
cd turtlebot_ws
mkdir src
cd src
git clone https://github.com/Cesarq19/pruebaTB3.git
~~~

### Instalacion y configuracion de dependencias 

Respecto al imu debemos instalar el siguiente debian package:
~~~
sudo apt-get install libqt5serialport5-dev
~~~

Mientras que para el lidar se debe realizar la siguiente configuracion seguir los pasos: https://github.com/Cesarq19/pruebaTB3/blob/main/YDLidar-SDK/doc/howto/how_to_build_and_install.md

### Instalar las dependencias necesarias mediante rosdep
~~~
rosdep install --from-paths src -y --ignore-src
~~~

### Bringup robot
~~~
ros2 launch turtlebot3_bringup turtlebot3_robot.launch.py
~~~