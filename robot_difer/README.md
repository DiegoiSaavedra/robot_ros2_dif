# robot_difer

Este paquete contiene el nodo `base_driver` para un robot diferencial equipado con un puente L298D, un LIDAR Youyeetoo L90D y un IMU MPU9250.  El nodo lee ticks de encoders a través de un puerto serie, calcula la odometría, publica la transformada `odom \rightarrow base_link` y envía comandos PWM al microcontrolador.

## Estructura del paquete

* `robot_difer/base_driver.py`: implementación del nodo Python.  Declara parámetros para el puerto serie y velocidad de baudios, calcula la odometría con los parámetros físicos proporcionados y difunde la transformación TF.
* `launch/robot_difer_launch.py`: archivo de lanzamiento que inicia `base_driver`, el driver del LIDAR (`ydlidar_ros2_driver`) y el nodo del IMU (`mpu9250`).  Ajuste los parámetros de puerto y frames según su hardware.
* `resource/robot_difer`: marcador necesario para registrar el paquete en `ament`.
* `package.xml`, `setup.py`, `setup.cfg`: metadatos y configuración de instalación para `ament_python`.

## Requisitos

* Ubuntu 20.04 con ROS 2 Jazzy instalado.
* Dependencias: `rclpy`, `geometry_msgs`, `nav_msgs`, `tf2_ros`, `tf_transformations`, `sensor_msgs`, `serial` (python3-serial), `ydlidar_ros2_driver`, `mpu9250`.

## Compilación

1. Cree un espacio de trabajo y clone este paquete dentro de `src` (según las instrucciones del proyecto, este paquete vive en `~/ros2_ws/src/robot_difer`).

```bash
mkdir -p ~/ros2_ws/src
# Copiar o clonar el paquete aquí
```

2. Compile el workspace con `colcon`:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

3. Fuente el overlay:

```bash
source install/setup.bash
```

## Ejecución

Para lanzar el robot con el LIDAR y el IMU ejecutando el nodo base_driver:

```bash
ros2 launch robot_difer robot_difer_launch.py
```

Puede ajustar los parámetros de puerto y baudios en el archivo de lanzamiento o mediante parámetros de línea de comandos.
