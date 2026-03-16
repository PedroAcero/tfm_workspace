## Lanzar simulador URSim (opcional)

Para simular pruebas con el robot, se lanza la simulación para la conexión con el dirver contenida en una imagen de Docker

```bash
ros2 run ur_client_library start_ursim.sh -m ur10
```
Si no hay problemas con el comando anterior, te abre un puerto en tu ordenador, que es accesible desde el navegador.

## Conexión con el robot

Como se explica en la [guía oficial](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/network_setup.html), es necesario configurar
la red del robot para que tenga una dirección IP estática. Si no se ha seguido este paso, el parametro `robot_ip` puede cambiar.

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.56.101 launch_rviz:=false
```
Una vez lanzado el driver, hay que ceder el control del robot a través del programa de `external_control` configurado en el _Teach Pendant_. Lanzar desde el _Teach_ o con los siguientes
comandos:

```bash
ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load "filename: external_control.urp"``
ros2 service call /dashboard_client/play std_srvs/srv/Trigger {}
```
## Lanzar la simulación de MoveIt

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=true
```

## Lanzar programas de desarrollo

```bash
ros2 launch npam_trajectory cartesian_trajectory.launch.py velocity_scaling:=1.0 acceleration_scaling:=1.0
```

```bash
ros2 launch npam_trajectory pilz_trajectory.launch.py
```

```bash
ros2 run npam_logger plan_logger
```
```bash
ros2 run npam_logger commanded_logger
```

## Verificaciones

Comprobar datos de posición, velocidad y transformaciones

```bash
ros2 topic echo /joint_states --once
```

```bash
ros2 run tf2_ros tf2_echo world cama_impresion
ros2 run tf2_ros tf2_echo world cama_impresion 2>/dev/null | grep -E "Translation|Quaternion"
```
## Cambiar parámetros de ROS

```bash
ros2 param set /scaled_joint_trajectory_controller constraints.shoulder_pan_joint.trajectory 5.0 && \
ros2 param set /scaled_joint_trajectory_controller constraints.shoulder_lift_joint.trajectory 5.0 && \
ros2 param set /scaled_joint_trajectory_controller constraints.elbow_joint.trajectory 5.0 && \
ros2 param set /scaled_joint_trajectory_controller constraints.wrist_1_joint.trajectory 5.0 && \
ros2 param set /scaled_joint_trajectory_controller constraints.wrist_2_joint.trajectory 5.0 && \
ros2 param set /scaled_joint_trajectory_controller constraints.wrist_3_joint.trajectory 5.0
```
