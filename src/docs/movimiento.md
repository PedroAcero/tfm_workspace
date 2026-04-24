# Prueba de movimiento con ROS

Iniciar _Teach Pendant_ y robot.

Configuración de red y comprobar que se ven.

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.56.101 launch_rviz:=false
```

Activar external control

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=true
```

```bash
ros2 launch npam_trajectory pilz_trajectory.launch.py
```


Flujo habitual de trabajo con una trayectoria nueva:

1. Ejecutar `cartesian_planner` con un escalado del 100%. Las velocidades **articulares** del robot estarán capadas por las restricciones establecidas en `joint_limits.yaml`
2. Medir con ayuda de `npam_logger` la velocidad **cartesiana** máxima de la trayectoria.
3. Modificar del archivo `pilz_cartesian_limits.yaml` el parámetro _max_trans_vel_ con ese nuevo valor.
4. Recompilar `npam_trajectory` y ejecutar `pilz_planner`.


```bash
ros2 topic echo /scaled_joint_trajectory_controller/state
```
