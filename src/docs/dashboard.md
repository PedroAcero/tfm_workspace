# Prueba de Control del _Teach Pendant_ con ROS2
 
Este documento es una guía de planificación para establecer conexión con el ur10 real, y validar el control externo del robot.
 
---

## Objetivos

- [ ] Familiarizarse con el entorno de trabajo del laboratorio  
- [ ] Documentar los datos específicos de la conexión del robot con el PC  
- [ ] Primer contacto de control del robot con ROS  
- [ ] Diagnóstico y análisis del timeout de desconexión 

---

## Requisitos previos

Antes de comenzar con las pruebas, es necesario tener configurada la *red de conexión* entre el ur10 y el PC.

### En el _Teach Pendant_ (ur10)

Accede a `Setup Robot → Network` y configura los siguientes parámetros:
 
| Parámetro              | Valor             |
|------------------------|:-----------------:|
| IP address             | `ip_ordenador`  |
| Subnet mask            | `XXX.XXX.XXX.X`   |
| Default gateway        | `XXX.XXX.X.X`    |
| Preferred DNS server   | `XXX.XXX.X.X`    |
| Alternative DNS server | `X.X.X.X`         |

### En el PC
 
1. Desactiva el WiFi y cualquier otra interfaz de red activa que use el mismo rango IP.
2. Crea una nueva conexión ethernet con los siguientes parámetros:
 
| Parámetro | Valor            |
|-----------|:----------------:|
| Modo IPv4 | Manual           |
| Address   | `XXX.XXX.X.X`   |
| Netmask   | `XXX.XXX.XXX.X`  |

3. Verifica la conexión entre el PC y el ur10 haciendo ping al robot desde la terminal:
 
```
ping <ip_ordenador>
```

Si todo ha ido bien y se ha establecido conexión, la respuesta de la terminal debería ser algo similar a lo siguiente:

```
64 bytes from <ip_ordenador>: icmp_seq=1 ttl=64 time=0.037 ms
```

## Lanzamiento del _Driver_ de ROS2
 
Antes de ceder el control a ROS2, es necesario lanzar los controladores de ROS2 a través del driver de Universal Robots ([link](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble))
 
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=<ip_ordenador> launch_rviz:=false
```
 
Con el driver lanzado, se comprueba que se encuentran activos los servicios de ROS para el _dashboard_.
 
```bash
ros2 service list | grep dashboard
```
 
## 1️⃣ Activar el External Control
 
Desde el _Teach Pendant_, ejecuta el programa de `external_control.urp` URCaps previamente instalado:
 
```bash
ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load "filename: external_control.urp"``
```
```bash
ros2 service call /dashboard_client/play std_srvs/srv/Trigger {}
```
 
> [!NOTE]
> El archivo `external_control.urp` debe estar previamente instalado en el robot. En caso de haber algún problema o reinstalar el External Control, revisar el siguiente [link](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html).

---

## 2️⃣ Escritura de mensajes en el _Teach Pendant_
 
### Añadir entrada al log del robot
 
```bash
ros2 service call /dashboard_client/add_to_log ur_dashboard_msgs/srv/AddToLog "{message: 'Mensaje desde mi pc: hola mundo'}"
```

### Mostrar un popup y cerrarlo
 
```bash
ros2 service call /dashboard_client/popup ur_dashboard_msgs/srv/Popup "{message: 'Pulsa OK para continuar'}"
 ```
```bash
ros2 service call /dashboard_client/close_popup std_srvs/srv/Trigger "{}"
```

---

## 3️⃣ Activar Freedrive
 
Mover el ur10 a través del programa predefinido de freedrive: 
 
```bash
ros2 topic pub --rate 2 /freedrive_mode_controller/enable_freedrive_mode std_msgs/msg/Bool "{data: true}"
```
En caso de querer hacer otras comprobaciones, mirar la lista de servicios disponibles en el entorno:

```bash
ros2 service list
```
 
---

## 4️⃣ Análisis del estado del robot durante las pruebas

Durante el desarrollo de estas pruebas, se recomienda comprobar de vez en cuando el estado del robot. Se ha comentado que existe un _timeout_ que desconecta el External Control sin avisar. Como objetivo secundario (aunque de gran importancia) en estas pruebas, se espera diagnosticar este _timeout_. 

### Comprobar el estado del programa

```bash
ros2 service call /dashboard_client/program_state ur_dashboard_msgs/srv/GetProgramState "{}"
```
Los posibles estados que pueden aparecer son los siguientes:

| Estado   | Descripción                             |
|----------|-----------------------------------------|
| STOPPED  | No hay programa en ejecución            |
| PLAYING  | Hay un programa cargando y en ejecución |
| PAUSED   | Hay un programa cargado y en pausa      |

> [!NOTE]
> Cuando se active el external control, el estado del programa debería ser `PLAYING`.

### Comprobar el modo del robot

```bash
ros2 service call /dashboard_client/get_robot_mode ur_dashboard_msgs/srv/GetRobotMode "{}"
```
Los posibles modos que pueden aparecer son los siguientes:

 Valor  | Estado             | Descripción                                     |
|-------|--------------------|-------------------------------------------------|
| -1    | NO_CONTROLLER      | No se detecta controlador                       |
| 0     | DISCONNECTED       | No hay conexión con el robot                    |
| 1     | CONFIRM_SAFETY     | Espera de confirmación tras una parada          |
| 2     | BOOTING            | Robot en proceso de arranque                    |
| 3     | POWER_OFF          | Alimentación apagada                            |
| 4     | POWER_ON           | Alimentación encendida, pero robot no operativo |
| 5     | IDLE               | Robot operativo, esperando un programa          |
| 6     | BACKDRIVE          | Modo _freedrive_                                |
| 7     | RUNNING            | Programa ejecutándose                           |
| 8     | UPDATING_FIRMWARE  | Software del robot actualizándose               |

### Comprobar modo de seguridad

```bash
ros2 service call /dashboard_client/get_safety_mode ur_dashboard_msgs/srv/GetSafetyMode "{}"
```
Los posibles modos de seguridad son los siguientes:


 Valor  | Estado                               | Descripción                                                                  |
|--------|-------------------------------------|------------------------------------------------------------------------------|
| 1      | NORMAL                              | El robot acepta movimientos o servicios                                      |
| 2      | REDUCED                             | El robot acepta movimientos con limitaciones                                 |
| 3      | PROTECTIVE_STOP                     | El controlador del robot detecta que no puede hacer el movimiento solicitado |
| 4      | RECOVERY                            | Estado del robot tras un evento de seguridad                                 |
| 5      | SAFEGUARD_STOP                      | Sistema salvaguarda                                                          |
| 6      | SYSTEM_EMERGENCY_STOP               | Parada a nivel sistema                                                       |
| 7      | ROBOT_EMERGENCY_STOP                | Parada a nivel robot                                                         |
| 8      | VIOLATION                           |                                                                              |
| 9      | FAULT                               | Fallo de seguridad, llamar a soporte técnico                                 |
| 10     | VALIDATE_JOINT_ID                   | Fallo de seguridad, responsabilidad del fabricante                           |  
| 11     | UNDEFINED_SAFETY_MODE               | Inconsistencia indefinida                                                    |
| 12     | AUTOMATIC_MODE_SAFEGUARD_STOP       |                                                                              |
| 13     | SYSTEM_THREE_POSITION_ENABLING_STOP |                                                                              |

### Restaurar el modo de operación

En caso de desconexión con el robot debido al _timeout_ o por cualquiero otro motivo, se puede recuperar el estado del robot con el siguiente comando:

```bash
ros2 action send_goal /ur_robot_state_helper/set_mode ur_dashboard_msgs/action/SetMode "{ target_robot_mode: 7, stop_program: true, play_program: true}"
```
> Este `action` de ROS intenta llevar el robot al modo `RUNNING` (7), detiene el programa activo y lo vuelve a lanzar.

## 5️⃣ Obtención de posiciones

Para la obtención de posiciones o comprobaciones en las trayectorias, se pueden leer el estado del robot con los siguientes comandos. 

```bash
ros2 topic echo /joint_states
```
> Este `topic` publica las posiciones articulares del robot.

```bash
ros2 run tf2_ros tf2_echo world cama_impresion
```
> Este programa predefinido da las transformaciones entre dos sistemas de referencia. El formato es de cuaterniones unitarios, posiciones cartesianas.
