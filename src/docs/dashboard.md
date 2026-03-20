# Prueba de Control del _Teach Pendant_ con ROS2
 
Este documento es una guía de planificación para establecer conexión con el ur10 real y validar el control externo, mensajes, freedrive y diagnóstico de estado.
 
---

## Requisitos previos

Antes de comenzar con las pruebas, es necesario tener configurada la *red de conexión* entre el ur10 y el PC.

### En el _Teach Pendant_ (ur10)

Accede a `Setup Robot → Network` y configura los siguientes parámetros:
 
| Parámetro              | Valor             |
|------------------------|:-----------------:|
| IP address             | `192.168.0.102`  |
| Subnet mask            | `255.255.255.0`   |
| Default gateway        | `192.168.0.9`    |
| Preferred DNS server   | `192.168.0.9`    |
| Alternative DNS server | `0.0.0.0`         |

### En el PC
 
1. Desactiva el WiFi y cualquier otra interfaz de red activa que use el mismo rango IP.
2. Crea una nueva conexión ethernet con los siguientes parámetros:
 
| Parámetro | Valor            |
|-----------|:----------------:|
| Modo IPv4 | Manual           |
| Address   | `192.168.0.1`   |
| Netmask   | `255.255.255.0`  |

3. Verifica la conexión entre el PC y el ur10 haciendo ping al robot desde la terminal:
 
```
ping 192.168.0.9
```

Si todo ha ido bien y se ha establecido conexión, la respuesta de la terminal debería ser algo similar a lo siguiente:

```
64 bytes from 192.168.56.101: icmp_seq=1 ttl=64 time=0.037 ms
```

## Lanzamiento del _Driver_ de ROS2
 
Antes de ceder el control a ROS2, es necesario lanzar los controladores de ROS2 a través del driver de Universal Robots ([link](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble))
 
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.0.9 launch_rviz:=false
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
 
---

## 4️⃣ Análisis del estado del robot durante las pruebas

## 5️⃣ Obtención de posición inicial



