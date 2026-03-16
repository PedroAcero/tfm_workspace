# Guía de Instalación

En este documento describe los pasos necesarios para instalar y configurar el repositorio desarrollado para el TFM y el INGENIA del curso 25/26.

## Requisitos previos

Para la instalación del repositorio se parte de un equipo con los siguientes componentes instalados:

- Ubuntu 22.04
- ROS2 Humble instalado ([guía oficial](https://docs.ros.org/en/humble/Installation.html))

## 1. Instalar herramientas de desarrollo

Para comenzar con la instalación, es recomendable actualizar la lista de paquetes e instalar las dependencias necesarias del sistema

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep git -y
```

## 2. Clonar el repositorio

El siguiente paso es clonar el repositorio en una carpeta llamada `workspace`

```bash
mkdir -p ~/workspace/src
cd ~/workspace/src
git clone https://github.com/PedroAcero/tfm_workspace.git
```

## 3. Instalación de las dependencias del sistema

Antes de compilar, es necesario instalar los paquetes de ROS2 necesarios para que funcione correctamente

```bash
sudo apt install \
  ros-humble-tf2-kdl \
  ros-humble-kdl-parser \
  ros-humble-moveit \
  ros-humble-ur-robot-driver \
  ros-humble-ur-msgs \
  ros-humble-generate-parameter-library \
  ros-humble-generate-parameter-library-py \
  ros-humble-ur-client-library \
  ros-humble-realtime-tools -y
```

## 4. Instalar dependencias con rosdep

`rosdep` analiza los paquetes de `workspace` y descarga automáticamente cualquier dependencia adicional que no esté instalada.

```bash
cd ~/workspace
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 5. Compilación 

Compilación de todos los paquetes del `workspace`. El flag `--allow-overriding` es necesario porque algunos paquetes del repo 
sobreescriben a otros ya instalados en el sistema.

```bash
cd ~/workspace
colcon build \
  --allow-overriding ur_description ur_moveit_config ur_controllers ur_robot_driver ur_calibration ur_dashboard_msgs ur_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Es posible que las primera vez no funcione la compilación. Antes de continuar, es necesario saltar a la sección "Resolución de errores", 
e identificar los errores encontrados durante la compilación.

### 6. Source del workspace

Cada vez que se abre una nueva terminal donde quieras usar el _workspace_, es necesario hacer `source` de los archivos compilados

```bash
source ~/workspace/install/setup.bash
```

Si no se quiere hacer `source` cada vez que abres una nueva terminal, puedes añadir el source al `.bashrc`

```bash
echo "source ~/workspace/install/setup.bash" >> ~/.bashrc
```

### 7. Verificación

Comprueba que todos los paquetes están disponibles correctamente con los siguientes comandos en una nueva terminal:

```bash
ros2 pkg list | grep npam
ros2 pkg list | grep ur_
```

La terminal debería devolver `npam_logger`, `npam_trajectory` y todos los paquetes desarrollados por [Universal Robots](https://github.com/UniversalRobots)

- `ur_description`
- `ur_moveit_config`
- `ur_controllers`
- `ur_robot_driver`
- `ur_calibration`
- `ur_dashboard_msgs`

### Simulador URSim con Docker

Para las pruebas de simulación, Universal Robots proporciona una [imagen Docker](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/simulation.html) del simulador URSim. Esto abre un puerto en tu sistema que es similar al robot real desde el punto de vista del driver.  

Para ello, 

```bash
# Añadir usuario al grupo docker (solo la primera vez, es necesario reiniciar)
sudo usermod -aG docker $USER
newgrp docker

docker run --rm -it -p 5900:5900 -p 6080:6080 --name ursim universalrobots/ursim_e-series
```

El simulador es accesible desde el navegador en: `http://localhost:6080`

---

## Resolución de errores

En esta sección se recogen los errores encontrados durante la instalación del repo en el portátil del laboratorio, y cómo se resolvieron.

### Error: `tf2_kdl::tf2_kdl` not found

El `CMakeLists.txt` de `npam_trajectory` no declara `tf2_kdl` como dependencia de CMake aunque sí la usa internamente. Añádela manualmente:

```bash
sed -i 's/find_package(Boost REQUIRED COMPONENTS system filesystem)/find_package(Boost REQUIRED COMPONENTS system filesystem)\nfind_package(tf2_kdl REQUIRED)/' \
  ~/workspace/src/npam_trajectory/CMakeLists.txt
```

### Error: incompatibilidades en `ur_controllers`

El `Universal_Robots_ROS2_Driver` incluido en el repo es la versión 2.9.0, mientras
que algunas dependencias del sistema están en versiones más recientes con cambios de
API incompatibles. Aplica el siguiente parche:
```bash
FILE=~/workspace/src/Universal_Robots_ROS2_Driver/ur_controllers/src/scaled_joint_trajectory_controller.cpp

sed -i 's/get_segment_tolerances(logger, params_)/get_segment_tolerances(params_)/' $FILE
sed -i 's/rt_has_pending_goal_ = false/rt_has_pending_goal_.writeFromNonRT(false)/g' $FILE
sed -i 's/!rt_has_pending_goal_/!(*rt_has_pending_goal_.readFromRT())/g' $FILE
sed -i 's/rt_has_pending_goal_ &&/(*rt_has_pending_goal_.readFromRT()) \&\&/g' $FILE
sed -i 's/!rt_is_holding_/!(*rt_is_holding_.readFromRT())/g' $FILE
sed -i 's/auto active_tol = active_tolerances_.readFromRT();/const auto \& active_tol = default_tolerances_;/' $FILE
sed -i 's/active_tol->/active_tol./g' $FILE
sed -i 's/!check_state_tolerance_per_joint/!joint_trajectory_controller::check_state_tolerance_per_joint/g' $FILE

python3 -c "
import re
with open('$FILE', 'r') as f:
    content = f.read()
content = re.sub(
    r'set_point_before_trajectory_msg\(([^,]+),\s*([^,]+),\s*joints_angle_wraparound_\)',
    r'set_point_before_trajectory_msg(\1, \2)',
    content
)
with open('$FILE', 'w') as f:
    f.write(content)
"
```

### Error: `ur_client_library/primary/primary_client.h` not found

La versión instalada de `ur_client_library` es demasiado antigua. Actualízala:
```bash
sudo apt install ros-humble-ur-client-library -y
```

### Error: `Could not get lock /var/lib/dpkg/lock-frontend`

El sistema está ejecutando actualizaciones automáticas en segundo plano. Espera unos
minutos a que terminen y vuelve a ejecutar el comando.

### Error: GPG key expired en `apt update`

La clave GPG del repositorio de ROS2 ha caducado. Renuévala con:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Error: `permission denied` al ejecutar Docker

El usuario no tiene permisos para usar Docker. Añádelo al grupo `docker`:
```bash
sudo usermod -aG docker $USER
newgrp docker
```
