#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/action/move_group_sequence.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/motion_sequence_item.hpp"
#include "moveit_msgs/msg/motion_sequence_request.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "moveit_msgs/msg/position_constraint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

using namespace std::chrono_literals;
using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;

// ============================================================
// FUNCIONES AUXILIARES
// ============================================================

struct WaypointData {
  geometry_msgs::msg::Pose pose;
  double feedrate; // F value en mm/s (leído del G-code)
};

void normalizeQuaternion(geometry_msgs::msg::Quaternion &q) {
  // Esta funcion normaliza un cuaternión q dado
  double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm > 1e-6) {
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;
  } else {
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
  }
}

bool parseG1Line(const std::string &line, WaypointData &waypoint) {
  /*
   * Parsea una línea G1 extrayendo posición (X,Y,Z), orientación como
   * cuaternión (I,J,K,W) y feedrate (F).
   * F es tratado como opcional: si no aparece en la línea, se usa el
   * valor por defecto de 80.0 mm/s para mantener compatibilidad.
   */
  std::regex x_regex("X([+-]?[0-9]*\\.?[0-9]+)");
  std::regex y_regex("Y([+-]?[0-9]*\\.?[0-9]+)");
  std::regex z_regex("Z([+-]?[0-9]*\\.?[0-9]+)");
  std::regex i_regex("I([+-]?[0-9]*\\.?[0-9]+)");
  std::regex j_regex("J([+-]?[0-9]*\\.?[0-9]+)");
  std::regex k_regex("K([+-]?[0-9]*\\.?[0-9]+)");
  std::regex w_regex("W([+-]?[0-9]*\\.?[0-9]+)");
  std::regex f_regex("F([+-]?[0-9]*\\.?[0-9]+)");

  std::smatch match;
  auto &pose = waypoint.pose;

  if (std::regex_search(line, match, x_regex)) {
    pose.position.x = std::stod(match[1].str());
  } else {
    return false;
  }

  if (std::regex_search(line, match, y_regex)) {
    pose.position.y = std::stod(match[1].str());
  } else {
    return false;
  }

  if (std::regex_search(line, match, z_regex)) {
    pose.position.z = std::stod(match[1].str());
  } else {
    return false;
  }

  if (std::regex_search(line, match, i_regex)) {
    pose.orientation.x = std::stod(match[1].str());
  } else {
    return false;
  }

  if (std::regex_search(line, match, j_regex)) {
    pose.orientation.y = std::stod(match[1].str());
  } else {
    return false;
  }

  if (std::regex_search(line, match, k_regex)) {
    pose.orientation.z = std::stod(match[1].str());
  } else {
    return false;
  }

  if (std::regex_search(line, match, w_regex)) {
    pose.orientation.w = std::stod(match[1].str());
  } else {
    return false;
  }

  // F es opcional: si no está presente, se asigna valor por defecto
  if (std::regex_search(line, match, f_regex)) {
    waypoint.feedrate = std::stod(match[1].str());
  } else {
    waypoint.feedrate = 80.0;
  }

  return true;
}

std::vector<WaypointData> gcode_reader(const std::string &filename,
                                       rclcpp::Logger logger) {
  // Función para la lectura del G-Code
  // Retorna un vector de tipo paersonalizado WaypointData
  std::vector<WaypointData> waypoints;

  std::ifstream file(filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger, "No se pudo abrir el archivo: %s", filename.c_str());
    return waypoints;
  }

  RCLCPP_INFO(logger, "Leyendo G-code desde: %s", filename.c_str());

  std::string line;
  int line_number = 0;
  int waypoints_count = 0;

  while (std::getline(file, line)) {
    line_number++;

    // Eliminar espacios al inicio y final
    line.erase(0, line.find_first_not_of(" \t\r\n"));
    line.erase(line.find_last_not_of(" \t\r\n") + 1);

    // Buscar líneas que empiecen con G1
    if (line.find("G1") == 0) {
      WaypointData waypoint;

      if (parseG1Line(line, waypoint)) {
        normalizeQuaternion(waypoint.pose.orientation);
        waypoints.push_back(waypoint);
        waypoints_count++;

        if (waypoints_count % 50 == 0) {
          RCLCPP_INFO(logger, "Leídos %d waypoints...", waypoints_count);
        }
      } else {
        RCLCPP_WARN(logger, "Línea %d: No se pudo parsear completamente: %s",
                    line_number, line.c_str());
      }
    }
  }

  file.close();
  RCLCPP_INFO(logger, "Total de waypoints leídos: %zu", waypoints.size());

  // Estadísticas del feedrate leído
  if (!waypoints.empty()) {
    double f_min = waypoints[0].feedrate;
    double f_max = waypoints[0].feedrate;
    for (const auto &wp : waypoints) {
      f_min = std::min(f_min, wp.feedrate);
      f_max = std::max(f_max, wp.feedrate);
    }
    RCLCPP_INFO(logger, "Rango de feedrate F: [%.1f, %.1f] mm/s", f_min, f_max);
  }

  return waypoints;
}

moveit_msgs::msg::Constraints
buildPoseConstraints(const geometry_msgs::msg::Pose &pose,
                     const std::string &link_name,
                     const std::string &frame_id) {
  /*
   * Construye un mensaje Constraints de MoveIt a partir de una Pose.
   *
   * Pilz LIN necesita que el objetivo de cada segmento esté expresado
   * como un goal_constraint, no como una Pose directamente.
   * Esta función encapsula la pose en:
   *   - PositionConstraint: esfera de 0.1mm de tolerancia
   *   - OrientationConstraint: tolerancia de 0.01 rad en cada eje
   *
   * La tolerancia de posición de 0.1mm es coherente con la precisión
   * mecánica del UR10 (~0.05mm repeatability).
   */
  moveit_msgs::msg::Constraints constraints;

  // ---- Constraint de posición ----
  moveit_msgs::msg::PositionConstraint pos_constraint;
  pos_constraint.header.frame_id = frame_id;
  pos_constraint.link_name = link_name;
  pos_constraint.target_point_offset.x = 0.0;
  pos_constraint.target_point_offset.y = 0.0;
  pos_constraint.target_point_offset.z = 0.0;

  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sphere.dimensions.push_back(0.0001); // 0.1mm de tolerancia

  geometry_msgs::msg::Pose sphere_pose;
  sphere_pose.position = pose.position;
  sphere_pose.orientation.w = 1.0;

  pos_constraint.constraint_region.primitives.push_back(sphere);
  pos_constraint.constraint_region.primitive_poses.push_back(sphere_pose);
  pos_constraint.weight = 1.0;
  constraints.position_constraints.push_back(pos_constraint);

  // ---- Constraint de orientación ----
  moveit_msgs::msg::OrientationConstraint orient_constraint;
  orient_constraint.header.frame_id = frame_id;
  orient_constraint.link_name = link_name;
  orient_constraint.orientation = pose.orientation;
  orient_constraint.absolute_x_axis_tolerance = 0.01; // rad
  orient_constraint.absolute_y_axis_tolerance = 0.01;
  orient_constraint.absolute_z_axis_tolerance = 0.01;
  orient_constraint.weight = 1.0;
  constraints.orientation_constraints.push_back(orient_constraint);

  return constraints;
}

moveit_msgs::msg::MotionSequenceRequest buildSequenceRequest(
    const std::vector<WaypointData> &waypoints, const std::string &group_name,
    const std::string &end_effector_link, const std::string &planning_frame,
    double max_cartesian_speed_mmps, rclcpp::Logger logger) {
  /*
   * Construye un MotionSequenceRequest de Pilz a partir de los waypoints.
   *
   * Cada waypoint del G-code se convierte en un MotionSequenceItem con:
   *   - planner_id = "LIN": movimiento en línea recta cartesiana
   *   - pipeline_id = "pilz_industrial_motion_planner"
   *   - max_velocity_scaling_factor: derivado exclusivamente del F del G-code
   *     normalizado por max_cartesian_speed (medido empíricamente).
   *   - blend_radius = 0.0: el robot pasa EXACTAMENTE por cada punto,
   *     sin atajos geométricos. Clave para precisión de deposición.
   *
   * La conversión de feedrate a scaling_factor es:
   *   scaling = F_mm_s / max_cartesian_speed_mm_s
   *
   * Donde max_cartesian_speed es la velocidad media medida empíricamente
   * ejecutando la trayectoria sin restricción. Esto garantiza que la
   * velocidad de consigna en la meseta del perfil trapezoidal coincide
   * con el F del G-code.
   *
   * NOTA: Con miles de waypoints, este request puede ser computacionalmente
   * intenso. El servidor Pilz resolverá IK para cada segmento LIN.
   */
  moveit_msgs::msg::MotionSequenceRequest sequence_request;

  RCLCPP_INFO(logger,
              "Construyendo MotionSequenceRequest con %zu segmentos LIN...",
              waypoints.size());

  for (size_t i = 0; i < waypoints.size(); i++) {
    moveit_msgs::msg::MotionSequenceItem item;

    item.req.group_name = group_name;
    item.req.planner_id = "LIN";
    item.req.pipeline_id = "pilz_industrial_motion_planner";
    item.req.allowed_planning_time = 10.0;
    item.req.num_planning_attempts = 1;

    // Calcular escalado según el feedrate F leído
    double velocity_scaling = waypoints[i].feedrate / max_cartesian_speed_mmps;
    velocity_scaling = std::max(velocity_scaling, 0.001);
    velocity_scaling = std::min(velocity_scaling, 1.0);

    item.req.max_velocity_scaling_factor = velocity_scaling;
    item.req.max_acceleration_scaling_factor = velocity_scaling;

    // Goal constraint construido desde la pose del waypoint
    item.req.goal_constraints.push_back(buildPoseConstraints(
        waypoints[i].pose, end_effector_link, planning_frame));

    // blend_radius = 0.0 garantiza paso exacto por cada waypoint.
    // Estudiar: trade-off blend_radius vs. precisión geométrica.
    item.blend_radius = 0.0;

    sequence_request.items.push_back(item);
  }

  RCLCPP_INFO(logger, "MotionSequenceRequest construida: %zu ítems LIN",
              sequence_request.items.size());

  return sequence_request;
}

// ============================================================
// MAIN
// ============================================================

int main(int argc, char *argv[]) {
  // ============================================================
  // INICIALIZAR ROS Y EL NODO "NPAM_PILZ_PLANNER"
  // ============================================================
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("npam_pilz_planner");
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "Nodo NPAM Pilz iniciado!");

  // Declarar parámetros con valores por defecto
  //
  // NOTA sobre el flujo planteado:
  // Este valor NO es el límite hardware del robot (1000 mm/s para UR10).
  // Es la velocidad cartesiana media medida empíricamente ejecutando la
  // trayectoria sin ninguna restricción de velocidad.
  //
  // Flujo de calibración:
  //   1) Calcular velocidad aproximada de ejecución de trayectoria (p. ej, con
  //   cartesian_trajectory)
  //   2) Asignar ese valor medido max_cartesian_speed
  //   3) Escalar esa velocidad máxima como la leída en g-code dividido entre
  //   la max_cartesian_speed
  //
  // Valor inicial de 1000 mm/s antes de calibrar en paso 1).

  const std::string default_gcode_dir =
      std::string(std::getenv("HOME")) +
      "/workspace/src/npam_trajectory/trayectorias/";

  node->declare_parameter("gcode_directory", default_gcode_dir);
  node->declare_parameter("gcode_filename",
                          "short_generated_gcode_medio_estrella_poses.gcode");
  node->declare_parameter("max_cartesian_speed", 1000.0);

  // Obtener valores de los parámetros
  std::string gcode_directory =
      node->get_parameter("gcode_directory").as_string();
  std::string gcode_filename =
      node->get_parameter("gcode_filename").as_string();
  double max_cartesian_speed =
      node->get_parameter("max_cartesian_speed").as_double();

  std::string gcode_path = gcode_directory + gcode_filename;

  // Validar parámetros
  if (max_cartesian_speed <= 0.0) {
    RCLCPP_ERROR(logger,
                 "max_cartesian_speed debe ser mayor que 0 (valor: %.1f)",
                 max_cartesian_speed);
    rclcpp::shutdown();
    return 1;
  }

  // Verificar que el archivo existe antes de continuar
  std::ifstream test_file(gcode_path);
  if (!test_file.good()) {
    RCLCPP_ERROR(logger, "El archivo G-code no existe o no es accesible:");
    RCLCPP_ERROR(logger, "  Ruta: %s", gcode_path.c_str());
    RCLCPP_ERROR(logger, "  Directorio: %s", gcode_directory.c_str());
    RCLCPP_ERROR(logger, "  Archivo: %s", gcode_filename.c_str());
    rclcpp::shutdown();
    return 1;
  }
  test_file.close();

  RCLCPP_INFO(logger, "Parámetros cargados:");
  RCLCPP_INFO(logger, "  --> max_cartesian_speed: %.1f mm/s",
              max_cartesian_speed);
  RCLCPP_INFO(logger, "  --> gcode_path: %s", gcode_path.c_str());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // ============================================================
  // CONFIGURACIÓN DE MOVEGROUPINTERFACE
  // Igual que en cartesian_planner.cpp, pero aquí NO se configura
  // el pipeline de planificación en MoveGroupInterface directamente,
  // ya que la planificación Pilz se lanza vía MoveGroupSequence action.
  // MoveGroupInterface se usa únicamente para consultas de estado
  // y para la ejecución final de la trayectoria planificada.
  // ============================================================
  RCLCPP_INFO(logger, "Esperando a que /move_group esté listo...");
  rclcpp::sleep_for(1s);

  static const std::string PLANNING_GROUP = "ur_manipulator";
  RCLCPP_INFO(logger, "Conectando con /move_group...");
  moveit::planning_interface::MoveGroupInterface move_group(node,
                                                            PLANNING_GROUP);
  RCLCPP_INFO(logger, "MoveGroupInterface creado para el grupo: '%s'",
              PLANNING_GROUP.c_str());
  RCLCPP_INFO(logger, "Planning frame: %s",
              move_group.getPlanningFrame().c_str());

  // Configurar end-effector (mismo que cartesian_planner.cpp)
  move_group.setEndEffectorLink("cama_impresion");
  RCLCPP_INFO(logger, "End effector link: %s",
              move_group.getEndEffectorLink().c_str());

  move_group.setStartStateToCurrentState();

  const std::string planning_frame = move_group.getPlanningFrame();
  const std::string end_effector_link = move_group.getEndEffectorLink();

  // ============================================================
  // LECTURA Y VALIDACIÓN DEL G-CODE
  // La única diferencia con cartesian_planner.cpp es que gcode_reader
  // ahora retorna WaypointData (pose + feedrate F) en lugar de solo Pose.
  // ============================================================
  RCLCPP_INFO(logger, "Leyendo waypoints desde G-code...");
  std::vector<WaypointData> waypoints = gcode_reader(gcode_path, logger);

  if (waypoints.empty()) {
    RCLCPP_ERROR(logger, "No se pudieron leer waypoints del archivo G-code");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(logger, "Waypoints leídos: %zu puntos", waypoints.size());

  // ============================================================
  // PLANIFICACIÓN CON PILZ LIN (MoveGroupSequence Action)
  //
  // Este bloque reemplaza los dos bloques de cartesian_planner.cpp:
  //   [Planificación OMPL → computeCartesianPath]
  //   [Parametrización temporal IPTP → applyTimeParameterization]
  //
  // En Pilz, ambos pasos se unifican en un único planificador que:
  //   1. Interpola la línea recta cartesiana entre waypoints consecutivos
  //   2. Resuelve IK a lo largo del camino
  //   3. Calcula el perfil trapezoidal de velocidad respetando F del G-code
  //      y los límites cinemáticos de los joints simultáneamente
  //
  // La interfaz es la acción MoveGroupSequence, que recibe un
  // MotionSequenceRequest con tantos ítems LIN como waypoints tiene
  // el G-code, y devuelve la trayectoria completa ya parametrizada.
  //
  // plan_only = true → Pilz planifica pero NO ejecuta, igual que en
  // cartesian_planner.cpp donde separamos planificación de ejecución.
  // ============================================================

  // Construir el MotionSequenceRequest con un ítem LIN por waypoint
  moveit_msgs::msg::MotionSequenceRequest sequence_request =
      buildSequenceRequest(waypoints, PLANNING_GROUP, end_effector_link,
                           planning_frame, max_cartesian_speed, logger);

  // Crear el cliente de acción para el servidor Pilz
  RCLCPP_INFO(logger, "Creando cliente de acción /sequence_move_group...");
  auto action_client = rclcpp_action::create_client<MoveGroupSequence>(
      node, "sequence_move_group");

  if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(logger,
                 "Servidor de acción 'sequence_move_group' no disponible. "
                 "Asegúrate de que pilz_industrial_motion_planner está "
                 "cargado como plugin en move_group.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // Configurar el goal: solo planificar (plan_only = true)
  MoveGroupSequence::Goal goal;
  goal.request = sequence_request;
  goal.planning_options.plan_only = true;

  RCLCPP_INFO(logger, "Enviando MotionSequenceRequest al planificador Pilz...");
  RCLCPP_INFO(logger,
              "  AVISO: planificación de %zu segmentos LIN puede tardar "
              "varios segundos",
              waypoints.size());

  // Enviar goal y esperar resultado (el executor está en hilo separado)
  auto goal_handle_future = action_client->async_send_goal(goal);
  auto goal_handle = goal_handle_future.get();

  if (!goal_handle) {
    RCLCPP_ERROR(logger, "El servidor Pilz rechazó el goal");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(logger, "Goal aceptado por Pilz. Esperando resultado...");
  auto result_future = action_client->async_get_result(goal_handle);
  auto result = result_future.get();

  // Verificar resultado de la acción
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(logger, "Acción MoveGroupSequence fallida (código: %d)",
                 static_cast<int>(result.code));
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  if (result.result->response.error_code.val !=
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(logger, "Error MoveIt en planificación Pilz (código: %d)",
                 result.result->response.error_code.val);
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // Extraer la trayectoria planificada (Pilz devuelve la secuencia fusionada)
  if (result.result->response.planned_trajectories.empty()) {
    RCLCPP_ERROR(logger, "Pilz no devolvió ninguna trayectoria");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  moveit_msgs::msg::RobotTrajectory trajectory_msg =
      result.result->response.planned_trajectories[0];

  // Diagnóstico de la trayectoria planificada
  RCLCPP_INFO(logger, "Planificación Pilz LIN: COMPLETADA");
  RCLCPP_INFO(logger, "  - Puntos en trayectoria: %zu",
              trajectory_msg.joint_trajectory.points.size());

  if (!trajectory_msg.joint_trajectory.points.empty()) {
    auto &last_point = trajectory_msg.joint_trajectory.points.back();
    double total_duration = last_point.time_from_start.sec +
                            last_point.time_from_start.nanosec * 1e-9;
    RCLCPP_INFO(logger, "  - Duración total: %.2f segundos", total_duration);

    // Verificar que la trayectoria contiene velocidades (Pilz las incluye)
    if (!last_point.velocities.empty()) {
      RCLCPP_INFO(logger,
                  "  - Trayectoria contiene velocidades y aceleraciones");
    } else {
      RCLCPP_WARN(logger, "  - La trayectoria no contiene velocidades");
    }
  }

  RCLCPP_INFO(logger, "  - Tiempo de planificación: %.3f s",
              result.result->response.planning_time);

  // ============================================================
  // EJECUCIÓN
  // Idéntico a cartesian_planner.cpp: previsualización en RViz
  // durante 10 segundos antes de enviar al controlador real.
  // ============================================================
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory_msg;

  RCLCPP_INFO(logger, "Previsualizando, en 10s se ejecutará la trayectoria...");
  rclcpp::sleep_for(10s);

  RCLCPP_INFO(logger, "Ejecutando trayectoria Pilz LIN...");
  auto exec_result = move_group.execute(plan);

  if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(logger, "Trayectoria Pilz LIN ejecutada correctamente");
  } else {
    RCLCPP_ERROR(logger, "Error al ejecutar trayectoria (código: %d)",
                 exec_result.val);
  }

  // ============================================================
  // CLEANUP
  // ============================================================
  RCLCPP_INFO(logger, "Finalizando...");
  rclcpp::shutdown();
  spinner.join();

  return 0;
}