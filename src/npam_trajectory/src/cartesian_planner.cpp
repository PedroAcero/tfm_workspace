#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

void normalizeQuaternion(geometry_msgs::msg::Quaternion &q) {
  // Funcion que normaliza un cuaternión dado
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

bool parseG1Line(const std::string &line, geometry_msgs::msg::Pose &pose) {
  // Regex para extraer valores: X, Y, Z, I, J, K, W
  std::regex x_regex("X([+-]?[0-9]*\\.?[0-9]+)");
  std::regex y_regex("Y([+-]?[0-9]*\\.?[0-9]+)");
  std::regex z_regex("Z([+-]?[0-9]*\\.?[0-9]+)");
  std::regex i_regex("I([+-]?[0-9]*\\.?[0-9]+)");
  std::regex j_regex("J([+-]?[0-9]*\\.?[0-9]+)");
  std::regex k_regex("K([+-]?[0-9]*\\.?[0-9]+)");
  std::regex w_regex("W([+-]?[0-9]*\\.?[0-9]+)");

  std::smatch match;

  // Extraer posición (X, Y, Z)
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

  // Extraer orientación (I, J, K, W) - quaternion
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

  return true;
}

std::vector<geometry_msgs::msg::Pose> gcode_reader(const std::string &filename,
                                                   rclcpp::Logger logger) {
  // Funcion para la lectura del G-Code
  std::vector<geometry_msgs::msg::Pose> waypoints;

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
      geometry_msgs::msg::Pose pose;

      if (parseG1Line(line, pose)) {
        // Normalizar quaternion
        normalizeQuaternion(pose.orientation);
        waypoints.push_back(pose);
        waypoints_count++;

        // Mostrar cada 50 waypoints para no saturar el log
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

  return waypoints;
}

bool applyTimeParameterization(robot_trajectory::RobotTrajectory &robot_traj,
                               double velocity_scaling,
                               double acceleration_scaling,
                               rclcpp::Logger logger) {
  /**
   * Aplica parametrización temporal a una trayectoria usando el algoritmo
   * Iterative Parabolic Time Parameterization (IPTP).
   *
   * Este algoritmo calcula las velocidades, aceleraciones y tiempos para cada
   * punto de la trayectoria, respetando los límites configurados.
   *
   */

  RCLCPP_INFO(logger, "Aplicando parametrización temporal...");
  RCLCPP_INFO(logger, " --> Escalado de velocidad: %.0f%%",
              velocity_scaling * 100);
  RCLCPP_INFO(logger, "  --> Escalado de aceleración: %.0f%%",
              acceleration_scaling * 100);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool success = iptp.computeTimeStamps(robot_traj, velocity_scaling,
                                        acceleration_scaling);

  if (success) {
    RCLCPP_INFO(logger, "Parametrización temporal: COMPLETADA");

    // Mostrar duración total de la trayectoria
    double total_duration = robot_traj.getDuration();
    RCLCPP_INFO(logger, "  - Duración total: %.2f segundos", total_duration);
    RCLCPP_INFO(logger, "  - Número de waypoints: %zu",
                robot_traj.getWayPointCount());
  } else {
    RCLCPP_ERROR(logger, "Parametrización temporal: FALLADA");
  }

  return success;
}

int main(int argc, char *argv[]) {
  // ============================================================
  // INICIALIZAR ROS Y EL NODO "NPAM_CARTESIAN_PLANNER"
  // ============================================================
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("npam_cartesian_planner");
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "Nodo de NPAM iniciado!");

  // Declarar parámetros con valores por defecto
  node->declare_parameter("velocity_scaling", 1.0);
  node->declare_parameter("acceleration_scaling", 1.0);
  node->declare_parameter(
      "gcode_directory",
      "/home/pedro/workspace/src/npam_trajectory/trayectorias/");
  node->declare_parameter("gcode_filename",
                          "short_generated_gcode_medio_estrella_poses.gcode");

  // Obtener valores de los parámetros
  double velocity_scaling = node->get_parameter("velocity_scaling").as_double();
  double acceleration_scaling =
      node->get_parameter("acceleration_scaling").as_double();
  std::string gcode_directory =
      node->get_parameter("gcode_directory").as_string();
  std::string gcode_filename =
      node->get_parameter("gcode_filename").as_string();

  std::string gcode_path = gcode_directory + gcode_filename;

  // Validar parámetros
  if (velocity_scaling <= 0.0 || velocity_scaling > 1.0) {
    RCLCPP_ERROR(logger,
                 "velocity_scaling debe estar entre 0.0 y 1.0 (valor: %.2f)",
                 velocity_scaling);
    rclcpp::shutdown();
    return 1;
  }

  if (acceleration_scaling <= 0.0 || acceleration_scaling > 1.0) {
    RCLCPP_ERROR(
        logger, "acceleration_scaling debe estar entre 0.0 y 1.0 (valor: %.2f)",
        acceleration_scaling);
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Parámetros cargados:");
  RCLCPP_INFO(logger, "  --> velocity_scaling: %.2f (%.0f%%)", velocity_scaling,
              velocity_scaling * 100);
  RCLCPP_INFO(logger, "  --> acceleration_scaling: %.2f (%.0f%%)",
              acceleration_scaling, acceleration_scaling * 100);
  RCLCPP_INFO(logger, "  --> gcode_path: %s", gcode_path.c_str());

  // Verificar que el archivo existe antes de continuar
  std::ifstream test_file(gcode_path);
  if (!test_file.good()) {
    RCLCPP_ERROR(logger, "El archivo G-code no existe o no es accesible:");
    RCLCPP_ERROR(logger, "  Ruta: %s", gcode_path.c_str());
    RCLCPP_ERROR(logger, "  Directorio: %s", gcode_directory.c_str());
    RCLCPP_ERROR(logger, "  Archivo: %s", gcode_filename.c_str());
    RCLCPP_ERROR(logger,
                 "Verifica que la ruta sea correcta y que el archivo exista.");
    rclcpp::shutdown();
    return 1;
  }
  test_file.close();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // ============================================================
  // COMUNICACIÓN CON MOVE_GROUP A TRAVÉS DE MOVE GROUP INTERFACE
  // Esperar a que se establezcan las variables de entorno
  // Declarar planning_group (definido en el srdf)
  // Declarar end_effector
  // Establecer variables de planificación
  // ============================================================
  RCLCPP_INFO(logger, "Esperando a que /move_group esté listo...");
  rclcpp::sleep_for(1s);

  // Comunicación de MoveGroupInterface con /move_group
  static const std::string PLANNING_GROUP = "ur_manipulator";
  RCLCPP_INFO(logger, "Conectando con /move_group...");
  moveit::planning_interface::MoveGroupInterface move_group(node,
                                                            PLANNING_GROUP);
  RCLCPP_INFO(logger, "MoveGroupInterface creado para el grupo: '%s'",
              PLANNING_GROUP.c_str());
  RCLCPP_INFO(logger, "Planning frame: %s",
              move_group.getPlanningFrame().c_str());

  // Configurar end-effector
  move_group.setEndEffectorLink("cama_impresion");
  RCLCPP_INFO(logger, "End effector link: %s",
              move_group.getEndEffectorLink().c_str());

  // Configurar estado inicial al estado actual
  move_group.setStartStateToCurrentState();
  RCLCPP_INFO(logger, "Estado inicial configurado al estado actual");

  // Configurar escalados de velocidades y aceleraciones en MoveGroup
  // move_group.setMaxVelocityScalingFactor(velocity_scaling);
  // move_group.setMaxAccelerationScalingFactor(acceleration_scaling);

  // ============================================================
  // EXTRAER PUNTOS DEL G-CODE, Y VALIDARLOS
  // ============================================================
  RCLCPP_INFO(logger, "Leyendo waypoints desde G-code...");
  std::vector<geometry_msgs::msg::Pose> waypoints =
      gcode_reader(gcode_path, logger);

  if (waypoints.empty()) {
    RCLCPP_ERROR(logger, "No se pudieron leer waypoints del archivo G-code");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(logger, "Waypoints leídos: %zu puntos", waypoints.size());

  // Mostrar solo los primeros 5 waypoints para verificar
  int show_count = std::min(5, static_cast<int>(waypoints.size()));
  for (int i = 0; i < show_count; i++) {
    RCLCPP_INFO(
        logger,
        "Waypoint %d: pos[%.4f, %.4f, %.4f] orient[%.4f, %.4f, %.4f, %.4f]", i,
        waypoints[i].position.x, waypoints[i].position.y,
        waypoints[i].position.z, waypoints[i].orientation.x,
        waypoints[i].orientation.y, waypoints[i].orientation.z,
        waypoints[i].orientation.w);
  }

  if (waypoints.size() > 5) {
    RCLCPP_INFO(logger, "... (mostrando solo los primeros 5 de %zu waypoints)",
                waypoints.size());
  }

  // ============================================================
  // PLANIFICACIÓN DE LA TRAYECTORIA GEOMÉTRICA "trajectory_msg"
  // ============================================================
  RCLCPP_INFO(logger, "Calculando trayectoria cartesiana...");

  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  const double eef_step = 0.01;      // Resolución: 1cm entre waypoints
  const double jump_threshold = 0.0; // Deshabilitar detección de saltos

  double fraction = move_group.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory_msg);

  RCLCPP_INFO(logger,
              "Trayectoria cartesiana calculada (%.2f%% del camino logrado)",
              fraction * 100.0);

  if (fraction < 0.95) {
    RCLCPP_WARN(logger,
                "No se pudo completar toda la trayectoria (solo %.2f%%)",
                fraction * 100.0);
  }

  // ============================================================
  // AÑADIR PLANIFICACIÓN TEMPORAL
  // robot_traj = trajectory_msg (geom) + tiempos
  // ============================================================
  robot_trajectory::RobotTrajectory robot_traj(move_group.getRobotModel(),
                                               PLANNING_GROUP);

  robot_traj.setRobotTrajectoryMsg(*move_group.getCurrentState(),
                                   trajectory_msg);

  bool time_param_success = applyTimeParameterization(
      robot_traj, velocity_scaling, acceleration_scaling, logger);

  if (!time_param_success) {
    RCLCPP_ERROR(logger, "Error en la parametrización temporal. Abortando.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // Convertir de vuelta a mensaje
  robot_traj.getRobotTrajectoryMsg(trajectory_msg);

  // Verificar que ahora tiene velocidades
  if (!trajectory_msg.joint_trajectory.points.empty() &&
      !trajectory_msg.joint_trajectory.points[0].velocities.empty()) {
    RCLCPP_INFO(logger,
                "Trayectoria ahora contiene velocidades y aceleraciones");
  } else {
    RCLCPP_WARN(logger, "La trayectoria sigue sin velocidades");
  }

  // ============================================================
  // PLAN & EXCECUTE
  // ============================================================
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory_msg;

  RCLCPP_INFO(logger, "Previsualizando, en 10s se ejecutará la trayectoria...");
  rclcpp::sleep_for(10s);

  RCLCPP_INFO(logger, "Ejecutando trayectoria cartesiana...");

  auto exec_result = move_group.execute(plan);

  if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(logger, "Trayectoria cartesiana ejecutada correctamente");
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