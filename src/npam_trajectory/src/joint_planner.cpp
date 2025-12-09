#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  // Inicializar ROS2
  rclcpp::init(argc, argv);

  // Crear nodo simple - sin cargar robot_description localmente
  // MoveGroupInterface usará el robot_model que ya tiene /move_group
  auto node = rclcpp::Node::make_shared("npam_joint_planner");
  auto logger = node->get_logger();

  RCLCPP_INFO(logger, "Nodo de NPAM iniciado!");

  // Crear executor en un thread separado
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // Dar tiempo a que /move_group esté completamente listo
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

  move_group.setEndEffectorLink("cama_impresion");

  RCLCPP_INFO(logger, "End effector link: %s",
              move_group.getEndEffectorLink().c_str());

  // Configurar objetivo en espacio de articulaciones
  std::vector<double> puntoA = {-2.155, -0.962, 0.663, -0.033, 1.624, 1.558};

  bool within_bounds = move_group.setJointValueTarget(puntoA);
  if (!within_bounds) {
    RCLCPP_WARN(logger,
                "Punto A fuera de límites, se truncará automáticamente");
  }

  // Planificar
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success =
      (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(logger, "Planificación: %s", success ? "ÉXITO" : "FALLÓ");

  // Ejecutar si la planificación tuvo éxito
  if (success) {
    auto exec_result = move_group.execute(plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "Trayectoria ejecutada correctamente");
    } else {
      RCLCPP_ERROR(logger, "Error al ejecutar (código: %d)", exec_result.val);
    }
  }

  // Cleanup
  rclcpp::shutdown();
  spinner.join();

  return 0;
}