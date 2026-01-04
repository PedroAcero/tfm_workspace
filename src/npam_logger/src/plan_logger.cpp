#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "moveit_msgs/msg/display_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fs = std::filesystem;

class PlannedLogger : public rclcpp::Node {
public:
  PlannedLogger() : Node("planned_logger") {
    // Declarar parámetro con valor por defecto
    this->declare_parameter<std::string>("nombre_archivo", "planned");

    // Obtener el parámetro
    std::string prefix = this->get_parameter("nombre_archivo").as_string();

    // Generar nombre de archivo con fecha y hora
    filename_ = prefix + "_" + getCurrentTimestamp() + ".csv";
    filepath_ = "/home/pedro/workspace/src/npam_logger/data/" + filename_;

    // Verificar si el archivo ya existe
    if (fs::exists(filepath_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "El archivo '%s' ya existe. Terminando programa.",
                   filepath_.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Archivo de destino: %s",
                filepath_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Esperando trayectoria planeada en /display_planned_path...");

    // Crear suscriptor
    subscription_ =
        this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
            "/display_planned_path", 10,
            std::bind(&PlannedLogger::trajectory_callback, this,
                      std::placeholders::_1));
  }

private:
  void trajectory_callback(
      const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "Trayectoria planeada recibida. Procesando...");

    if (msg->trajectory.empty()) {
      RCLCPP_WARN(this->get_logger(), "El mensaje no contiene trayectorias.");
      return;
    }

    // Obtener timestamp absoluto de recepción
    auto now = this->now();
    double timestamp_absolute = now.seconds();

    // Extraer la primera trayectoria
    const auto &robot_trajectory = msg->trajectory[0];
    const auto &joint_trajectory = robot_trajectory.joint_trajectory;

    RCLCPP_INFO(this->get_logger(), "Joints: %zu, Puntos: %zu",
                joint_trajectory.joint_names.size(),
                joint_trajectory.points.size());

    // Escribir al CSV
    writeToCSV(joint_trajectory, timestamp_absolute);

    RCLCPP_INFO(this->get_logger(), "Datos guardados exitosamente en: %s",
                filepath_.c_str());
  }

  void writeToCSV(const trajectory_msgs::msg::JointTrajectory &trajectory,
                  double timestamp_abs) {
    // Verificar nuevamente si el archivo existe
    if (fs::exists(filepath_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "El archivo '%s' ya existe. No se sobrescribirá.",
                   filepath_.c_str());
      return;
    }

    std::ofstream file(filepath_);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo crear el archivo: %s",
                   filepath_.c_str());
      return;
    }

    // Configurar precisión
    file << std::fixed << std::setprecision(9);

    // Escribir encabezado
    file << "timestamp_absolute,timestamp_relative";

    // Columnas de posición para cada joint
    for (const auto &joint_name : trajectory.joint_names) {
      file << "," << joint_name << "_pos";
    }

    // Columnas de velocidad para cada joint
    for (const auto &joint_name : trajectory.joint_names) {
      file << "," << joint_name << "_vel";
    }

    file << "\n";

    // Iterar sobre cada punto de la trayectoria
    for (const auto &point : trajectory.points) {
      // Calcular timestamp relativo desde el inicio
      double timestamp_relative =
          point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;

      // Calcular timestamp absoluto para este punto
      double point_timestamp_abs = timestamp_abs + timestamp_relative;

      // Escribir timestamps
      file << point_timestamp_abs << "," << timestamp_relative;

      // Escribir posiciones de todos los joints
      for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
        file << ",";
        if (i < point.positions.size()) {
          file << point.positions[i];
        } else {
          file << "0.0";
        }
      }

      // Escribir velocidades de todos los joints
      for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
        file << ",";
        if (i < point.velocities.size()) {
          file << point.velocities[i];
        } else {
          file << "0.0";
        }
      }

      file << "\n";
    }

    file.close();
  }

  std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
    localtime_r(&time_t_now, &tm_now);

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
  }

  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr
      subscription_;
  std::string filename_;
  std::string filepath_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannedLogger>());
  rclcpp::shutdown();
  return 0;
}