#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fs = std::filesystem;

class CommandedLogger : public rclcpp::Node {
public:
  CommandedLogger() : Node("commanded_logger") {
    // Declarar parámetro con valor por defecto
    this->declare_parameter<std::string>("filename_prefix", "commanded");

    // Obtener el parámetro
    std::string prefix = this->get_parameter("filename_prefix").as_string();

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
    RCLCPP_INFO(this->get_logger(), "Esperando ejecución de trayectoria...");

    // Crear suscriptor al estado del controlador
    subscription_ = this->create_subscription<
        control_msgs::msg::JointTrajectoryControllerState>(
        "/scaled_joint_trajectory_controller/state", 10,
        std::bind(&CommandedLogger::state_callback, this,
                  std::placeholders::_1));

    // Variables de control
    last_sample_time_ = this->now();
    sample_interval_ns_ = 500000000; // 0.5 segundos
  }

  ~CommandedLogger() {
    if (!data_buffer_.empty()) {
      writeToCSV();
    }
  }

private:
  struct DataPoint {
    double timestamp_relative;
    std::vector<std::string> joint_names;
    std::vector<double> desired_positions;
    std::vector<double> desired_velocities;
    std::vector<double> actual_positions;
    std::vector<double> actual_velocities;
  };

  std::string filename_;
  std::string filepath_;
  std::vector<DataPoint> data_buffer_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::
      SharedPtr subscription_;
  bool recording_ = false;
  rclcpp::Time start_time_;
  rclcpp::Time last_sample_time_;
  int64_t sample_interval_ns_;
  DataPoint last_data_point_;
  int stationary_count_ = 0;
  const int STATIONARY_THRESHOLD = 20;

  void state_callback(
      const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
    // Detectar si hay movimiento
    bool is_moving = checkIfMoving(msg->desired.velocities);

    // Estado: WAITING -> RECORDING
    if (!recording_ && is_moving) {
      RCLCPP_INFO(this->get_logger(),
                  "Movimiento detectado. Iniciando grabación...");
      recording_ = true;
      start_time_ = this->now();
      last_sample_time_ = this->now();
      data_buffer_.clear();
      stationary_count_ = 0;
    }

    // Estado: RECORDING
    if (recording_) {
      auto now = this->now();
      auto time_since_last_sample = now - last_sample_time_;

      // Verificar si el desired ha cambiado significativamente
      bool desired_changed = hasDesiredChanged(msg);

      // Capturar si ha pasado el intervalo O si desired cambió
      if (time_since_last_sample.nanoseconds() >= sample_interval_ns_ ||
          desired_changed) {
        DataPoint dp;
        dp.timestamp_relative = (now - start_time_).seconds();
        dp.joint_names = msg->joint_names;
        dp.desired_positions = msg->desired.positions;
        dp.desired_velocities = msg->desired.velocities;
        dp.actual_positions = msg->actual.positions;
        dp.actual_velocities = msg->actual.velocities;

        data_buffer_.push_back(dp);
        last_data_point_ = dp;
        last_sample_time_ = now;

        RCLCPP_DEBUG(this->get_logger(), "Muestra capturada: t=%.3f s",
                     dp.timestamp_relative);
      }

      // Detectar fin del movimiento
      if (!is_moving) {
        stationary_count_++;
        if (stationary_count_ >= STATIONARY_THRESHOLD) {
          RCLCPP_INFO(this->get_logger(),
                      "Movimiento finalizado. Deteniendo grabación...");
          recording_ = false;
          writeToCSV();
          RCLCPP_INFO(this->get_logger(), "Total de muestras capturadas: %zu",
                      data_buffer_.size());
        }
      } else {
        stationary_count_ = 0;
      }
    }
  }

  bool checkIfMoving(const std::vector<double> &velocities) {
    const double velocity_threshold = 0.001; // rad/s

    for (const auto &vel : velocities) {
      if (std::abs(vel) > velocity_threshold) {
        return true;
      }
    }
    return false;
  }

  bool hasDesiredChanged(
      const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
    if (last_data_point_.desired_positions.empty())
      return true;

    const double position_threshold = 0.001; // rad

    for (size_t i = 0; i < msg->desired.positions.size(); ++i) {
      if (i >= last_data_point_.desired_positions.size())
        return true;

      double diff = std::abs(msg->desired.positions[i] -
                             last_data_point_.desired_positions[i]);
      if (diff > position_threshold)
        return true;
    }

    return false;
  }

  void writeToCSV() {
    if (data_buffer_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No hay datos para guardar.");
      return;
    }

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

    // Obtener nombres de joints del primer punto
    const auto &joint_names = data_buffer_[0].joint_names;

    // Escribir encabezado
    file << "timestamp_relative";
    for (const auto &name : joint_names) {
      file << "," << name << "_desired_pos";
    }
    for (const auto &name : joint_names) {
      file << "," << name << "_actual_pos";
    }
    for (const auto &name : joint_names) {
      file << "," << name << "_desired_vel";
    }
    for (const auto &name : joint_names) {
      file << "," << name << "_actual_vel";
    }
    file << "\n";

    // Configurar precisión
    file << std::fixed << std::setprecision(9);

    // Escribir datos
    for (const auto &dp : data_buffer_) {
      file << dp.timestamp_relative;

      // Posiciones deseadas
      for (const auto &pos : dp.desired_positions) {
        file << "," << pos;
      }

      // Posiciones reales
      for (const auto &pos : dp.actual_positions) {
        file << "," << pos;
      }

      // Velocidades deseadas
      for (const auto &vel : dp.desired_velocities) {
        file << "," << vel;
      }

      // Velocidades reales
      for (const auto &vel : dp.actual_velocities) {
        file << "," << vel;
      }

      file << "\n";
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Archivo guardado exitosamente: %s",
                filepath_.c_str());
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
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandedLogger>());
  rclcpp::shutdown();
  return 0;
}