#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace proxemics_costmap_layer
{

class ProxemicsLayer : public nav2_costmap_2d::Layer
{
public:
  ProxemicsLayer() = default;

  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node) {
      RCLCPP_ERROR(rclcpp::get_logger("ProxemicsLayer"), "No se pudo obtener el nodo.");
      return;
    }

    RCLCPP_INFO(node->get_logger(), "Inicializando ProxemicsLayer");

    // Obtiene el TFBuffer desde la layered_costmap (ya gestionado por Nav2)
    tf_buffer_ = layered_costmap_->getTFBuffer();
    if (!tf_buffer_) {
      RCLCPP_ERROR(node->get_logger(), "No se pudo obtener el TFBuffer de la capa.");
    }

    // Declarar parámetros (prefijados con el nombre del plugin si es necesario)
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("ellipse_a", rclcpp::ParameterValue(0.5));
    declareParameter("ellipse_b", rclcpp::ParameterValue(0.3));
    declareParameter("obstacle_cost", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));
    declareParameter("person_topic", rclcpp::ParameterValue(std::string("/human_pose/persons_poses")));
    declareParameter("transform_tolerance", rclcpp::ParameterValue(0.1)); // en segundos

    // Leer valores de parámetros
    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "ellipse_a", a_);
    node->get_parameter(name_ + "." + "ellipse_b", b_);
    int cost_i;
    node->get_parameter(name_ + "." + "obstacle_cost", cost_i);
    lethal_cost_ = static_cast<unsigned char>(cost_i);
    node->get_parameter(name_ + "." + "person_topic", person_topic_);
    node->get_parameter(name_ + "." + "transform_tolerance", transform_tolerance_);

    // Suscripción a la PoseArray de personas
    person_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
      person_topic_, rclcpp::QoS(10),
      std::bind(&ProxemicsLayer::personArrayCallback, this, std::placeholders::_1));

    // Marca la capa como actualizable
    current_ = true;
  }

  // Callback que recibe un PoseArray y actualiza la lista de poses de las personas
  void personArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    auto node = node_.lock();
    if (!node || !tf_buffer_) {
      return;
    }

    // Frame global configurado en el costmap (por ejemplo, "map")
    std::string global_frame = layered_costmap_->getGlobalFrameID();

    // Vector temporal para almacenar las poses transformadas
    std::vector<geometry_msgs::msg::PoseStamped> transformed_poses;
    for (const auto & pose : msg->poses) {
      geometry_msgs::msg::PoseStamped pose_in;
      pose_in.header = msg->header;  // Asumimos que todas las poses comparten el mismo header
      pose_in.pose = pose;
      geometry_msgs::msg::PoseStamped pose_out;
      try {
        pose_out = tf_buffer_->transform(
          pose_in,
          global_frame,
          tf2::durationFromSec(transform_tolerance_));
        transformed_poses.push_back(pose_out);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(node->get_logger(), "No se pudo transformar una de las poses: %s", ex.what());
        continue;
      }
    }

    // Actualizamos la lista de poses
    persons_poses_ = transformed_poses;
  }

  // Ajusta los límites para actualizar la zona donde se dibujan las elipses de cada persona
  void updateBounds(double /*origin_x*/, double /*origin_y*/, double /*origin_yaw*/,
                    double* min_x, double* min_y, double* max_x, double* max_y) override
  {
    if (!enabled_ || persons_poses_.empty()) {
      return;
    }

    // Itera sobre cada pose para calcular el bounding box
    for (const auto & pose_stamped : persons_poses_) {
      double px = pose_stamped.pose.position.x;
      double py = pose_stamped.pose.position.y;
      double ellipse_min_x = px - a_;
      double ellipse_max_x = px + a_;
      double ellipse_min_y = py - b_;
      double ellipse_max_y = py + b_;

      *min_x = std::min(*min_x, ellipse_min_x);
      *min_y = std::min(*min_y, ellipse_min_y);
      *max_x = std::max(*max_x, ellipse_max_x);
      *max_y = std::max(*max_y, ellipse_max_y);
    }
  }

  // Aplica el costo letal dentro de las elipses correspondientes a cada persona
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j, int max_i, int max_j) override
  {
    if (!enabled_ || persons_poses_.empty()) {
      return;
    }

    double wx, wy;
    // Recorre la región actualizada
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        master_grid.mapToWorld(i, j, wx, wy);

        // Verifica para cada persona si el punto está dentro de la elipse
        for (const auto & pose_stamped : persons_poses_) {
          double dx = wx - pose_stamped.pose.position.x;
          double dy = wy - pose_stamped.pose.position.y;
          // Ecuación de la elipse: (x²/a² + y²/b²) <= 1
          if ((dx * dx) / (a_ * a_) + (dy * dy) / (b_ * b_) <= 1.0) {
            master_grid.setCost(i, j, lethal_cost_);
            break;  // Si se cumple para alguna persona, no es necesario comprobar las demás
          }
        }
      }
    }
  }

  // Método reset (no se necesita implementación en este caso)
  void reset() override {}

  // Indica que la capa es "clearable" (se puede limpiar)
  bool isClearable() override { return true; }

private:
  // TF buffer (obtenido de Nav2)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Suscripción a la PoseArray de personas
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr person_sub_;
  std::string person_topic_{"/human_pose/persons_poses"};

  // Lista de poses transformadas de las personas
  std::vector<geometry_msgs::msg::PoseStamped> persons_poses_;

  // Parámetros
  bool enabled_{true};
  double a_{0.5};  // Semieje mayor de la elipse
  double b_{0.3};  // Semieje menor de la elipse
  double transform_tolerance_{0.1};
  unsigned char lethal_cost_{nav2_costmap_2d::LETHAL_OBSTACLE};
};

}  // namespace proxemics_costmap_layer

// Exportar la clase para pluginlib
PLUGINLIB_EXPORT_CLASS(proxemics_costmap_layer::ProxemicsLayer, nav2_costmap_2d::Layer)
