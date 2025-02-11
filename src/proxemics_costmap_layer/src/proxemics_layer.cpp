#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
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

    // Normalmente, Nav2 te da un tf_buffer_ en Costmap2DROS. Puedes usarlo así:
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declarar parámetros
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("ellipse_a", rclcpp::ParameterValue(0.5));
    declareParameter("ellipse_b", rclcpp::ParameterValue(0.3));
    declareParameter("obstacle_cost", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));
    declareParameter("person_topic", rclcpp::ParameterValue(std::string("/person_pose")));
    declareParameter("transform_tolerance", rclcpp::ParameterValue(0.1)); // seg

    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "ellipse_a", a_);
    node->get_parameter(name_ + "." + "ellipse_b", b_);
    int cost_i;
    node->get_parameter(name_ + "." + "obstacle_cost", cost_i);
    lethal_cost_ = static_cast<unsigned char>(cost_i);
    node->get_parameter(name_ + "." + "person_topic", person_topic_);
    node->get_parameter(name_ + "." + "transform_tolerance", transform_tolerance_);

    // Suscripción a la pose de la persona
    person_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      person_topic_, 10,
      std::bind(&ProxemicsLayer::personCallback, this, std::placeholders::_1));
  }

  void personCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    auto node = node_.lock();
    if (!node) return;

    // Transformar la pose a nuestro frame global del costmap
    // (p. ej. "map"), que se obtiene con layered_costmap_->getGlobalFrameID()
    geometry_msgs::msg::PoseStamped pose_in = *msg;
    geometry_msgs::msg::PoseStamped pose_out;
    std::string global_frame = layered_costmap_->getGlobalFrameID();

    try {
      tf_buffer_->transform(
        pose_in,
        pose_out,
        global_frame,
        tf2::durationFromSec(transform_tolerance_));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(node->get_logger(), "No se pudo transformar la pose de la persona: %s", ex.what());
      return;
    }

    // Guardar la posición transformada
    person_x_ = (pose_out.pose.position.x)/1000;
    person_y_ = (pose_out.pose.position.y)/1000;
    person_received_ = true;
    // RCLCPP_INFO(rclcpp::get_logger("ProxemicsLayer"), "Persona en (%.2f, %.2f)", person_x_, person_y_);

  }

  void updateBounds(double /*origin_x*/, double /*origin_y*/, double /*origin_yaw*/,
                    double* min_x, double* min_y, double* max_x, double* max_y) override
  {
    if (!enabled_ || !person_received_) {
      return;
    }
    // Extremos del bounding box de la elipse
    double ellipse_min_x = person_x_ - a_;
    double ellipse_max_x = person_x_ + a_;
    double ellipse_min_y = person_y_ - b_;
    double ellipse_max_y = person_y_ + b_;

    *min_x = std::min(*min_x, ellipse_min_x);
    *min_y = std::min(*min_y, ellipse_min_y);
    *max_x = std::max(*max_x, ellipse_max_x);
    *max_y = std::max(*max_y, ellipse_max_y);
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j, int max_i, int max_j) override
  {
    if (!enabled_ || !person_received_) {
      return;
    }

    double wx, wy;
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        master_grid.mapToWorld(i, j, wx, wy);

        double dx = wx - person_x_;
        double dy = wy - person_y_;
        // Ecuación de la elipse: (x^2 / a^2) + (y^2 / b^2) <= 1
        if ((dx * dx) / (a_ * a_) + (dy * dy) / (b_ * b_) <= 1.0) {
          // Ponle coste letal (o el que definas en obstacle_cost)
          master_grid.setCost(i, j, lethal_cost_);
        }
      }
    }
  }

  void reset() override {}
  bool isClearable() override { return true; }

private:
  void reconfigureCB() { /* Ejemplo de callback dinámico, si lo quisieras... */ }

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subs
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr person_sub_;
  std::string person_topic_{"/person_pose"};

  // Flags y datos
  bool person_received_{false};
  double person_x_{0.0};
  double person_y_{0.0};

  // Parámetros
  bool enabled_{true};
  double a_{0.5};
  double b_{0.3};
  double transform_tolerance_{0.1};
  unsigned char lethal_cost_{nav2_costmap_2d::LETHAL_OBSTACLE};
};

} // namespace proxemics_costmap_layer

PLUGINLIB_EXPORT_CLASS(proxemics_costmap_layer::ProxemicsLayer, nav2_costmap_2d::Layer)
