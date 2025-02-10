// src/proxemics_layer.cpp

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <algorithm>

namespace proxemics_costmap_layer
{

class ProxemicsLayer : public nav2_costmap_2d::Layer
{
public:
  ProxemicsLayer() = default;

  // Se llama al inicializar la capa
  virtual void onInitialize() override
  {
    // Bloquear el puntero node_ para obtener un shared_ptr
    auto node = node_.lock();
    if (!node) {
      RCLCPP_ERROR(rclcpp::get_logger("ProxemicsLayer"), "No se pudo obtener el nodo.");
      return;
    }

    RCLCPP_INFO(node->get_logger(), "Inicializando ProxemicsLayer");

    // Declarar parámetros (usando rclcpp::ParameterValue)
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("ellipse_a", rclcpp::ParameterValue(0.5));
    declareParameter("ellipse_b", rclcpp::ParameterValue(0.3));


    // Obtener los parámetros desde el nodo
    node->get_parameter("ellipse_a", a_);
    node->get_parameter("ellipse_b", b_);

    enabled_ = true; // Habilitar la capa

    // Crear la suscripción al tópico "person_pose"
    person_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "person_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        person_x_ = msg->pose.position.x;
        person_y_ = msg->pose.position.y;
        person_received_ = true;
      });
  }

  // Actualiza los límites de la zona a modificar en el costmap
  virtual void updateBounds(double /*origin_x*/, double /*origin_y*/, double /*origin_yaw*/,
                            double* min_x, double* min_y, double* max_x, double* max_y) override
  {
    if (!enabled_ || !person_received_) return;


    double ellipse_min_x = person_x_ - a_;
    double ellipse_max_x = person_x_ + a_;
    double ellipse_min_y = person_y_ - b_;
    double ellipse_max_y = person_y_ + b_;

    *min_x = std::min(*min_x, ellipse_min_x);
    *min_y = std::min(*min_y, ellipse_min_y);
    *max_x = std::max(*max_x, ellipse_max_x);
    *max_y = std::max(*max_y, ellipse_max_y);
  }

  // Actualiza los costos del costmap asignando un costo letal a las celdas dentro de la elipse
  virtual void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                           int min_i, int min_j, int max_i, int max_j) override
  {
    if (!enabled_ || !person_received_) return;


    double wx, wy;
    for (int i = min_i; i < max_i; ++i)
    {
      for (int j = min_j; j < max_j; ++j)
      {
        // Convertir coordenadas de celda a coordenadas del mundo
        master_grid.mapToWorld(i, j, wx, wy);
        double dx = wx - person_x_;
        double dy = wy - person_y_;
        // Si la celda está dentro de la elipse (ecuación de la elipse normalizada)
        if ((dx * dx) / (a_ * a_) + (dy * dy) / (b_ * b_) <= 1.0)
        {
          master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }

  // Implementación mínima de los métodos virtuales puros
  virtual void reset() override {}
  virtual bool isClearable() override { return true; }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr person_sub_;
  bool person_received_{false};
  double person_x_{0.0};
  double person_y_{0.0};
  double a_{0.5};  // semi-eje mayor de la elipse
  double b_{0.3};  // semi-eje menor de la elipse
};

} // namespace proxemics_costmap_layer

// Macro para exportar la clase como plugin
PLUGINLIB_EXPORT_CLASS(proxemics_costmap_layer::ProxemicsLayer, nav2_costmap_2d::Layer)
