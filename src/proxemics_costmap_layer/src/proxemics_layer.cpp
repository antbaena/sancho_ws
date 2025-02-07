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

class ProxemicsLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  ProxemicsLayer() {}

  // Se llama cuando se inicializa la capa
  virtual void onInitialize() override
  {
    RCLCPP_INFO(getLogger(), "Initializing ProxemicsLayer");

    // Se suscribe al tópico 'person_pose' para obtener la posición de la persona
    person_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "person_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        person_x_ = msg->pose.position.x;
        person_y_ = msg->pose.position.y;
        person_received_ = true;
      });

    // Se leen parámetros para definir el tamaño de la elipse
    declareParameter("ellipse_a", 0.5);  // semi-eje mayor [m]
    declareParameter("ellipse_b", 0.3);  // semi-eje menor [m]
    a_ = getParameter("ellipse_a").as_double();
    b_ = getParameter("ellipse_b").as_double();
  }

  // Actualiza los límites de la región que se modificará en el costmap
  virtual void updateBounds(double /*origin_x*/, double /*origin_y*/, double /*origin_yaw*/,
                            double* min_x, double* min_y, double* max_x, double* max_y) override
  {
    if (!person_received_) return;

    double ellipse_min_x = person_x_ - a_;
    double ellipse_max_x = person_x_ + a_;
    double ellipse_min_y = person_y_ - b_;
    double ellipse_max_y = person_y_ + b_;

    *min_x = std::min(*min_x, ellipse_min_x);
    *min_y = std::min(*min_y, ellipse_min_y);
    *max_x = std::max(*max_x, ellipse_max_x);
    *max_y = std::max(*max_y, ellipse_max_y);
  }

  // Actualiza los costos del costmap asignando un alto costo (obstáculo) dentro de la elipse
  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                           int min_i, int min_j, int max_i, int max_j) override
  {
    if (!person_received_) return;

    double wx, wy;
    for (int i = min_i; i < max_i; ++i)
    {
      for (int j = min_j; j < max_j; ++j)
      {
        // Convertir coordenadas de celda a coordenadas del mundo
        master_grid.mapToWorld(i, j, wx, wy);
        double dx = wx - person_x_;
        double dy = wy - person_y_;
        // Si la celda está dentro de la elipse definida, se asigna un costo letal
        if ((dx * dx) / (a_ * a_) + (dy * dy) / (b_ * b_) <= 1.0)
        {
          master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr person_sub_;
  bool person_received_{false};
  double person_x_{0.0};
  double person_y_{0.0};
  double a_{0.5};  // semi-eje mayor de la elipse
  double b_{0.3};  // semi-eje menor de la elipse
};

}  // namespace proxemics_costmap_layer

// Macro para exportar la clase como plugin
PLUGINLIB_EXPORT_CLASS(proxemics_costmap_layer::ProxemicsLayer, nav2_costmap_2d::Layer)
