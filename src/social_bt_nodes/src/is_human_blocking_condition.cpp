#include <memory>
#include <string>
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "people_msgs/msg/people.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace nav2_behavior_tree
{

class IsHumanInPathCondition : public BT::ConditionNode
{
public:
  IsHumanInPathCondition(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf),
    node_(rclcpp::Node::make_shared("is_human_in_path_bt_node")),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {
    people_sub_ = node_->create_subscription<people_msgs::msg::People>(
      "/people", 10,
      std::bind(&IsHumanInPathCondition::peopleCallback, this, std::placeholders::_1));
  }

  static BT::PortsList providedPorts()
  {
    return { };
  }

  void peopleCallback(const people_msgs::msg::People::SharedPtr msg)
  {
    latest_people_msg_ = msg;
  }

  BT::NodeStatus tick() override
  {
    if (!latest_people_msg_) {
      // No hemos recibido todavía el mensaje
      return BT::NodeStatus::FAILURE;
    }

    // Obtener la pose actual del robot
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "No TF disponible: %s", ex.what());
      return BT::NodeStatus::FAILURE;
    }

    double robot_x = transform.transform.translation.x;
    double robot_y = transform.transform.translation.y;

    // Comprobamos cada persona
    for (const auto & person : latest_people_msg_->people) {
      double dx = person.position.x - robot_x;
      double dy = person.position.y - robot_y;
      double distance = std::hypot(dx, dy);

      // Opcional: filtrar personas muy lejanas
      if (distance > 2.0) {  // sólo mirar personas en un radio de 2 metros
        continue;
      }

      // Simple: considerar "en el camino" si está en frente del robot
      if (dx > 0 && std::abs(dy) < 0.5) {
        // Persona en un cono estrecho al frente (dx positivo, dy pequeño)
        return BT::NodeStatus::SUCCESS;
      }
    }

    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  people_msgs::msg::People::SharedPtr latest_people_msg_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace nav2_behavior_tree

// Registrar el nodo
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsHumanInPathCondition>("IsHumanInPath");
}
