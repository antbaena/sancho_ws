#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_behavior_tree/condition_node.hpp"
#include "people_msgs/msg/people.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace social_bt_nodes
{

class IsHumanBlocking : public nav2_behavior_tree::ConditionNode
{
public:
  IsHumanBlocking(const std::string & name, const BT::NodeConfiguration & config)
  : nav2_behavior_tree::ConditionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("is_human_blocking_node");
    people_sub_ = node_->create_subscription<people_msgs::msg::People>(
      "/people", rclcpp::QoS(10),
      std::bind(&IsHumanBlocking::peopleCallback, this, std::placeholders::_1));
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    rclcpp::spin_some(node_);

    for (const auto & person : people_)
    {
      double dx = person.position.x;
      double dy = person.position.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      // Verifica si la persona está dentro de 1.0 metro y en el frente del robot
      if (distance < 1.0 && std::abs(dy) < 0.5)
      {
        RCLCPP_INFO(node_->get_logger(), "Persona detectada bloqueando el camino.");
        return BT::NodeStatus::SUCCESS;
      }
    }

    return BT::NodeStatus::FAILURE;
  }

private:
  void peopleCallback(const people_msgs::msg::People::SharedPtr msg)
  {
    people_ = msg->people;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  std::vector<people_msgs::msg::Person> people_;
};

}  // namespace social_bt_nodes

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(social_bt_nodes::IsHumanBlocking, BT::ConditionNode)
