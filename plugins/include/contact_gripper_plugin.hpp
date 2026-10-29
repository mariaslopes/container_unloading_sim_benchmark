#ifndef CONTACT_GRIPPER_PLUGIN__CONTACT_GRIPPER_PLUGIN_HPP_
#define CONTACT_GRIPPER_PLUGIN__CONTACT_GRIPPER_PLUGIN_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

namespace contact_gripper_plugin
{
  class ContactGripperPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
  public:
    ContactGripperPlugin() = default;
    ~ContactGripperPlugin() override;

    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

  private:
    void GripCallback(const std_msgs::msg::Bool::SharedPtr _msg);

    gz::sim::Entity FindClosestLink(const gz::sim::EntityComponentManager &_ecm,
                                    double &_min_dist,
                                    std::string &_closest_name) const;
    void Attach(gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _child);
    void Detach(gz::sim::EntityComponentManager &_ecm, const std::chrono::steady_clock::duration &_now, bool _remove_object);

    // Simulation side (only touched from the simulation thread)
    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::sim::Entity gripper_link_{gz::sim::kNullEntity};
    gz::sim::Entity joint_entity_{gz::sim::kNullEntity};
    gz::sim::Entity attached_link_{gz::sim::kNullEntity};
    bool configured_{false};

    // Parameters
    std::string link_name_;
    std::string topic_name_;
    std::string robot_namespace_;
    double distance_threshold_{0.0};

    // Shared between the ROS thread and the simulation thread
    std::atomic<bool> grip_requested_{false};

    // ROS 2
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    std::thread spin_thread_;

    struct PendingRemoval
    {
      gz::sim::Entity model;
      std::chrono::steady_clock::duration remove_time;
      std::string name;
    };

    std::string attached_object_name_;
    gz::sim::Entity attached_model_{gz::sim::kNullEntity};
    bool remove_on_release_{false};
    std::chrono::steady_clock::duration remove_delay_{std::chrono::seconds(5)};
    std::vector<PendingRemoval> pending_removals_;
  };

} // namespace contact_gripper_plugin

#endif // CONTACT_GRIPPER_PLUGIN__CONTACT_GRIPPER_PLUGIN_HPP_