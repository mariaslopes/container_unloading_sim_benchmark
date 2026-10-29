#include "contact_gripper_plugin.hpp"

#include <cctype>
#include <limits>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Static.hh>

namespace contact_gripper_plugin
{
  ContactGripperPlugin::~ContactGripperPlugin()
  {
    if (executor_)
    {
      executor_->cancel();
    }
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
    if (executor_ && node_)
    {
      executor_->remove_node(node_);
    }
  }

  void ContactGripperPlugin::Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/)
  {
    gzmsg << "[contact_gripper] Loading contact_gripper_plugin" << std::endl;

    model_ = gz::sim::Model(_entity);
    if (!model_.Valid(_ecm))
    {
      gzerr << "[contact_gripper] Plugin must be attached to a model." << std::endl;
      return;
    }

    // ---- Parameters ----------------------------------------------------------
    if (_sdf->HasElement("robotNamespace"))
    {
      robot_namespace_ = _sdf->Get<std::string>("robotNamespace");
    }

    if (!_sdf->HasElement("bodyName"))
    {
      gzerr << "[contact_gripper] Missing <bodyName>, cannot proceed." << std::endl;
      return;
    }
    link_name_ = _sdf->Get<std::string>("bodyName");

    gripper_link_ = model_.LinkByName(_ecm, link_name_);
    if (gripper_link_ == gz::sim::kNullEntity)
    {
      std::string found;
      for (const auto &link : model_.Links(_ecm))
      {
        auto name = _ecm.Component<gz::sim::components::Name>(link);
        if (name)
        {
          found += " " + name->Data();
        }
      }
      gzerr << "[contact_gripper] Link named [" << link_name_ << "] does not exist.\n"
            << "  Check it exists and is not merged away by a fixed joint "
            << "(use <preserveFixedJoint> or attach to the parent link).\n"
            << "  Found links:" << found << std::endl;
      return;
    }

    if (!_sdf->HasElement("threshold"))
    {
      gzerr << "[contact_gripper] Missing <threshold>, cannot proceed." << std::endl;
      return;
    }
    distance_threshold_ = _sdf->Get<double>("threshold");

    if (!_sdf->HasElement("control_topic"))
    {
      gzerr << "[contact_gripper] Missing <control_topic>, cannot proceed." << std::endl;
      return;
    }
    topic_name_ = _sdf->Get<std::string>("control_topic");

    if (_sdf->HasElement("removeOnRelease"))
    {
      remove_on_release_ = _sdf->Get<bool>("removeOnRelease");
    }

    if (_sdf->HasElement("removeDelay"))
    {
      remove_delay_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(_sdf->Get<double>("removeDelay")));
    }

    // ---- ROS 2 ---------------------------------------------------------------
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    // Node names may only contain alphanumerics and '_'
    std::string node_name = "contact_gripper_" + model_.Name(_ecm);
    for (auto &c : node_name)
    {
      if (!std::isalnum(static_cast<unsigned char>(c)))
      {
        c = '_';
      }
    }
    node_ = std::make_shared<rclcpp::Node>(node_name, robot_namespace_);

    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        topic_name_, rclcpp::QoS(10),
        std::bind(&ContactGripperPlugin::GripCallback, this, std::placeholders::_1));
    gzmsg << "[contact_gripper] Listening for std_msgs/Bool on "
          << sub_->get_topic_name() << std::endl;

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]()
                               { executor_->spin(); });

    configured_ = true;
    gzmsg << "[contact_gripper] Loaded contact_gripper_plugin" << std::endl;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // ROS callbacks only record the requested state; the ECM is modified in
  // PreUpdate, because it must not be touched from the ROS thread.
  void ContactGripperPlugin::GripCallback(const std_msgs::msg::Bool::SharedPtr _msg)
  {
    grip_requested_ = _msg->data;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void ContactGripperPlugin::PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm)
  {
    if (!configured_)
    {
      return;
    }

    // Remove released objects once their delay has elapsed
    for (auto it = pending_removals_.begin(); it != pending_removals_.end();)
    {
      if (_info.simTime >= it->remove_time)
      {
        if (_ecm.HasEntity(it->model))
        {
          _ecm.RequestRemoveEntity(it->model, true);
          RCLCPP_INFO(node_->get_logger(), "Removed '%s' from the world",
                      it->name.c_str());
        }
        it = pending_removals_.erase(it);
      }
      else
      {
        ++it;
      }
    }

    // The held object may have been removed from the world
    if (joint_entity_ != gz::sim::kNullEntity && !_ecm.HasEntity(attached_link_))
    {
      Detach(_ecm, _info.simTime, false);
    }

    const bool want_grip = grip_requested_.load();
    const bool attached = (joint_entity_ != gz::sim::kNullEntity);

    if (want_grip && !attached)
    {
      double min_dist = 0.0;
      std::string closest_name;
      const gz::sim::Entity target = FindClosestLink(_ecm, min_dist, closest_name);
      if (target != gz::sim::kNullEntity)
      {
        RCLCPP_INFO(node_->get_logger(), "Closest object '%s' at %.3f m",
                    closest_name.c_str(), min_dist);
        Attach(_ecm, target);
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "No object within threshold, closest is '%s' at %.3f m (threshold %.3f m)",
                             closest_name.c_str(), min_dist, distance_threshold_);
      }
    }
    else if (!want_grip && attached)
    {
      Detach(_ecm, _info.simTime, remove_on_release_);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  gz::sim::Entity ContactGripperPlugin::FindClosestLink(
      const gz::sim::EntityComponentManager &_ecm,
      double &_min_dist,
      std::string &_closest_name) const
  {
    const auto gripper_pos = gz::sim::worldPose(gripper_link_, _ecm).Pos();

    gz::sim::Entity closest = gz::sim::kNullEntity;
    _min_dist = std::numeric_limits<double>::infinity();

    _ecm.Each<gz::sim::components::Link>(
        [&](const gz::sim::Entity &_link, const gz::sim::components::Link *) -> bool
        {
          const gz::sim::Entity top_model = gz::sim::topLevelModel(_link, _ecm);

          // Skip our own robot (including nested models)
          if (top_model == model_.Entity())
          {
            return true;
          }

          // Skip the ground plane and static models (they cannot be carried)
          auto name = _ecm.Component<gz::sim::components::Name>(top_model);
          if (name && name->Data() == "ground_plane")
          {
            return true;
          }
          auto is_static = _ecm.Component<gz::sim::components::Static>(top_model);
          if (is_static && is_static->Data())
          {
            return true;
          }

          // Skip objects that were released and are waiting to be removed
          for (const auto &pending : pending_removals_)
          {
            if (pending.model == top_model)
            {
              return true;
            }
          }

          const double dist =
              gripper_pos.Distance(gz::sim::worldPose(_link, _ecm).Pos());
          if (dist < _min_dist)
          {
            _min_dist = dist;
            closest = _link;
          }
          return true;
        });

    _closest_name = "unknown";
    if (closest != gz::sim::kNullEntity)
    {
      auto name = _ecm.Component<gz::sim::components::Name>(
          gz::sim::topLevelModel(closest, _ecm));
      if (name)
      {
        _closest_name = name->Data();
      }
    }

    // Report the closest object always, but only return it if it is in range
    return (_min_dist <= distance_threshold_) ? closest : gz::sim::kNullEntity;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void ContactGripperPlugin::Attach(
      gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _child)
  {
    // The Physics system turns this component into a fixed joint that keeps
    // the current relative pose between the two links.
    joint_entity_ = _ecm.CreateEntity();
    _ecm.CreateComponent(
        joint_entity_,
        gz::sim::components::DetachableJoint({gripper_link_, _child, "fixed"}));
    attached_link_ = _child;
    attached_model_ = gz::sim::topLevelModel(_child, _ecm);

    attached_object_name_ = "unknown";
    auto name = _ecm.Component<gz::sim::components::Name>(attached_model_);
    if (name)
    {
      attached_object_name_ = name->Data();
    }

    RCLCPP_INFO(node_->get_logger(), "Gripper ON: attached to '%s'",
                attached_object_name_.c_str());
  }

  ////////////////////////////////////////////////////////////////////////////////
  void ContactGripperPlugin::Detach(
      gz::sim::EntityComponentManager &_ecm,
      const std::chrono::steady_clock::duration &_now,
      bool _remove_object)
  {
    // Release the joint now, the object is removed later in PreUpdate
    _ecm.RequestRemoveEntity(joint_entity_);

    RCLCPP_INFO(node_->get_logger(), "Gripper OFF: released '%s'",
                attached_object_name_.c_str());

    if (_remove_object && attached_model_ != gz::sim::kNullEntity &&
        _ecm.HasEntity(attached_model_))
    {
      pending_removals_.push_back(
          {attached_model_, _now + remove_delay_, attached_object_name_});
      RCLCPP_INFO(node_->get_logger(), "'%s' will be removed in %.1f s",
                  attached_object_name_.c_str(),
                  std::chrono::duration<double>(remove_delay_).count());
    }

    joint_entity_ = gz::sim::kNullEntity;
    attached_link_ = gz::sim::kNullEntity;
    attached_model_ = gz::sim::kNullEntity;
    attached_object_name_.clear();
  }

} // namespace contact_gripper_plugin

GZ_ADD_PLUGIN(
    contact_gripper_plugin::ContactGripperPlugin,
    gz::sim::System,
    contact_gripper_plugin::ContactGripperPlugin::ISystemConfigure,
    contact_gripper_plugin::ContactGripperPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    contact_gripper_plugin::ContactGripperPlugin,
    "contact_gripper_plugin::ContactGripperPlugin")