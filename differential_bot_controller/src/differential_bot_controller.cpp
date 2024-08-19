#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "differential_bot_controller/differential_bot_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel"; // stamped velocity topic
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
}  // namespace

namespace differential_bot_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

DifferentialBotController::DifferentialBotController() : controller_interface::ControllerInterface() {}

const char * DifferentialBotController::feedback_type() const
{
  return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

controller_interface::CallbackReturn DifferentialBotController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration DifferentialBotController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.left_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.right_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration DifferentialBotController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.left_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + feedback_type());
  }
  for (const auto & joint_name : params_.right_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + feedback_type());
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type DifferentialBotController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  auto logger = get_node()->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = time - last_command_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  Twist command = *last_command_msg;
  double & linear_command = command.twist.linear.x;
  double & angular_command = command.twist.angular.z;

  previous_update_timestamp_ = time;

  // Apply (possibly new) multipliers:
  const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
  const double left_wheel_radius = params_.left_wheel_radius_multiplier * params_.wheel_radius;
  const double right_wheel_radius = params_.right_wheel_radius_multiplier * params_.wheel_radius;

  // Compute wheels velocities:
  const double velocity_left =
    (linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
  const double velocity_right =
    (linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius; 

  // Set wheels velocities:
  for (size_t index = 0; index < static_cast<size_t>(params_.wheels_per_side); ++index)
  {
    registered_left_wheel_handles_[index].velocity.get().set_value(velocity_left);
    registered_right_wheel_handles_[index].velocity.get().set_value(velocity_right);
  } 

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn DifferentialBotController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  if (params_.left_wheel_names.size() != params_.right_wheel_names.size())
  {
    RCLCPP_ERROR(
      logger, "The number of left wheels [%zu] and the number of right wheels [%zu] are different",
      params_.left_wheel_names.size(), params_.right_wheel_names.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.left_wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "Wheel names parameters are empty!");
    return controller_interface::CallbackReturn::ERROR;
  }


  cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};
  // publish_limited_velocity_ = params_.publish_limited_velocity;
  use_stamped_vel_ = params_.use_stamped_vel;


  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // left and right sides are both equal at this point
  params_.wheels_per_side = params_.left_wheel_names.size();

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void
      {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(
            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
      });
  }
  else
  {
    velocity_command_unstamped_subscriber_ =
      get_node()->create_subscription<geometry_msgs::msg::Twist>(
        DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
        {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(
              get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }

          // Write fake header in the stored stamped command
          std::shared_ptr<Twist> twist_stamped;
          received_velocity_msg_ptr_.get(twist_stamped);
          twist_stamped->twist = *msg;
          twist_stamped->header.stamp = get_node()->get_clock()->now();
        });
  }


  std::string controller_namespace = std::string(get_node()->get_namespace());

  if (controller_namespace == "/")
  {
    controller_namespace = "";
  }
  else
  {
    controller_namespace = controller_namespace.erase(0, 1) + "/";
  }


  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DifferentialBotController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto left_result =
    configure_side("left", params_.left_wheel_names, registered_left_wheel_handles_);
  const auto right_result =
    configure_side("right", params_.right_wheel_names, registered_right_wheel_handles_);

  if (
    left_result == controller_interface::CallbackReturn::ERROR ||
    right_result == controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (registered_left_wheel_handles_.empty() || registered_right_wheel_handles_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Either left wheel interfaces, right wheel interfaces are non existent");
    return controller_interface::CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DifferentialBotController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  if (!is_halted)
  {
    halt();
    is_halted = true;
  }
  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DifferentialBotController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DifferentialBotController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool DifferentialBotController::reset()
{
  //odometry_.resetOdometry();

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

controller_interface::CallbackReturn DifferentialBotController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void DifferentialBotController::halt()
{
  const auto halt_wheels = [](auto & wheel_handles)
  {
    for (const auto & wheel_handle : wheel_handles)
    {
      wheel_handle.velocity.get().set_value(0.0);
    }
  };

  halt_wheels(registered_left_wheel_handles_);
  halt_wheels(registered_right_wheel_handles_);
}

controller_interface::CallbackReturn DifferentialBotController::configure_side(
  const std::string & side, const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "No '%s' wheel names specified", side.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel_name : wheel_names)
  {
    const auto interface_name = feedback_type();
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&wheel_name, &interface_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }

  return controller_interface::CallbackReturn::SUCCESS;
}
}  // namespace differential_bot_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  differential_bot_controller::DifferentialBotController, controller_interface::ControllerInterface)
