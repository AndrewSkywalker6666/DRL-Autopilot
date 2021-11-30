// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo_ros_motor_model.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo_plugins
{
  /// Class to hold private data members (PIMPL pattern)
  class GazeboRosMotorModelPrivate
  {
  public:
    /// Connection to world update event. Callback is called while this is alive.
    gazebo::event::ConnectionPtr update_connection_;

    /// Node for ROS communication.
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_sub_;

    /// variables
    std::string joint_name_;
    std::string link_name_;
    std::string command_sub_topic_;

    int turning_direction_;

    double max_rot_velocity_;
    double moment_constant_;
    double motor_constant_;
    double rolling_moment_coefficient_;
    double rotor_drag_coefficient_;
    double rotor_velocity_slowdown_sim_;
    double time_constant_down_;
    double time_constant_up_;

    double prev_sim_time_;
    double sampling_time_;

    double ref_motor_rot_vel_;
    double motor_rot_vel_;
    double wind_vel_;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::JointPtr joint_;
    gazebo::physics::LinkPtr link_;

    std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
  };

  GazeboRosMotorModel::GazeboRosMotorModel()
      : impl_(std::make_unique<GazeboRosMotorModelPrivate>())
  {
  }

  GazeboRosMotorModel::~GazeboRosMotorModel()
  {
  }

  void GazeboRosMotorModel::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    // Create a GazeboRos node instead of a common ROS node.
    // Pass it SDF parameters so common options like namespace and remapping
    // can be handled.
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

    // get params
    impl_->joint_name_ = sdf->Get<std::string>("jointName");
    impl_->link_name_ = sdf->Get<std::string>("linkName");
    impl_->command_sub_topic_ = sdf->Get<std::string>("commandSubTopic");
    if (sdf->Get<std::string>("turningDirection") == "cw")
    {
      impl_->turning_direction_ = -1;
    }
    else if (sdf->Get<std::string>("turningDirection") == "ccw")
    {
      impl_->turning_direction_ = 1;
    }
    else
    {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Error determining rotate direction, plugin not loaded");
      return;
    }
    impl_->max_rot_velocity_ = sdf->Get<double>("maxRotVelocity");
    impl_->moment_constant_ = sdf->Get<double>("momentConstant");
    impl_->motor_constant_ = sdf->Get<double>("motorConstant");
    impl_->rolling_moment_coefficient_ = sdf->Get<double>("rollingMomentCoefficient");
    impl_->rotor_drag_coefficient_ = sdf->Get<double>("rotorDragCoefficient");
    impl_->rotor_velocity_slowdown_sim_ = sdf->Get<double>("rotorVelocitySlowdownSim");
    impl_->time_constant_down_ = sdf->Get<double>("timeConstantDown");
    impl_->time_constant_up_ = sdf->Get<double>("timeConstantUp");
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribing to command at [%s]", impl_->command_sub_topic_.c_str());

    // get model joint and link pointers
    impl_->model_ = model;
    impl_->joint_ = model->GetJoint(impl_->joint_name_);
    impl_->link_ = model->GetLink(impl_->link_name_);
    if (impl_->joint_ == nullptr || impl_->link_ == nullptr)
    {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint or link do not exist, plugin not loaded");
      return;
    }

    // setpoint callback
    impl_->ref_motor_rot_vel_ = 0;
    const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();
    impl_->setpoint_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float64>(
        impl_->command_sub_topic_,
        qos.get_subscription_qos(impl_->command_sub_topic_, rclcpp::SystemDefaultsQoS()),
        std::bind(&GazeboRosMotorModel::OnRosMsg, this, std::placeholders::_1));

    // Create a connection so the OnUpdate function is called at every simulation
    // iteration. Remove this call, the connection and the callback if not needed.
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosMotorModel::OnUpdate, this, std::placeholders::_1));

    // rotor velocity filter
    impl_->rotor_velocity_filter_.reset(new FirstOrderFilter<double>(impl_->time_constant_up_, impl_->time_constant_down_, impl_->ref_motor_rot_vel_));
  }

  void GazeboRosMotorModel::OnRosMsg(std_msgs::msg::Float64::SharedPtr msg)
  {
    impl_->ref_motor_rot_vel_ = std::min((double)msg->data, impl_->max_rot_velocity_);
    //RCLCPP_INFO(impl_->ros_node_->get_logger(), "%f", impl_->ref_motor_rot_vel_);
  }

  void GazeboRosMotorModel::OnUpdate(const gazebo::common::UpdateInfo &_info)
  {
    // update time delta
    impl_->sampling_time_ = _info.simTime.Double() - impl_->prev_sim_time_;
    impl_->prev_sim_time_ = _info.simTime.Double();

    // get current omega to calculate forces and moments
    impl_->motor_rot_vel_ = impl_->joint_->GetVelocity(0);

    // thrust = omega^2 * motor_constant, wind is set to zero for now
    impl_->wind_vel_ = 0;
    double real_motor_velocity = impl_->motor_rot_vel_ * impl_->rotor_velocity_slowdown_sim_;
    double force = std::abs(real_motor_velocity * real_motor_velocity * impl_->motor_constant_);
    ignition::math::Vector3d body_velocity = impl_->link_->WorldLinearVel();
    ignition::math::Vector3d joint_axis = impl_->joint_->GlobalAxis(0);
    impl_->link_->AddRelativeForce(ignition::math::Vector3d(0, 0, force));
    ignition::math::Vector3d relative_wind_velocity = body_velocity - impl_->wind_vel_;
    ignition::math::Vector3d velocity_parallel_to_rotor_axis = (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
    double vel = velocity_parallel_to_rotor_axis.Length();
    double scalar = 1 - vel / 25.0; // at 25 m/s the rotor will not produce any force anymore
    scalar = ignition::math::clamp(scalar, 0.0, 1.0);
    impl_->link_->AddRelativeForce(ignition::math::Vector3d(0, 0, force * scalar));

    // air drag = omega * lambda * v_perp_air
    ignition::math::Vector3d velocity_perpendicular_to_rotor_axis = relative_wind_velocity - (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
    ignition::math::Vector3d air_drag = -std::abs(real_motor_velocity) * impl_->rotor_drag_coefficient_ * velocity_perpendicular_to_rotor_axis;
    impl_->link_->AddForce(air_drag);

    // spinning torque = omega^2 * motor_constant * moment_constant
    gazebo::physics::Link_V parent_links = impl_->link_->GetParentJointsLinks();
    ignition::math::Pose3d pose_difference = impl_->link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
    ignition::math::Vector3d drag_torque(0, 0, -impl_->turning_direction_ * force * impl_->moment_constant_);
    ignition::math::Vector3d drag_torque_parent_frame = pose_difference.Rot().RotateVector(drag_torque); // Transforming the drag torque into the parent frame to handle arbitrary rotor orientations.
    parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

    // rolling moment = omega * mu * v_perp_air
    ignition::math::Vector3d rolling_moment;
    rolling_moment = -std::abs(real_motor_velocity) * impl_->rolling_moment_coefficient_ * velocity_perpendicular_to_rotor_axis;
    parent_links.at(0)->AddTorque(rolling_moment);

    // apply vel filter
    double ref_motor_rot_vel;
    ref_motor_rot_vel = impl_->rotor_velocity_filter_->updateFilter(impl_->ref_motor_rot_vel_, impl_->sampling_time_);

    // set velocity
    impl_->joint_->SetVelocity(0, impl_->turning_direction_ * ref_motor_rot_vel / impl_->rotor_velocity_slowdown_sim_);
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboRosMotorModel)
} // namespace gazebo_plugins
