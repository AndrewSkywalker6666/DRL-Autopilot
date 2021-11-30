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

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros_joint_pid_ctrl.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <memory>

namespace gazebo_plugins
{
  /// Class to hold private data members (PIMPL pattern)
  class GazeboRosJointPidCtrlPrivate
  {
  public:
    /// Connection to world update event. Callback is called while this is alive.
    gazebo::event::ConnectionPtr update_connection_;

    /// Node for ROS communication.
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr setpoint_sub_;
    std::string topic_;

    /// joints to be controlled
    gazebo::physics::JointPtr target_joint_;
    std::string ctrl_type_;
    double init_pos_;
    double init_vel_;
    gazebo::common::PID pid_param_;

    /// model need to be passed into callback
    gazebo::physics::ModelPtr model_;
  };

  GazeboRosJointPidCtrl::GazeboRosJointPidCtrl()
      : impl_(std::make_unique<GazeboRosJointPidCtrlPrivate>())
  {
  }

  GazeboRosJointPidCtrl::~GazeboRosJointPidCtrl()
  {
  }

  void GazeboRosJointPidCtrl::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {

    // Create a GazeboRos node instead of a common ROS node.
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

    // Get Qos profiles
    const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

    // check sdf file
    if (
        !sdf->HasElement("joint_name") ||
        !sdf->HasElement("ctrl_type") ||
        !sdf->HasElement("init_pos") ||
        !sdf->HasElement("init_vel") ||
        !sdf->HasElement("p") ||
        !sdf->HasElement("i") ||
        !sdf->HasElement("d") ||
        !sdf->HasElement("ros_topic"))
    {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Missing params, joint pid control plugin not loaded");
      return;
    }

    // store model and joint pointer as impl
    impl_->model_ = model;
    impl_->target_joint_ = model->GetJoint(sdf->Get<std::string>("joint_name"));
    if(impl_->target_joint_ == nullptr)
    {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint do not exist, joint pid control plugin not loaded");
      return;
    }

    // get control params
    impl_->ctrl_type_ = sdf->Get<std::string>("ctrl_type");
    impl_->init_pos_ = sdf->Get<double>("init_pos");
    impl_->init_vel_ = sdf->Get<double>("init_vel");
    impl_->pid_param_ = gazebo::common::PID(sdf->Get<double>("p"), sdf->Get<double>("i"), sdf->Get<double>("d"));

    // turn on PID controller
    if (impl_->ctrl_type_ == "velocity")
    {
      model->GetJointController()->SetVelocityPID(impl_->target_joint_->GetScopedName(), impl_->pid_param_);
      model->GetJointController()->SetVelocityTarget(impl_->target_joint_->GetScopedName(), impl_->init_vel_);
    }
    else if (impl_->ctrl_type_ == "position")
    {
      model->GetJointController()->SetPositionPID(impl_->target_joint_->GetScopedName(), impl_->pid_param_);
      model->GetJointController()->SetPositionTarget(impl_->target_joint_->GetScopedName(), impl_->init_pos_);
    }

    // subscribe to ros topic and register callback
    impl_->topic_ = sdf->Get<std::string>("ros_topic");
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Listening to setpoint at [%s]", impl_->topic_.c_str());
    if (impl_->ctrl_type_ == "velocity")
    {
      impl_->setpoint_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(
          impl_->topic_,
          qos.get_subscription_qos(impl_->topic_, rclcpp::SystemDefaultsQoS()),
          std::bind(&GazeboRosJointPidCtrl::OnVelMsg, this, std::placeholders::_1));
    }
    else if (impl_->ctrl_type_ == "position")
    {
      impl_->setpoint_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(
          impl_->topic_,
          qos.get_subscription_qos(impl_->topic_, rclcpp::SystemDefaultsQoS()),
          std::bind(&GazeboRosJointPidCtrl::OnPosMsg, this, std::placeholders::_1));
    }

    // Create a connection so the OnUpdate function is called at every simulation
    // iteration. Remove this call, the connection and the callback if not needed.
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosJointPidCtrl::OnUpdate, this));
  }

  void GazeboRosJointPidCtrl::OnVelMsg(std_msgs::msg::Float32::SharedPtr msg)
  {
    impl_->model_->GetJointController()->SetVelocityTarget(impl_->target_joint_->GetScopedName(), msg->data);
  }

  void GazeboRosJointPidCtrl::OnPosMsg(std_msgs::msg::Float32::SharedPtr msg)
  {
    impl_->model_->GetJointController()->SetPositionTarget(impl_->target_joint_->GetScopedName(), msg->data);
  }

  void GazeboRosJointPidCtrl::OnUpdate()
  {
    // Do something every simulation iteration
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboRosJointPidCtrl)
} // namespace gazebo_plugins
