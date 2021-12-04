//
// Created by yueqian on 30/11/2021.
//

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

// set velocity feature uses joint_->SetParam, physical force is inherently implemented
// set position feature uses joint_->SetPosition, preserves velocity, teleport kinematics

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros_joint_motor.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <memory>

namespace gazebo_plugins
{
    /// Class to hold private data members (PIMPL pattern)
    class GazeboRosJointMotorPrivate
    {
    public:
        /// Connection to world update event. Callback is called while this is alive.
        gazebo::event::ConnectionPtr update_connection_;

        /// Node for ROS communication.
        gazebo_ros::Node::SharedPtr ros_node_;
        std::string vel_topic_;
        std::string pos_topic_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_setpoint_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pos_setpoint_sub_;

        /// gazebo entities
        gazebo::physics::JointPtr joint_;
        gazebo::physics::ModelPtr model_;

        /// variables
        double max_effort_;
        double vel_setpoint_;
    };

    GazeboRosJointMotor::GazeboRosJointMotor()
        : impl_(std::make_unique<GazeboRosJointMotorPrivate>())
    {
    }

    GazeboRosJointMotor::~GazeboRosJointMotor()
    {
    }

    void GazeboRosJointMotor::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        // Create a GazeboRos node instead of a common ROS node.
        // Pass it SDF parameters so common options like namespace and remapping
        // can be handled.
        impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
        const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();

        impl_->model_ = model;
        impl_->joint_ = model->GetJoint(sdf->Get<std::string>("joint_name"));
        if (impl_->joint_ == nullptr)
        {
            RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint do not exist, joint pid control plugin not loaded");
            return;
        }

        impl_->vel_topic_ = sdf->Get<std::string>("ros_topic_vel");
        impl_->max_effort_ = sdf->Get<double>("max_effort");
        impl_->pos_topic_ = sdf->Get<std::string>("ros_topic_pos");

        // The model pointer gives you direct access to the physics object,
        // for example:
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribing to velocity setpoint at [%s]", impl_->vel_topic_.c_str());
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribing to position setpoint at [%s]", impl_->pos_topic_.c_str());

        // use joint motor feature
        impl_->vel_setpoint_ = 0;
        impl_->joint_->SetParam("fmax", 0, impl_->max_effort_);
        impl_->joint_->SetParam("vel", 0, impl_->vel_setpoint_);
        impl_->joint_->SetPosition(0, 0, true);

        // setup callback
        impl_->vel_setpoint_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(
            impl_->vel_topic_,
            qos.get_subscription_qos(impl_->vel_topic_, rclcpp::SystemDefaultsQoS()),
            std::bind(&GazeboRosJointMotor::OnVelMsg, this, std::placeholders::_1));

        impl_->pos_setpoint_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(
            impl_->pos_topic_,
            qos.get_subscription_qos(impl_->pos_topic_, rclcpp::SystemDefaultsQoS()),
            std::bind(&GazeboRosJointMotor::OnPosMsg, this, std::placeholders::_1));


        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&GazeboRosJointMotor::OnUpdate, this));

    }

    void GazeboRosJointMotor::OnVelMsg(std_msgs::msg::Float32::SharedPtr msg)
    {
        impl_->vel_setpoint_ = msg->data;
    }

    void GazeboRosJointMotor::OnPosMsg(std_msgs::msg::Float32::SharedPtr msg)
    {
        impl_->joint_->SetPosition(0, msg->data, true);
    }

    void GazeboRosJointMotor::OnUpdate()
    {
        impl_->joint_->SetParam("vel", 0, impl_->vel_setpoint_);
    }

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosJointMotor)
} // namespace gazebo_plugins