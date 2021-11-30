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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_MOTOR_MODEL_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_MOTOR_MODEL_HPP_

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <std_msgs/msg/float64.hpp>
#include <ignition/math.hh>
#include <string>
// For std::unique_ptr, could be removed
#include <memory>
#include "common.h"

namespace gazebo_plugins
{
  // Forward declaration of private data class.
  class GazeboRosMotorModelPrivate;

  /// Example ROS-powered Gazebo plugin with some useful boilerplate.
  /// \details This is a `ModelPlugin`, but it could be any supported Gazebo plugin type, such as
  /// System, Visual, GUI, World, Sensor, etc.
  class GazeboRosMotorModel : public gazebo::ModelPlugin
  {
  public:
    /// Constructor
    GazeboRosMotorModel();

    /// Destructor
    virtual ~GazeboRosMotorModel();

    /// Gazebo calls this when the plugin is loaded.
    /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
    /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
    /// `gazebo::rendering::VisualPtr`, etc.
    /// \param[in] sdf SDF element containing user-defined parameters.
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  protected:
    /// Optional callback to be called at every simulation iteration.
    virtual void OnUpdate(const gazebo::common::UpdateInfo &_info);

    virtual void OnRosMsg(std_msgs::msg::Float64::SharedPtr msg);

  private:
    /// Recommended PIMPL pattern. This variable should hold all private
    /// data members.
    std::unique_ptr<GazeboRosMotorModelPrivate> impl_;
  };
} // namespace gazebo_plugins

#endif // GAZEBO_PLUGINS__GAZEBO_ROS_MotorModel_HPP_
