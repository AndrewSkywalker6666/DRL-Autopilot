//
// Created by yueqian on 30/11/2021.
// Refer to this site for more information
// http://gazebosim.org/tutorials?tut=set_velocity
//

#ifndef CUSTOM_GAZEBO_PLUGINS_GAZEBO_ROS_JOINT_MOTOR_HPP
#define CUSTOM_GAZEBO_PLUGINS_GAZEBO_ROS_JOINT_MOTOR_HPP

#include <gazebo/common/Plugin.hh>
#include <std_msgs/msg/float32.hpp>

// For std::unique_ptr, could be removed
#include <memory>

namespace gazebo_plugins
{
    // Forward declaration of private data class.
    class GazeboRosJointMotorPrivate;

    /// Example ROS-powered Gazebo plugin with some useful boilerplate.
    /// \details This is a `ModelPlugin`, but it could be any supported Gazebo plugin type, such as
    /// System, Visual, GUI, World, Sensor, etc.
    class GazeboRosJointMotor : public gazebo::ModelPlugin
    {
    public:
        /// Constructor
        GazeboRosJointMotor();

        /// Destructor
        virtual ~GazeboRosJointMotor();

        /// Gazebo calls this when the plugin is loaded.
        /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
        /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
        /// `gazebo::rendering::VisualPtr`, etc.
        /// \param[in] sdf SDF element containing user-defined parameters.
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

    protected:
        virtual void OnRosMsg(std_msgs::msg::Float32::SharedPtr msg);

        virtual void OnUpdate();

    private:
        /// Recommended PIMPL pattern. This variable should hold all private
        /// data members.
        std::unique_ptr<GazeboRosJointMotorPrivate> impl_;
    };
} // namespace gazebo_plugins

#endif //CUSTOM_GAZEBO_PLUGINS_GAZEBO_ROS_JOINT_MOTOR_HPP
