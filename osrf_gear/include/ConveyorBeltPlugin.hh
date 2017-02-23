/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _GAZEBO_CONVEYOR_BELT_PLUGIN_HH_
#define _GAZEBO_CONVEYOR_BELT_PLUGIN_HH_

#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/math/Angle.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/util/system.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief A plugin for simulating a conveyor belt.
  /// The plugin accepts the following SDF parameters:

  /// <velocity>: Sets the initial velocity of the belt (m/s).
  /// <joint>: Joint name used to control the belt.
  /// <belt>: Belt's link name.
  ///
  /// Here's an example of a valid SDF conveyor belt:
  /// <model name="conveyor_belt">
  ///   <static>false</static>
  ///   <pose>1.21 -2 0.8126 0 0 -1.57079</pose>
  ///   <plugin name="conveyor_belt_plugin" filename="libConveyorBeltPlugin.so">
  ///     <robot_namespace>/ariac</robot_namespace>
  ///     <velocity>0</velocity>
  ///   </plugin>
  ///   <link name="belt">
  ///     <pose>-5 0 0 0 0 0</pose>
  ///     <inertial>
  ///       <inertia>
  ///         <ixx>3.8185</ixx>
  ///         <ixy>0</ixy>
  ///         <ixz>0</ixz>
  ///         <iyy>1781.5</iyy>
  ///         <iyz>0</iyz>
  ///         <izz>1784.72</izz>
  ///       </inertia>
  ///       <mass>100</mass>
  ///     </inertial>
  ///     <visual name="belt_visual">
  ///       <geometry>
  ///         <box>
  ///           <size>14.62206 0.65461 0.18862</size>
  ///         </box>
  ///       </geometry>
  ///     </visual>
  ///     <collision name="belt_collision">
  ///       <geometry>
  ///         <box>
  ///           <size>14.62206 0.65461 0.18862</size>
  ///         </box>
  ///       </geometry>
  ///       <surface>
  ///         <friction>
  ///           <ode>
  ///             <mu>1.0</mu>
  ///             <mu2>1.0</mu2>
  ///           </ode>
  ///           <torsional>
  ///             <coefficient>1000.0</coefficient>
  ///             <patch_radius>0.1</patch_radius>
  ///           </torsional>
  ///         </friction>
  ///       </surface>
  ///     </collision>
  ///   </link>
  ///   <joint name="belt_joint" type="prismatic">
  ///     <parent>world</parent>
  ///     <child>belt</child>
  ///     <axis>
  ///       <xyz>1 0 0</xyz>
  ///       <limit>
  ///         <lower>0</lower>
  ///         <upper>1.0</upper>
  ///       </limit>
  ///     </axis>
  ///   </joint>
  /// </model>
  class GAZEBO_VISIBLE ConveyorBeltPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ConveyorBeltPlugin() = default;

    /// \brief Destructor.
    public: virtual ~ConveyorBeltPlugin();

    /// \brief Load the model plugin.
    /// \param[in] _model Pointer to the model that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    /// \brief Callback that receives the world update event
    protected: void OnUpdate();

    /// \brief Get the velocity of the conveyor belt.
    /// \return Velocity of the belt (m/s).
    protected: double Velocity() const;

    /// \brief Set the state of the conveyor belt.
    /// \param[in] _velocity Velocity of the belt (m/s).
    protected: void SetVelocity(const double _velocity);

    /// \brief Belt velocity (m/s).
    protected: double beltVelocity;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief The joint that controls the movement of the belt.
    private: physics::JointPtr joint;

    /// \brief The belt's link.
    private: physics::LinkPtr link;

    /// \brief When the joint reaches this point, it will go back to its initial
    /// position.
    private: math::Angle limit;
  };
}
#endif
