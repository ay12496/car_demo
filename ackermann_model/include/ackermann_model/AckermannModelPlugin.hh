/*
 * Copyright 2016 Open Source Robotics Foundation
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

#ifndef ACKERMANNMODEL_PLUGIN_HH_
#define ACKERMANNMODEL_PLUGIN_HH_

#include <ignition/math/Vector3.hh>
#include <ignition/msgs/cmd_vel2d.pb.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>

#include <ackermann_model/Control.h>
#include <ackermann_model/Status.h>

#include <ros/ros.h>


namespace gazebo
{

  class AckermannModelPluginPrivate
  {
    /// \enum DirectionType
    /// \brief Direction selector switch type.
    public: enum DirectionType {
              /// \brief Reverse
              REVERSE = -1,
              /// \brief Neutral
              NEUTRAL = 0,
              /// \brief Forward
              FORWARD = 1
            };
    
    /// \enum ControlMode
    /// \brief Control Mode: Auto(Planner)/Manual(Keyboard)      
    public: enum ControlMode {
              /// \brief Manual
              Manual,
              /// \brief Auto
              Auto
            };

    ///  \brief NodeHandler
    public: ros::NodeHandle nh;

    /// \brief Control Subcriber
    public: ros::Subscriber controlSub;

    /// \brief Vehicle Status Publisher
    public: ros::Publisher statusPub;

		/// \brief Status Message
	  public: ackermann_model::Status statusMsg;

    /// \brief Pointer to the world
    public: physics::WorldPtr world;

    /// \brief Pointer to the parent model
    public: physics::ModelPtr model;

    /// \brief Transport node
    public: transport::NodePtr gznode;

    /*TODO: find ignition module working. For now removing ignition code as no use found
    /// \brief Ignition transport node
    public: ignition::transport::Node node;

    /// \brief Ignition transport position pub
    public: ignition::transport::Node::Publisher posePub;

    /// \brief Ignition transport console pub
    public: ignition::transport::Node::Publisher consolePub;
    */

    /// \brief Physics update event connection
    public: event::ConnectionPtr updateConnection;

    /// \brief Chassis link
    public: physics::LinkPtr chassisLink;

    /// \brief Front left wheel joint
    public: physics::JointPtr flWheelJoint;

    /// \brief Front right wheel joint
    public: physics::JointPtr frWheelJoint;

    /// \brief Rear left wheel joint
    public: physics::JointPtr blWheelJoint;

    /// \brief Rear right wheel joint
    public: physics::JointPtr brWheelJoint;

    /// \brief Front left wheel steering joint
    public: physics::JointPtr flWheelSteeringJoint;

    /// \brief Front right wheel steering joint
    public: physics::JointPtr frWheelSteeringJoint;

    /// \brief Steering wheel joint
    public: physics::JointPtr handWheelJoint;

    /// \brief PID control for the front left wheel steering joint
    public: common::PID flWheelSteeringPID;

    /// \brief PID control for the front right wheel steering joint
    public: common::PID frWheelSteeringPID;

    /// \brief PID control for steering wheel joint
    public: common::PID handWheelPID;

    /// \brief Last pose msg time
    public: common::Time lastMsgTime;

    /// \brief Last sim time received
    public: common::Time lastSimTime;

    /// \brief Last sim time when a pedal command is received
    public: common::Time lastPedalCmdTime;

    /// \brief Last sim time when a steering command is received
    public: common::Time lastSteeringCmdTime;

    /// \brief Last sim time when a EV mode command is received
    public: common::Time lastModeCmdTime;

    /// \brief Current direction of the vehicle: FORWARD, NEUTRAL, REVERSE.
    public: DirectionType directionState;

    /// \brief Chassis aerodynamic drag force coefficient,
    /// with units of [N / (m/s)^2]
    public: double chassisAeroForceGain = 0;

    /// \brief Max torque that can be applied to the front wheels
    public: double frontTorque = 0;

    /// \brief Max torque that can be applied to the back wheels
    public: double backTorque = 0;

    /// \brief Max speed (m/s) of the car
    public: double maxSpeed = 0;

    /// \brief Max steering angle
    public: double maxSteer = 0;

    /// \brief Max torque that can be applied to the front brakes
    public: double frontBrakeTorque = 0;

    /// \brief Max torque that can be applied to the rear brakes
    public: double backBrakeTorque = 0;

    /// \brief Angle ratio between the steering wheel and the front wheels
    public: double steeringRatio = 0;

    /// \brief Max range of hand steering wheel
    public: double handWheelHigh = 0;

    /// \brief Min range of hand steering wheel
    public: double handWheelLow = 0;

    /// \brief Front left wheel desired steering angle (radians)
    public: double flWheelSteeringCmd = 0;

    /// \brief Front right wheel desired steering angle (radians)
    public: double frWheelSteeringCmd = 0;

    /// \brief Steering wheel desired angle (radians)
    public: double handWheelCmd = 0;

    /// \brief Front left wheel radius
    public: double flWheelRadius = 0;

    /// \brief Front right wheel radius
    public: double frWheelRadius = 0;

    /// \brief Rear left wheel radius
    public: double blWheelRadius = 0;

    /// \brief Rear right wheel radius
    public: double brWheelRadius = 0;

    /// \brief Front left joint friction
    public: double flJointFriction = 0;

    /// \brief Front right joint friction
    public: double frJointFriction = 0;

    /// \brief Rear left joint friction
    public: double blJointFriction = 0;

    /// \brief Rear right joint friction
    public: double brJointFriction = 0;

    /// \brief Distance distance between front and rear axles
    public: double wheelbaseLength = 0;

    /// \brief Distance distance between front left and right wheels
    public: double frontTrackWidth = 0;

    /// \brief Distance distance between rear left and right wheels
    public: double backTrackWidth = 0;

    /// \brief Gas energy density (J/gallon)
    public: const double kGasEnergyDensity = 1.29e8;

    /// \brief Battery charge capacity in Watt-hours
    public: double batteryChargeWattHours = 280;

    /// \brief Battery discharge capacity in Watt-hours
    public: double batteryDischargeWattHours = 260;

    /// \brief Gas engine efficiency
    public: double gasEfficiency = 0.37;

    /// \brief Minimum gas flow rate (gallons / sec)
    public: double minGasFlow = 1e-4;

    /// \brief Gas consumption (gallon)
    public: double gasConsumption = 0;

    /// \brief Battery state-of-charge (percent, 0.0 - 1.0)
    public: double batteryCharge = 0.75;

    /// \brief Battery charge threshold when it has to be recharged.
    public: const double batteryLowThreshold = 0.125;

    /// \brief Whether EV mode is on or off.
    public: bool evMode = false;

    /// \brief Gas pedal position in percentage. 1.0 = Fully accelerated.
    public: double gasPedalPercent = 0;

    /// \brief Power for charging a low battery (Watts).
    public: const double kLowBatteryChargePower = 2000;

    /// \brief Threshold delimiting the gas pedal (throttle) low and medium
    /// ranges.
    public: const double kGasPedalLowMedium = 0.25;

    /// \brief Threshold delimiting the gas pedal (throttle) medium and high
    /// ranges.
    public: const double kGasPedalMediumHigh = 0.5;

    /// \brief Threshold delimiting the speed (throttle) low and medium
    /// ranges in miles/h.
    public: const double speedLowMedium = 26.0;

    /// \brief Threshold delimiting the speed (throttle) medium and high
    /// ranges in miles/h.
    public: const double speedMediumHigh = 46.0;

    /// \brief Brake pedal position in percentage. 1.0 =
    public: double brakePedalPercent = 0;

    /// \brief Hand brake position in percentage.
    public: double handbrakePercent = 1.0;

    /// \brief Angle of steering wheel at last update (radians)
    public: double handWheelAngle = 0;

    /// \brief Steering angle of front left wheel at last update (radians)
    public: double flSteeringAngle = 0;

    /// \brief Steering angle of front right wheel at last update (radians)
    public: double frSteeringAngle = 0;

    /// \brief Linear velocity of chassis c.g. in world frame at last update (m/s)
    public: ignition::math::Vector3d chassisLinearVelocity;

    /// \brief Angular velocity of front left wheel at last update (rad/s)
    public: double flWheelAngularVelocity = 0;

    /// \brief Angular velocity of front right wheel at last update (rad/s)
    public: double frWheelAngularVelocity = 0;

    /// \brief Angular velocity of back left wheel at last update (rad/s)
    public: double blWheelAngularVelocity = 0;

    /// \brief Angular velocity of back right wheel at last update (rad/s)
    public: double brWheelAngularVelocity = 0;

    /// \brief Subscriber to the keyboard topic
    public: transport::SubscriberPtr keyboardSub;

    /// \brief Mutex to protect updates
    public: std::mutex mutex;

    /// \brief Odometer
    public: double odom = 0.0;

    /// \brief Keyboard control type
    public: int keyControl = 1;

    /// \brief Vehicle Control Auto/Manual mode
    public: ControlMode controlMode = Manual;

    /// \brief Publisher for the world_control topic.
    public: transport::PublisherPtr worldControlPub;
  };

  /// \brief A model plugin for ackermann model
  class AckermannModelPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: AckermannModelPlugin();

    /// \brief Destructor.
    public: virtual ~AckermannModelPlugin();

    // Documentation Inherited
    public: virtual void Reset();

    /// \brief Load the controller.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief ROS subscriber callback
    private: void OnCommand(const ackermann_model::Control::ConstPtr &msg);

    /// \brief Callback each time a key message is received.
    /// \param[in] _msg Keypress message.
    private: void OnKeyPress(ConstAnyPtr &_msg);

    /// \brief Callback each time a key message is received.
    /// \param[in] _msg Keypress message.
    // private: void OnKeyPressIgn(const ignition::msgs::Any &_msg);

    /// \brief Key control
    /// \param[in] _key key value
    private: void KeyControl(const int _key);

    /// \brief Key control type A
    /// \param[in] _key key value
    private: void KeyControlTypeA(const int _key);

    /// \brief Key control type B
    /// \param[in] _key key value
    private: void KeyControlTypeB(const int _key);

    /// \param[in] _msg Pose message
    private: void OnCmdVel(const ignition::msgs::Pose &_msg);

    /// \brief Command to change gear to reverse, neutral or forward (drive)
    /// \param[in] _msg Int32 message data
    private: void OnCmdGear(const ignition::msgs::Int32 &_msg);

    /// \brief Command to enable EV mode
    /// \param[in] _msg Boolean message data
    private: void OnCmdMode(const ignition::msgs::Boolean &_msg);

    /// \brief Command to reset the world
    /// \param[in] _msg Int32 message data. Not used
    private: void OnReset(const ignition::msgs::Any &_msg);

    /// \brief Command to stop the simulation
    /// \param[in] _msg Int32 message data. Not used
    private: void OnStop(const ignition::msgs::Any &_msg);

    /// \brief Update on every time step
    private: void Update();

    /// \brief Update steering wheel to front left/right wheel ratio
    private: void UpdateHandWheelRatio();

    /// \brief Get the radius of a collision
    private: double CollisionRadius(physics::CollisionPtr _collision);

    /// \brief Get the multiplier that is determined based on the direction
    /// state of the vehicle.
    /// \return 1.0 if FORWARD, -1.0 if REVERSE, 0.0 otherwise
    private: double GasTorqueMultiplier();

    /// \brief Private data
    private: std::unique_ptr<AckermannModelPluginPrivate> dataPtr;

    /// ROS Namespace
    private: std::string robot_namespace_;
  };
}
#endif
