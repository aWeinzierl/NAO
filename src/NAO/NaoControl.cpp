#include "NaoControl.h"

#include <exception>
#include <memory>
#include <iostream>

#include <boost/math/constants/constants.hpp>

#include "../StdExtension/MakeUnique.h"

namespace NAO {

    void NaoControl::spin_thread() {
        ros::Rate r(30);
        while (!m_stop_thread) {
            ros::spinOnce();
            r.sleep();
        }
    }

    double NaoControl::degree_to_radians(const double angle) const noexcept {
        return angle / 180 * boost::math::constants::pi<double>();
    }

    double NaoControl::radian_to_degrees(const double angle) const noexcept {
        return angle * 180 / boost::math::constants::pi<double>();
    }


    NaoControl::NaoControl() : m_jointAnglesClient("/joint_angles_action", true), m_timeOut(10) {

        m_stop_thread = false;

        // subscribe to topic joint_states and specify that all data will be processed by function sensor_callback
        m_sensor_data_sub = m_nodeHandle.subscribe("/joint_states", 1, &NaoControl::sensor_callback, this);

        // subscribe to topic bumper and specify that all data will be processed by function bumper_callback
        m_bumper_sub = m_nodeHandle.subscribe("/bumper", 1, &NaoControl::bumper_callback, this);

        // subscribe to topic tactile_touch and specify that all data will be processed by function tactile_callback
        m_tactile_sub = m_nodeHandle.subscribe("/tactile_touch", 1, &NaoControl::tactile_callback, this);

        m_spin_thread = stdExtension::make_unique<boost::thread>(boost::bind(&NaoControl::spin_thread, this));

        m_bumper_callbacks[Bumper::BACK_BUMPER] = {};
        m_bumper_callbacks[Bumper::LEFT_BUMPER] = {};
        m_bumper_callbacks[Bumper::RIGHT_BUMPER] = {};
        m_head_touch_callbacks[HeadTouch::FRONT_HEAD_TOUCH] = {};
        m_head_touch_callbacks[HeadTouch::MIDDLE_HEAD_TOUCH] = {};
        m_head_touch_callbacks[HeadTouch::BACK_HEAD_TOUCH] = {};

        for (const auto &joint: AllContinuousJoints) {
            m_c_joints_callbacks[joint] = {};
        }

        m_jointAnglesClient.waitForServer();
    }

    NaoControl::~NaoControl() {
        m_stop_thread = true;
        sleep(1);
        m_spin_thread->join();
    }

// this callback function provides information about nao feet bumpers
    void NaoControl::bumper_callback(const naoqi_bridge_msgs::Bumper::ConstPtr &bumperState) {

        auto callbacks = m_bumper_callbacks.find(static_cast<Bumper>(bumperState->bumper))->second;
        for (const auto &callback :callbacks) {
            callback(bumperState->state);
        }
    }

// this callback provides information about current head tactile buttons.
    void NaoControl::tactile_callback(const naoqi_bridge_msgs::HeadTouch::ConstPtr &tactileState) {
        auto callbacks = m_head_touch_callbacks.find(static_cast<HeadTouch >(tactileState->button))->second;
        for (const auto &callback :callbacks) {
            callback(tactileState->state);
        }
    }


// this callback recives info about current joint states
    void NaoControl::sensor_callback(const sensor_msgs::JointState::ConstPtr &jointState) {

        for (const auto &keyValue: m_c_joints_callbacks) {
            auto &callbacks = keyValue.second;
            auto index = continuousJoints.find(keyValue.first)->second.Get_index();
            auto value = jointState->position.at(index);
            auto valueInDegrees = radian_to_degrees(value);
            for (const auto &callback : callbacks) {
                callback(valueInDegrees);
            }
        }
    }

// this is main loop which should send commands to the nao arms.
    void NaoControl::publish_joint_states() {
        /*
         * TODO tutorial
         */
        std::cout << "adada" << std::endl;
        Move_joint_to_position_async(ContinuousJoint::LEFT_SHOULDER_PITCH, -0.56, 0.05).Block_until_motion_finished();
    }

    void NaoControl::Block_until_motion_finished() {
        ros::Rate r_sleep(20);
        while (!m_jointAnglesClient.waitForResult(m_timeOut) && ros::ok()) {
            r_sleep.sleep();
        }
    }

    void NaoControl::create_and_sendAction(float jointGoalAngle, float velocity, const std::string &jointName) {
        naoqi_bridge_msgs::JointAnglesWithSpeedGoal action;
        action.joint_angles.relative = 0;
        action.joint_angles.speed = velocity;
        action.joint_angles.joint_names.push_back(jointName);
        action.joint_angles.joint_angles.push_back(jointGoalAngle);
        m_jointAnglesClient.sendGoal(action);
    }


    NaoControl &NaoControl::Move_joint_to_position_async(ContinuousJoint joint, float goalPosition, float velocity) {
        auto jointSpecPtr = continuousJoints.find(joint);
        if (jointSpecPtr == continuousJoints.end()) {
            std::cout << "joint not implemented" << std::flush;
            throw std::logic_error("joint not implemented");
        }
        auto jointSpec = jointSpecPtr->second;

        auto goalPositionRadians = degree_to_radians(goalPosition);
        std::cout << std::endl << "joint: " << continuousJoints.find(joint)->second.Get_name() << std::endl;
        std::cout << "radions: " << goalPositionRadians << "      " << std::endl;
        if (!jointSpec.Value_within_boundary(goalPositionRadians)) {
            std::cout << "std::out_of_range(\"goalPosition\")" << std::flush;
            throw std::out_of_range(
                    "goalPosition = " + std::to_string(goalPositionRadians) + " <-> [" +
                    std::to_string(jointSpec.Get_value_range().lowerLimit) + " ; " +
                    std::to_string(jointSpec.Get_value_range().upperLimit) + "]");
        }

        create_and_sendAction(goalPositionRadians, velocity, jointSpec.Get_name());

        return *this;
    }

    NaoControl &NaoControl::Move_joint_to_position_async(DiscreteJoint joint, float goalPosition, float velocity) {
        auto jointSpec = discreteJoints.find(joint)->second;
        if (!jointSpec.Value_valid(goalPosition))
            throw std::out_of_range("goalPosition");
        create_and_sendAction(goalPosition, velocity, jointSpec.Get_name());

        return *this;
    }

    void NaoControl::Block_forever() {
        spin_thread();
        m_stop_thread = false;
    }

    void NaoControl::Unblock() {
        m_stop_thread = true;
    }

    void NaoControl::SubscribeToSensor(Bumper bumper,
                                       const boost::function<void(bool pressed)> &callback) {
        m_bumper_callbacks[bumper].push_back(callback);
    }

    void NaoControl::SubscribeToSensor(ContinuousJoint joint,
                                       const boost::function<void(double angle)>& callback) {
        m_c_joints_callbacks[joint].push_back(callback);
    }

    void NaoControl::SubscribeToSensor(HeadTouch headTouch,
                                       const boost::function<void(bool pressed)>& callback) {
        m_head_touch_callbacks[headTouch].push_back(callback);
    }
}