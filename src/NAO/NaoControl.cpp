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

    NaoControl::NaoControl() : m_jointAnglesClient("/joint_angles_action", true), m_timeOut(10) {

        m_stop_thread = false;

        // subscribe to topic joint_states and specify that all data will be processed by function sensor_callback
        m_sensor_data_sub = m_nodeHandle.subscribe("/joint_states", 1, &NaoControl::sensor_callback, this);

        // subscribe to topic bumper and specify that all data will be processed by function bumper_callback
        m_bumper_sub = m_nodeHandle.subscribe("/bumper", 1, &NaoControl::bumper_callback, this);

        // subscribe to topic tactile_touch and specify that all data will be processed by function tactile_callback
        m_tactile_sub = m_nodeHandle.subscribe("/tactile_touch", 1, &NaoControl::tactile_callback, this);

        m_spin_thread = stdExtension::make_unique<boost::thread>(boost::bind(&NaoControl::spin_thread, this));
    }

    NaoControl::~NaoControl() {
        m_stop_thread = true;
        sleep(1);
        m_spin_thread->join();
    }

// this callback function provides information about nao feet bumpers
    void NaoControl::bumper_callback(const naoqi_bridge_msgs::Bumper::ConstPtr &bumperState) {

    }

// this callback provides information about current head tactile buttons.
    void NaoControl::tactile_callback(const naoqi_bridge_msgs::HandTouch::ConstPtr &tactileState) {


    }

    constexpr bool NaoControl::within_interval_exclusive(double value, const Interval &interval) {
        return interval.lowerLimit < value && interval.upperLimit > value;
    }

// this function checks joint limits of the left arm. You need to provide JointState vector
    bool NaoControl::check_joint_limits(sensor_msgs::JointState joints) const {
        std::vector<double> &positions = joints.position;

        for (const auto &keyValue : continuousJoints) {
            const auto &jointSpec = keyValue.second;
            if (!jointSpec.Value_within_boundary(positions.at(jointSpec.Get_index()))) {
                return false;
            }
        }

        return true;
    }

// this callback recives info about current joint states
    void NaoControl::sensor_callback(const sensor_msgs::JointState::ConstPtr &jointState) {
        m_current_left_arm_state.name.clear();
        m_current_left_arm_state.position.clear();
        m_current_right_arm_state.name.clear();
        m_current_right_arm_state.position.clear();
        m_current_head_legs_state.name.clear();
        m_current_head_legs_state.position.clear();

        m_current_left_arm_state.header.stamp = ros::Time::now();

        m_current_left_arm_state.name.push_back(jointState->name.at(2));
        m_current_left_arm_state.position.push_back(jointState->position.at(2));
        m_current_left_arm_state.name.push_back(jointState->name.at(3));
        m_current_left_arm_state.position.push_back(jointState->position.at(3));
        m_current_left_arm_state.name.push_back(jointState->name.at(4));
        m_current_left_arm_state.position.push_back(jointState->position.at(4));
        m_current_left_arm_state.name.push_back(jointState->name.at(5));
        m_current_left_arm_state.position.push_back(jointState->position.at(5));
        m_current_left_arm_state.name.push_back(jointState->name.at(6));
        m_current_left_arm_state.position.push_back(jointState->position.at(6));

        m_current_right_arm_state.name.push_back(jointState->name.at(20));
        m_current_right_arm_state.position.push_back(jointState->position.at(20));
        m_current_right_arm_state.name.push_back(jointState->name.at(21));
        m_current_right_arm_state.position.push_back(jointState->position.at(21));
        m_current_right_arm_state.name.push_back(jointState->name.at(22));
        m_current_right_arm_state.position.push_back(jointState->position.at(22));
        m_current_right_arm_state.name.push_back(jointState->name.at(23));
        m_current_right_arm_state.position.push_back(jointState->position.at(23));
        m_current_right_arm_state.name.push_back(jointState->name.at(24));
        m_current_right_arm_state.position.push_back(jointState->position.at(24));

        m_current_head_legs_state.name.push_back(jointState->name.at(0));
        m_current_head_legs_state.position.push_back(jointState->position.at(0));
        m_current_head_legs_state.name.push_back(jointState->name.at(1));
        m_current_head_legs_state.position.push_back(jointState->position.at(1));
        for (int i = 7; i < 20; i++) {
            m_current_head_legs_state.name.push_back(jointState->name.at(i));
            m_current_head_legs_state.position.push_back(jointState->position.at(i));
        }
        m_current_head_legs_state.name.push_back(jointState->name.at(25));
        m_current_head_legs_state.position.push_back(jointState->position.at(25));
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

}