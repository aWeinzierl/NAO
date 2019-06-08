#include "NaoControl.h"

#include <exception>
#include <memory>

#include <boost/math/constants/constants.hpp>

namespace NAO {

    void NaoControl::spinThread() {
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

        // subscribe to topic joint_states and specify that all data will be processed by function sensorCallback
        m_sensor_data_sub = m_nodeHandle.subscribe("/joint_states", 1, &NaoControl::sensorCallback, this);

        // subscribe to topic bumper and specify that all data will be processed by function bumperCallback
        m_bumper_sub = m_nodeHandle.subscribe("/bumper", 1, &NaoControl::bumperCallback, this);

        // subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
        m_tactile_sub = m_nodeHandle.subscribe("/tactile_touch", 1, &NaoControl::tactileCallback, this);

        m_spin_thread = std::make_unique<boost::thread>(boost::bind(&NaoControl::spinThread, this));
    }

    NaoControl::~NaoControl() {
        m_stop_thread = true;
        sleep(1);
        m_spin_thread->join();
    }

// this callback function provides information about nao feet bumpers
    void NaoControl::bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr &bumperState) {

    }

// this callback provides information about current head tactile buttons.
    void NaoControl::tactileCallback(const naoqi_bridge_msgs::HandTouch::ConstPtr &tactileState) {


    }

    constexpr bool NaoControl::within_interval_exclusive(double value, const Interval &interval) {
        return interval.lowerLimit < value && interval.upperLimit > value;
    }

// this function checks joint limits of the left arm. You need to provide JointState vector
    bool NaoControl::check_joint_limits_left_arm(sensor_msgs::JointState joints) {
        std::vector<double> &positions = joints.position;

        return within_interval_exclusive(positions.at(2), LEFT_SHOULDER_PITCH_LIMITS)
               && within_interval_exclusive(positions.at(3), LEFT_SHOULDER_ROLL_LIMITS)
               && within_interval_exclusive(positions.at(4), LEFT_ELBOW_YAW_LIMITS)
               && within_interval_exclusive(positions.at(5), LEFT_ELBOW_ROLL_LIMITS)
               && within_interval_exclusive(positions.at(6), LEFT_WRIST_YAW_LIMITS)
               && LEFT_HAND_STATES.end() != LEFT_HAND_STATES.find(positions.at(7));
    }

// this function checks joint limits of the right arm. You need to provide JointState vector
    bool NaoControl::check_joint_limits_right_arm(sensor_msgs::JointState joints) {
        std::vector<double> &positions = joints.position;

        return within_interval_exclusive(positions.at(20), RIGHT_SHOULDER_PITCH_LIMITS)
               && within_interval_exclusive(positions.at(21), RIGHT_SHOULDER_ROLL_LIMITS)
               && within_interval_exclusive(positions.at(22), RIGHT_ELBOW_YAW_LIMITS)
               && within_interval_exclusive(positions.at(23), RIGHT_ELBOW_ROLL_LIMITS)
               && within_interval_exclusive(positions.at(24), RIGHT_WRIST_YAW_LIMITS)
               && RIGHT_HAND_STATES.end() != RIGHT_HAND_STATES.find(positions.at(25));
    }

// this callback recives info about current joint states
    void NaoControl::sensorCallback(const sensor_msgs::JointState::ConstPtr &jointState) {
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
        Pitch_right_shoulder(-0.56, 0.05);
    }

    void NaoControl::block_until_action_finished() {
        ros::Rate r_sleep(20);
        while (!m_jointAnglesClient.waitForResult(m_timeOut) && ros::ok()) {
            r_sleep.sleep();
        }
    }

    void NaoControl::Pitch_left_shoulder_async(float goalPosition, float velocity) {
        auto goalPositionRadians = degree_to_radians(goalPosition);
        if (!within_interval_exclusive(goalPositionRadians, RIGHT_SHOULDER_PITCH_LIMITS))
            throw std::out_of_range("goalPosition");
        createAndSendAction(goalPositionRadians, velocity, "LShoulderPitch");
    }

    void NaoControl::Pitch_left_shoulder(float goalPosition, float velocity) {
        Pitch_left_shoulder_async(goalPosition, velocity);
        block_until_action_finished();
    }

    void NaoControl::Roll_left_shoulder_async(float goalPosition, float velocity) {
        auto goalPositionRadians = degree_to_radians(goalPosition);
        if (!within_interval_exclusive(goalPositionRadians, RIGHT_SHOULDER_ROLL_LIMITS))
            throw std::out_of_range("goalPosition");
        createAndSendAction(goalPositionRadians, velocity, "LShoulderRoll");
    }

    void NaoControl::Roll_left_shoulder(float goalPosition, float velocity) {
        Roll_left_shoulder_async(goalPosition, velocity);
        block_until_action_finished();
    }

    void NaoControl::Yaw_left_elbow_async(float goalPosition, float velocity) {
        auto goalPositionRadians = degree_to_radians(goalPosition);
        if (!within_interval_exclusive(goalPositionRadians, RIGHT_SHOULDER_ROLL_LIMITS))
            throw std::out_of_range("goalPosition");
        createAndSendAction(goalPositionRadians, velocity, "LElbowYaw");
    }

    void NaoControl::Yaw_left_elbow(float goalPosition, float velocity) {
        Yaw_left_elbow_async(goalPosition, velocity);
        block_until_action_finished();
    }

    void NaoControl::Roll_left_elbow_async(float goalPosition, float velocity) {
        auto goalPositionRadians = degree_to_radians(goalPosition);
        if (!within_interval_exclusive(goalPositionRadians, RIGHT_SHOULDER_ROLL_LIMITS))
            throw std::out_of_range("goalPosition");
        createAndSendAction(goalPositionRadians, velocity, "LElbowRoll");
    }

    void NaoControl::Roll_left_elbow(float goalPosition, float velocity) {
        Roll_left_elbow_async(goalPosition, velocity);
        block_until_action_finished();
    }

    void NaoControl::Yaw_left_wrist_async(float goalPosition, float velocity) {
        auto goalPositionRadians = degree_to_radians(goalPosition);
        if (!within_interval_exclusive(goalPositionRadians, RIGHT_SHOULDER_ROLL_LIMITS))
            throw std::out_of_range("goalPosition");
        createAndSendAction(goalPositionRadians, velocity, "LWristYaw");
    }

    void NaoControl::Yaw_left_wrist(float goalPosition, float velocity) {
        Yaw_left_wrist_async(goalPosition, velocity);
        block_until_action_finished();
    }

    void NaoControl::Adjust_left_hand_positionAsync(NaoControl::HAND_POSITION goalPosition, float velocity) {
        createAndSendAction(static_cast<double>(goalPosition), velocity, "LHand");
    }

    void NaoControl::Adjust_left_hand_position(NaoControl::HAND_POSITION goalPosition, float velocity) {
        Adjust_left_hand_positionAsync(goalPosition, velocity);
        block_until_action_finished();
    }

    void NaoControl::Pitch_right_shoulder_async(float goalPosition, float velocity) {
        auto goalPositionRadians = degree_to_radians(goalPosition);
        if (!within_interval_exclusive(goalPositionRadians, RIGHT_SHOULDER_PITCH_LIMITS))
            throw std::out_of_range("goalPosition");
        createAndSendAction(goalPositionRadians, velocity, "RShoulderPitch");
    }

    void NaoControl::Pitch_right_shoulder(float goalPosition, float velocity) {
        Pitch_right_shoulder_async(goalPosition, velocity);
        block_until_action_finished();
    }

    void NaoControl::Roll_right_shoulder_async(float goalPosition, float velocity) {
        auto goalPositionRadians = degree_to_radians(goalPosition);
        if (!within_interval_exclusive(goalPositionRadians, RIGHT_SHOULDER_ROLL_LIMITS))
            throw std::out_of_range("goalPosition");
        createAndSendAction(goalPositionRadians, velocity, "RShoulderRoll");
    }

    void NaoControl::Roll_right_shoulder(float goalPosition, float velocity) {
        Roll_right_shoulder_async(goalPosition, velocity);
        block_until_action_finished();
    }

    void NaoControl::Yaw_right_elbow_async(float goalPosition, float velocity) {
        auto goalPositionRadians = degree_to_radians(goalPosition);
        if (!within_interval_exclusive(goalPositionRadians, RIGHT_SHOULDER_ROLL_LIMITS))
            throw std::out_of_range("goalPosition");
        createAndSendAction(goalPositionRadians, velocity, "RElbowYaw");
    }

    void NaoControl::Yaw_right_elbow(float goalPosition, float velocity) {
        Yaw_right_elbow_async(goalPosition, velocity);
        block_until_action_finished();
    }

    void NaoControl::Roll_right_elbow_async(float goalPosition, float velocity) {
        auto goalPositionRadians = degree_to_radians(goalPosition);
        if (!within_interval_exclusive(goalPositionRadians, RIGHT_SHOULDER_ROLL_LIMITS))
            throw std::out_of_range("goalPosition");
        createAndSendAction(goalPositionRadians, velocity, "RElbowRoll");
    }

    void NaoControl::Roll_right_elbow(float goalPosition, float velocity) {
        Roll_right_elbow_async(goalPosition, velocity);
        block_until_action_finished();
    }

    void NaoControl::Yaw_right_wrist_async(float goalPosition, float velocity) {
        auto goalPositionRadians = degree_to_radians(goalPosition);
        if (!within_interval_exclusive(goalPositionRadians, RIGHT_SHOULDER_ROLL_LIMITS))
            throw std::out_of_range("goalPosition");
        createAndSendAction(goalPositionRadians, velocity, "RWristYaw");
    }

    void NaoControl::Yaw_right_wrist(float goalPosition, float velocity) {
        Yaw_right_wrist_async(goalPosition, velocity);
        block_until_action_finished();
    }

    void NaoControl::Adjust_right_hand_positionAsync(NaoControl::HAND_POSITION goalPosition, float velocity) {
        createAndSendAction(static_cast<double>(goalPosition), velocity, "RHand");
    }

    void NaoControl::Adjust_right_hand_position(NaoControl::HAND_POSITION goalPosition, float velocity) {
        Adjust_right_hand_positionAsync(goalPosition, velocity);
        block_until_action_finished();
    }

    naoqi_bridge_msgs::JointAnglesWithSpeedGoal NaoControl::createAndSendAction(float jointGoalAngle, float velocity,
                                                                                const std::string &jointName) {
        naoqi_bridge_msgs::JointAnglesWithSpeedGoal action;
        action.joint_angles.relative = 0;
        action.joint_angles.speed = velocity;
        action.joint_angles.joint_names.push_back(jointName);
        action.joint_angles.joint_angles.push_back(jointGoalAngle);
        m_jointAnglesClient.sendGoal(action);
    }

}