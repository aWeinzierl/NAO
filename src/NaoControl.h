#pragma once

#include <boost/thread.hpp>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <actionlib/client/simple_action_client.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HandTouch.h>
#include <unordered_set>

namespace NAO {

    typedef actionlib::SimpleActionClient<naoqi_bridge_msgs::JointAnglesWithSpeedAction> JointAnglesClient;

    struct Interval {
        constexpr Interval(double lowerLimit, double upperLimit) : lowerLimit(lowerLimit), upperLimit(upperLimit) {}

        float lowerLimit;
        float upperLimit;
    };

    struct ApproximatelyEqual {
        template<typename KeyType>
        bool operator()(const KeyType &lhs, const KeyType &rhs) const {
            return std::abs(lhs - rhs) < std::numeric_limits<KeyType>::epsilon();
        }
    };

    class NaoControl {
    public:

        enum class HAND_POSITION {
            OPEN = 0,
            CLOSED = 1
        };

        NaoControl();

        ~NaoControl();

        // this is main loop which should send commands to the nao arms.
        void publish_joint_states();

        static constexpr Interval LEFT_SHOULDER_PITCH_LIMITS = Interval(-2.0857, 2.0857); // NOLINT(cert-err58-cpp)
        static constexpr Interval LEFT_SHOULDER_ROLL_LIMITS = Interval(-0.3142, 1.3265); // NOLINT(cert-err58-cpp)
        static constexpr Interval LEFT_ELBOW_YAW_LIMITS = Interval(-2.0857, 2.0857); // NOLINT(cert-err58-cpp)
        static constexpr Interval LEFT_ELBOW_ROLL_LIMITS = Interval(-1.5446, -0.0349); // NOLINT(cert-err58-cpp)
        static constexpr Interval LEFT_WRIST_YAW_LIMITS = Interval(-1.8238, 1.8238); // NOLINT(cert-err58-cpp)
        const std::unordered_set<
                double,
                std::unordered_set<double>::hasher,
                ApproximatelyEqual,
                std::unordered_set<double>::allocator_type> LEFT_HAND_STATES = {
                static_cast<double>(HAND_POSITION::OPEN),
                static_cast<double>(HAND_POSITION::CLOSED)
        };


        static constexpr Interval RIGHT_SHOULDER_PITCH_LIMITS = LEFT_SHOULDER_PITCH_LIMITS;
        static constexpr Interval RIGHT_SHOULDER_ROLL_LIMITS = Interval(-1.3265, 0.3142); // NOLINT(cert-err58-cpp)
        static constexpr Interval RIGHT_ELBOW_YAW_LIMITS = LEFT_ELBOW_YAW_LIMITS;
        static constexpr Interval RIGHT_ELBOW_ROLL_LIMITS = Interval(0.0349, 1.5446); // NOLINT(cert-err58-cpp)
        static constexpr Interval RIGHT_WRIST_YAW_LIMITS = LEFT_WRIST_YAW_LIMITS;
        const std::unordered_set<
                double,
                std::unordered_set<double>::hasher,
                ApproximatelyEqual,
                std::unordered_set<double>::allocator_type> RIGHT_HAND_STATES = LEFT_HAND_STATES;

        void Pitch_left_shoulder(float goalPosition, float velocity);

        void Pitch_left_shoulder_async(float goalPosition, float velocity);

        void Roll_left_shoulder(float goalPosition, float velocity);

        void Roll_left_shoulder_async(float goalPosition, float velocity);

        void Yaw_left_elbow(float goalPosition, float velocity);

        void Yaw_left_elbow_async(float goalPosition, float velocity);

        void Roll_left_elbow(float goalPosition, float velocity);

        void Roll_left_elbow_async(float goalPosition, float velocity);

        void Yaw_left_wrist(float goalPosition, float velocity);

        void Yaw_left_wrist_async(float goalPosition, float velocity);

        void Adjust_left_hand_position(HAND_POSITION goalPosition, float velocity);

        void Adjust_left_hand_positionAsync(HAND_POSITION goalPosition, float velocity);

        void Pitch_right_shoulder(float goalPosition, float velocity);

        void Pitch_right_shoulder_async(float goalPosition, float velocity);

        void Roll_right_shoulder(float goalPosition, float velocity);

        void Roll_right_shoulder_async(float goalPosition, float velocity);

        void Yaw_right_elbow(float goalPosition, float velocity);

        void Yaw_right_elbow_async(float goalPosition, float velocity);

        void Roll_right_elbow(float goalPosition, float velocity);

        void Roll_right_elbow_async(float goalPosition, float velocity);

        void Yaw_right_wrist(float goalPosition, float velocity);

        void Yaw_right_wrist_async(float goalPosition, float velocity);

        void Adjust_right_hand_position(HAND_POSITION goalPosition, float velocity);

        void Adjust_right_hand_positionAsync(HAND_POSITION goalPosition, float velocity);


    private:

        // ros handler
        ros::NodeHandle m_nodeHandle;

        // subscriber to joint states
        ros::Subscriber m_sensor_data_sub;

        // subscriber to bumpers states
        ros::Subscriber m_bumper_sub;

        // subscriber to head tactile states
        ros::Subscriber m_tactile_sub;

        // variables which store current states of the joints
        sensor_msgs::JointState m_current_left_arm_state;
        sensor_msgs::JointState m_current_right_arm_state;
        sensor_msgs::JointState m_current_head_legs_state;

        //define actionlib client for joint angles control
        JointAnglesClient m_jointAnglesClient;

        std::unique_ptr<boost::thread> m_spin_thread;

        const ros::Duration m_timeOut;

        bool m_stop_thread;

        // this callback function provides information about nao feet bumpers
        void bumper_callback(const naoqi_bridge_msgs::Bumper::ConstPtr &bumperState);

        // this callback provides information about current head tactile buttons.
        void tactile_callback(const naoqi_bridge_msgs::HandTouch::ConstPtr &tactileState);

        static constexpr bool within_interval_exclusive(double value, const Interval &interval);

        // this function checks joint limits of the left arm. You need to provide JointState vector
        bool check_joint_limits_left_arm(sensor_msgs::JointState joints);

        // this function checks joint limits of the right arm. You need to provide JointState vector
        bool check_joint_limits_right_arm(sensor_msgs::JointState joints);

        // this callback recives info about current joint states
        void sensor_callback(const sensor_msgs::JointState::ConstPtr &jointState);

        void block_until_action_finished();

        naoqi_bridge_msgs::JointAnglesWithSpeedGoal createAndSendAction(float jointGoalAngle, float velocity,
                                                                        const std::string &jointName);
        double degree_to_radians(double angle) const noexcept;

        void spin_thread();
    };
}