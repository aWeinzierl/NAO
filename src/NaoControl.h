#pragma once

#include <boost/thread.hpp>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <actionlib/client/simple_action_client.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HandTouch.h>
#include <unordered_set>

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
            std::unordered_set<double>::allocator_type> LEFT_HAND_STATES = {HAND_POSITION::OPEN, HAND_POSITION::CLOSED};


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

    void Pitch_right_shoulder(float goalPosition, float velocity);
    void Pitch_right_shoulder_async(float goalPosition, float velocity);

private:

    // ros handler
    ros::NodeHandle nh_;

    // subscriber to joint states
    ros::Subscriber sensor_data_sub;

    // subscriber to bumpers states
    ros::Subscriber bumper_sub;

    // subscriber to head tactile states
    ros::Subscriber tactile_sub;

    // variables which store current states of the joints
    sensor_msgs::JointState current_left_arm_state;
    sensor_msgs::JointState current_right_arm_state;
    sensor_msgs::JointState current_head_legs_state;

    //define actionlib client for joint angles control
    JointAnglesClient jointAnglesClient;

    boost::thread *spin_thread;


    // this callback function provides information about nao feet bumpers
    void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr &bumperState);

    // this callback provides information about current head tactile buttons.
    void tactileCallback(const naoqi_bridge_msgs::HandTouch::ConstPtr &tactileState);

    bool withinIntervalExclusive(double value, const Interval &interval);


    // this function checks joint limits of the left arm. You need to provide JointState vector
    bool check_joint_limits_left_arm(sensor_msgs::JointState joints);

    // this function checks joint limits of the right arm. You need to provide JointState vector
    bool check_joint_limits_right_arm(sensor_msgs::JointState joints);

    // this callback recives info about current joint states
    void sensorCallback(const sensor_msgs::JointState::ConstPtr &jointState);

    void block_until_action_finished();

    const ros::Duration m_timeOut;
};