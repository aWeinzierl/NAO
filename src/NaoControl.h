#pragma once

#include <boost/thread.hpp>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <actionlib/client/simple_action_client.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HandTouch.h>
#include <unordered_set>

typedef actionlib::SimpleActionClient<naoqi_bridge_msgs::JointAnglesWithSpeedAction> JointAnglesClient;

class Interval;

struct ApproximatelyEqual {
    template<typename KeyType>
    bool operator()(const KeyType &lhs, const KeyType &rhs) const {
        return std::abs(lhs - rhs) < std::numeric_limits<KeyType>::epsilon();
    }
};

class NaoControl {
public:
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

    NaoControl();

    ~NaoControl();

    // this callback function provides information about nao feet bumpers
    void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr &bumperState);

    // this callback provides information about current head tactile buttons.
    void tactileCallback(const naoqi_bridge_msgs::HandTouch::ConstPtr &tactileState);

    bool withinIntervalExclusive(double value, const Interval &interval);

    const std::unordered_set<
            double,
            std::unordered_set<double>::hasher,
            ApproximatelyEqual,
            std::unordered_set<double>::allocator_type> allowedStatesForHands = {0, 1};

    // this function checks joint limits of the left arm. You need to provide JointState vector
    bool check_joint_limits_left_arm(sensor_msgs::JointState joints);

    // this function checks joint limits of the right arm. You need to provide JointState vector
    bool check_joint_limits_right_arm(sensor_msgs::JointState joints);

    // this callback recives info about current joint states
    void sensorCallback(const sensor_msgs::JointState::ConstPtr &jointState);


    // this is main loop which should send commands to the nao arms.
    void publish_joint_states();

    void ExecuteActionAsync(naoqi_bridge_msgs::JointAnglesWithSpeedGoal action);

    void ExecuteAction(
            naoqi_bridge_msgs::JointAnglesWithSpeedGoal action,
            ros::Duration timeOut = ros::Duration(10));
};