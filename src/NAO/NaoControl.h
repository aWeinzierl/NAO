#pragma once

#include <unordered_set>
#include <unordered_map>

#include <boost/thread.hpp>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <actionlib/client/simple_action_client.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HandTouch.h>
#include <rxcpp/rx.hpp>

#include "Joints.h"
#include "ContinuousJointSpecification.h"
#include "DiscreteJointSpecification.h"

namespace NAO {

    typedef actionlib::SimpleActionClient<naoqi_bridge_msgs::JointAnglesWithSpeedAction> JointAnglesClient;

    struct EnumClassHash {
        template<typename T>
        std::size_t operator()(T t) const {
            return static_cast<std::size_t>(t);
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

        const std::unordered_map<ContinuousJoint, ContinuousJointSpecification, EnumClassHash> continuousJoints{
                {ContinuousJoint::HEAD_PITCH,           ContinuousJointSpecification("HeadPitch",
                                                                                     Interval(-2.0857, 2.0857), 0)},
                {ContinuousJoint::HEAD_YAW,             ContinuousJointSpecification("HeadYaw",
                                                                                     Interval(-0.6720, 0.5149), 1)},

                {ContinuousJoint::LEFT_SHOULDER_PITCH,  ContinuousJointSpecification("LShoulderPitch",
                                                                                     Interval(-2.0857, 2.0857), 2)},
                {ContinuousJoint::LEFT_SHOULDER_ROLL,   ContinuousJointSpecification("LShoulderRoll",
                                                                                     Interval(-0.3142, 1.3265), 3)},
                {ContinuousJoint::LEFT_ELBOW_YAW,       ContinuousJointSpecification("LElbowYaw",
                                                                                     Interval(-2.0857, 2.0857), 4)},
                {ContinuousJoint::LEFT_ELBOW_ROLL,      ContinuousJointSpecification("LElbowRoll",
                                                                                     Interval(-1.5446, -0.0349), 5)},
                {ContinuousJoint::LEFT_WRIST_YAW,       ContinuousJointSpecification("LWristYaw",
                                                                                     Interval(-1.8238, 1.8238), 6)},

                {ContinuousJoint::RIGHT_SHOULDER_PITCH, ContinuousJointSpecification("RShoulderPitch",
                                                                                     Interval(-2.0857, 2.0857), 20)},
                {ContinuousJoint::RIGHT_SHOULDER_ROLL,  ContinuousJointSpecification("RShoulderRoll",
                                                                                     Interval(-1.3265, 0.3142), 21)},
                {ContinuousJoint::RIGHT_ELBOW_YAW,      ContinuousJointSpecification("RElbowYaw",
                                                                                     Interval(-2.0857, 2.0857), 22)},
                {ContinuousJoint::RIGHT_ELBOW_ROLL,     ContinuousJointSpecification("RElbowRoll",
                                                                                     Interval(0.0349, 1.5446), 23)},
                {ContinuousJoint::RIGHT_WRIST_YAW,      ContinuousJointSpecification("RWristYaw",
                                                                                     Interval(-1.8238, 1.8238), 24)},
        };

        const std::unordered_map<DiscreteJoint, DiscreteJointSpecification, EnumClassHash> discreteJoints{
                {DiscreteJoint::LEFT_HAND,  DiscreteJointSpecification("LHand", {
                        static_cast<double>(HAND_POSITION::OPEN),
                        static_cast<double>(HAND_POSITION::CLOSED)
                }, 7)},
                {DiscreteJoint::RIGHT_HAND, DiscreteJointSpecification("RHand", {
                        static_cast<double>(HAND_POSITION::OPEN),
                        static_cast<double>(HAND_POSITION::CLOSED)
                }, 25)},
        };

        NaoControl& Move_joint_to_position_async(ContinuousJoint joint, float goalPosition, float velocity);

        NaoControl& Move_joint_to_position_async(DiscreteJoint joint, float goalPosition, float velocity);

        void Block_until_motion_finished();
        void Block_forever();
        void Unblock();

        rxcpp::observable<naoqi_bridge_msgs::Bumper::ConstPtr> Bumper_sensor_state;
        rxcpp::observable<naoqi_bridge_msgs::HandTouch::ConstPtr> Hand_touch_sensor_state;
        rxcpp::observable<sensor_msgs::JointState::ConstPtr> Joint_sensor_state;

    private:

        std::vector<boost::function<void(const naoqi_bridge_msgs::Bumper::ConstPtr &)>> bumperCallbacks;

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

        // this function checks joint limits of the joints. You need to provide JointState vector
        bool check_joint_limits(sensor_msgs::JointState joints) const;


        // this callback recives info about current joint states
        void sensor_callback(const sensor_msgs::JointState::ConstPtr &jointState);


        void create_and_sendAction(float jointGoalAngle, float velocity, const std::string &jointName);

        double degree_to_radians(double angle) const noexcept;

        void spin_thread();

    };
}
