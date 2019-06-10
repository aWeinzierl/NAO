#include <ros/init.h>
#include "NAO/NaoControl.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "nao");
    NAO::NaoControl nao;
    bool leftHandOpen = true;
    nao.Move_joint_to_position_async(NAO::DiscreteJoint::LEFT_HAND, leftHandOpen, 0.05).Block_until_motion_finished();


    nao.SubscribeToSensor(NAO::Bumper::LEFT_BUMPER,
                         [&nao, &leftHandOpen](bool pressed) {
                             if (pressed) {
                                 nao.Move_joint_to_position_async(NAO::DiscreteJoint::LEFT_HAND, leftHandOpen, 0.05);
                                 leftHandOpen = !leftHandOpen;
                             }
                         });

    bool rightHandOpen = true;
    nao.Move_joint_to_position_async(NAO::DiscreteJoint::RIGHT_HAND, rightHandOpen, 0.05).Block_until_motion_finished();
    nao.SubscribeToSensor(NAO::Bumper::RIGHT_BUMPER,
                         [&nao, &rightHandOpen](bool pressed) {
                             if (pressed) {
                                 nao.Move_joint_to_position_async(NAO::DiscreteJoint::RIGHT_HAND, rightHandOpen, 0.05);
                                 rightHandOpen = !rightHandOpen;
                             }
                         });

    double currentHeadPosition = 0;
    nao.SubscribeToSensor(NAO::ContinuousJoint::HEAD_YAW,
                         [&currentHeadPosition](double value) {
                             currentHeadPosition = value;
                         });

    nao.SubscribeToSensor(NAO::HeadTouch::FRONT_HEAD_TOUCH, //front clockwise ; middle counter
                         [&nao, &currentHeadPosition](bool pressed) {
                             if (pressed) {
                                 nao.Move_joint_to_position_async(NAO::ContinuousJoint::HEAD_YAW,
                                                                 currentHeadPosition - 10, 0.05);
                             }
                         });

    nao.SubscribeToSensor(NAO::HeadTouch::BACK_HEAD_TOUCH, //front clockwise ; middle counter
                         [&nao, &currentHeadPosition](bool pressed) {
                             if (pressed) {
                                 nao.Move_joint_to_position_async(NAO::ContinuousJoint::HEAD_YAW,
                                                                 currentHeadPosition + 10, 0.05);
                             }
                         });


    nao.Move_joint_to_position_async(NAO::ContinuousJoint::RIGHT_SHOULDER_PITCH, 0,
                                    0.05).Block_until_motion_finished();
    nao.Move_joint_to_position_async(NAO::ContinuousJoint::RIGHT_ELBOW_YAW, 0, 0.05).Block_until_motion_finished();
    nao.Move_joint_to_position_async(NAO::ContinuousJoint::LEFT_SHOULDER_PITCH, 0,
                                    0.05).Block_until_motion_finished();
    nao.Move_joint_to_position_async(NAO::ContinuousJoint::LEFT_ELBOW_YAW, 0, 0.05).Block_until_motion_finished();

    nao.Block_forever();

    return 0;

}
