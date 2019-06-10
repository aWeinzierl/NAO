#include <ros/init.h>
#include "NAO/NaoControl.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "nao_tutorial6");
    NAO::NaoControl ic;
    bool leftHandOpen = true;
    ic.Move_joint_to_position_async(NAO::DiscreteJoint::LEFT_HAND, leftHandOpen, 0.05).Block_until_motion_finished();


    ic.SubscribeToSensor(NAO::Bumper::LEFT_BUMPER,
                         [&ic, &leftHandOpen](bool pressed) {
                             if (pressed) {
                                 ic.Move_joint_to_position_async(NAO::DiscreteJoint::LEFT_HAND, leftHandOpen, 0.05);
                                 leftHandOpen = !leftHandOpen;
                             }
                         });

    bool rightHandOpen = true;
    ic.Move_joint_to_position_async(NAO::DiscreteJoint::RIGHT_HAND, rightHandOpen, 0.05).Block_until_motion_finished();
    ic.SubscribeToSensor(NAO::Bumper::RIGHT_BUMPER,
                         [&ic, &rightHandOpen](bool pressed) {
                             if (pressed) {
                                 ic.Move_joint_to_position_async(NAO::DiscreteJoint::RIGHT_HAND, rightHandOpen, 0.05);
                                 rightHandOpen = !rightHandOpen;
                             }
                         });

    double currentHeadPosition=0;
    ic.SubscribeToSensor(NAO::ContinuousJoint::HEAD_YAW,
                         [&currentHeadPosition](double value) {
                             currentHeadPosition = value;
                         });

    ic.SubscribeToSensor(NAO::HeadTouch::FRONT_HEAD_TOUCH, //front clockwise ; middle counter
                         [&ic, &currentHeadPosition](bool pressed) {
                             if (pressed) {
                                 ic.Move_joint_to_position_async(NAO::ContinuousJoint::HEAD_YAW, currentHeadPosition + 10, 0.05);
                             }
                         });

    ic.SubscribeToSensor(NAO::HeadTouch::MIDDLE_HEAD_TOUCH, //front clockwise ; middle counter
                         [&ic, &currentHeadPosition](bool pressed) {
                             if (pressed) {
                                 ic.Move_joint_to_position_async(NAO::ContinuousJoint::HEAD_YAW, currentHeadPosition - 10, 0.05);
                             }
                         });


    ic.Move_joint_to_position_async(NAO::ContinuousJoint::RIGHT_SHOULDER_PITCH, 0,
                                    0.05).Block_until_motion_finished();
    ic.Move_joint_to_position_async(NAO::ContinuousJoint::RIGHT_ELBOW_YAW, 0, 0.05).Block_until_motion_finished();
    ic.Move_joint_to_position_async(NAO::ContinuousJoint::LEFT_SHOULDER_PITCH, 0,
                                    0.05).Block_until_motion_finished();
    ic.Move_joint_to_position_async(NAO::ContinuousJoint::LEFT_ELBOW_YAW, 0, 0.05).Block_until_motion_finished();

    ic.Block_forever();

    return 0;

}
