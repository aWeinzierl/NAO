#include <ros/init.h>
#include "NAO/NaoControl.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "nao_tutorial6");
    NAO::NaoControl ic;
    sleep(2);
    bool leftHandOpen = true;
    ic.Move_joint_to_position_async(NAO::DiscreteJoint::LEFT_HAND, leftHandOpen ,0.05).Block_until_motion_finished();


    ic.SubscribeToSensor(NAO::Bumper::LEFT_BUMPER,
                         [&ic, &leftHandOpen](bool pressed) {
                             if (pressed){
                                 ic.Move_joint_to_position_async(NAO::DiscreteJoint::LEFT_HAND, leftHandOpen ,0.05);
                                 leftHandOpen = !leftHandOpen;
                             }
                         });

    bool rightHandOpen = true;
    ic.SubscribeToSensor(NAO::Bumper::RIGHT_BUMPER,
                         [&ic, &rightHandOpen](bool pressed) {
                             if (pressed){
                                 ic.Move_joint_to_position_async(NAO::DiscreteJoint::RIGHT_HAND, rightHandOpen ,0.05);
                                 rightHandOpen = !rightHandOpen;
                             }
                         });

    ic.Move_joint_to_position_async(NAO::ContinuousJoint::RIGHT_SHOULDER_PITCH,0,
                                    0.05).Block_until_motion_finished();
    ic.Move_joint_to_position_async(NAO::ContinuousJoint::RIGHT_ELBOW_YAW, -50, 0.05).Block_until_motion_finished();
    ic.Move_joint_to_position_async(NAO::ContinuousJoint::LEFT_SHOULDER_PITCH, 0,
                                    0.05).Block_until_motion_finished();
    ic.Move_joint_to_position_async(NAO::ContinuousJoint::LEFT_ELBOW_YAW, 50, 0.05).Block_until_motion_finished();

    ic.Block_forever();

    return 0;

}
