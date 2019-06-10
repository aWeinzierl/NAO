#include <ros/init.h>
#include "NAO/NaoControl.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "nao_tutorial6");
    NAO::NaoControl ic;
    sleep(2);

    ic.Move_joint_to_position_async(NAO::ContinuousJoint::RIGHT_SHOULDER_PITCH,0,
                                    0.05).Block_until_motion_finished();
    ic.Move_joint_to_position_async(NAO::ContinuousJoint::RIGHT_ELBOW_YAW, -50, 0.05).Block_until_motion_finished();
    ic.Move_joint_to_position_async(NAO::ContinuousJoint::LEFT_SHOULDER_PITCH, 0,
                                    0.05).Block_until_motion_finished();
    ic.Move_joint_to_position_async(NAO::ContinuousJoint::LEFT_ELBOW_YAW, 50, 0.05).Block_until_motion_finished();

    return 0;

}
