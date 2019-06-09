#include <ros/init.h>
#include "NAO/NaoControl.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "nao_tutorial6");
    NAO::NaoControl ic;
    sleep(2);

    ic.Move_joint_to_position_async(NAO::ContinuousJoint::RIGHT_SHOULDER_PITCH, -0.5,
                                    0.05)->Block_until_motion_finished();
    ic.Move_joint_to_position_async(NAO::ContinuousJoint::RIGHT_ELBOW_YAW, 0, 0.05)->Block_until_motion_finished();
    ic.Move_joint_to_position_async(NAO::ContinuousJoint::LEFT_SHOULDER_PITCH, -0.5,
                                    0.05)->Block_until_motion_finished();
    ic.Move_joint_to_position_async(NAO::ContinuousJoint::LEFT_ELBOW_YAW, 0, 0.05)->Block_until_motion_finished();


    ic.Bumper_sensor_state.with_latest_from(ic.Joint_sensor_state).subscribe(
            [&ic](std::tuple<naoqi_bridge_msgs::Bumper, sensor_msgs::JointState> bumperAndSensors) {
                auto bumper = std::get<0>(bumperAndSensors);
                auto joints = std::get<1>(bumperAndSensors);

                NAO::DiscreteJoint relevantHand;
                if (bumper.bumper == bumper.left) {
                    relevantHand = NAO::DiscreteJoint::LEFT_HAND;
                } else if (bumper.bumper == bumper.right) {
                    relevantHand = NAO::DiscreteJoint::RIGHT_HAND;
                } else return;

                auto handPosition = joints.position[
                        ic.discreteJoints.find(relevantHand)->second.Get_index()];


                if (handPosition == 0) {
                    ic.Move_joint_to_position_async(relevantHand, 1, 0.1);
                } else {
                    ic.Move_joint_to_position_async(relevantHand, 0, 0.1);

                }
            }
    );

    ic.Hand_touch_sensor_state.with_latest_from(ic.Joint_sensor_state).subscribe(
            [&ic](std::tuple<naoqi_bridge_msgs::HandTouch, sensor_msgs::JointState> touchAndSensors) {
                auto touchSensor = std::get<0>(touchAndSensors);
                auto joints = std::get<1>(touchAndSensors);

                double offset;
                //TODO: which action results in which value?
                offset = -0.1;

                auto headPosition = joints.position[
                        ic.continuousJoints.find(NAO::ContinuousJoint::HEAD_YAW)->second.Get_index()];

                ic.Move_joint_to_position_async(NAO::ContinuousJoint::HEAD_YAW, headPosition+offset, 0.05);

            }
    );

    return 0;

}
