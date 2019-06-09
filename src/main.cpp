#include <ros/init.h>
#include "NaoControl.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "nao_tutorial6");
    NAO::NaoControl ic;
    sleep(2);
    ic.publish_joint_states();


    double headPosition = 0;
    ic.Bumper_sensor_state.subscribe([&ic, &headPosition] (const naoqi_bridge_msgs::Bumper& bumper) {


        std::cout << "bumper: " << bumper.bumper;
        std::cout << "state: " << bumper.state;

        ic.Move_joint_to_position_async(NAO::ContinuousJoint::RIGHT_SHOULDER_PITCH, headPosition, 0.1);
    });

    ic.Bumper_sensor_state.with_latest_from(ic.Joint_sensor_state).subscribe(
            [&ic](std::tuple<naoqi_bridge_msgs::Bumper,sensor_msgs::JointState> bumperAndSensors){
                auto bumper = std::get<0>(bumperAndSensors);
                auto joints = std::get<1>(bumperAndSensors);
                auto headPosition = joints.position.at(
                        ic.continuousJoints.find(NAO::ContinuousJoint::HEAD_YAW)->second.Get_index()
                        );
                ic.Move_joint_to_position_async(NAO::ContinuousJoint::HEAD_YAW, headPosition+0.1, 0.1);
            }
            );

    return 0;

}
