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

    return 0;

}
