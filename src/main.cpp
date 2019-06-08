#include <ros/init.h>
#include "NaoControl.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "nao_tutorial6");
    NAO::NaoControl ic;
    sleep(2);
    ic.publish_joint_states();

    return 0;

}
