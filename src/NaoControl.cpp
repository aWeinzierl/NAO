#include "NaoControl.h"

bool stop_thread = false;

void spinThread() {
    ros::Rate r(30);
    while (!stop_thread) {
        ros::spinOnce();
        r.sleep();
    }
}



NaoControl::NaoControl() : jointAnglesClient("/joint_angles_action", true) {
    // subscribe to topic joint_states and specify that all data will be processed by function sensorCallback
    sensor_data_sub = nh_.subscribe("/joint_states", 1, &NaoControl::sensorCallback, this);

    // subscribe to topic bumper and specify that all data will be processed by function bumperCallback
    bumper_sub = nh_.subscribe("/bumper", 1, &NaoControl::bumperCallback, this);

    // subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
    tactile_sub = nh_.subscribe("/tactile_touch", 1, &NaoControl::tactileCallback, this);

    stop_thread = false;
    spin_thread = new boost::thread(&spinThread);
}

NaoControl::~NaoControl() {
    stop_thread = true;
    sleep(1);
    spin_thread->join();
}

// this callback function provides information about nao feet bumpers
void NaoControl::bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr &bumperState) {

}

// this callback provides information about current head tactile buttons.
void NaoControl::tactileCallback(const naoqi_bridge_msgs::HandTouch::ConstPtr &tactileState) {


}

bool NaoControl::withinIntervalExclusive(const double value, const Interval &interval) {
    return interval.lowerLimit < value && interval.upperLimit > value;
}


// this function checks joint limits of the left arm. You need to provide JointState vector
bool NaoControl::check_joint_limits_left_arm(sensor_msgs::JointState joints) {
    std::vector<double> &positions = joints.position;

    return withinIntervalExclusive(positions.at(2), LEFT_SHOULDER_PITCH_LIMITS)
           && withinIntervalExclusive(positions.at(3), LEFT_SHOULDER_ROLL_LIMITS)
           && withinIntervalExclusive(positions.at(4), LEFT_ELBOW_YAW_LIMITS)
           && withinIntervalExclusive(positions.at(5), LEFT_ELBOW_ROLL_LIMITS)
           && withinIntervalExclusive(positions.at(6), LEFT_WRIST_YAW_LIMITS)
           && LEFT_HAND_STATES.end() != LEFT_HAND_STATES.find(positions.at(7));
}

// this function checks joint limits of the right arm. You need to provide JointState vector
bool NaoControl::check_joint_limits_right_arm(sensor_msgs::JointState joints) {
    std::vector<double> &positions = joints.position;

    return withinIntervalExclusive(positions.at(20), RIGHT_SHOULDER_PITCH_LIMITS)
           && withinIntervalExclusive(positions.at(21), RIGHT_SHOULDER_ROLL_LIMITS)
           && withinIntervalExclusive(positions.at(22), RIGHT_ELBOW_YAW_LIMITS)
           && withinIntervalExclusive(positions.at(23), RIGHT_ELBOW_ROLL_LIMITS)
           && withinIntervalExclusive(positions.at(24), RIGHT_WRIST_YAW_LIMITS)
           && RIGHT_HAND_STATES.end() != RIGHT_HAND_STATES.find(positions.at(25));
}

// this callback recives info about current joint states
void NaoControl::sensorCallback(const sensor_msgs::JointState::ConstPtr &jointState) {
    current_left_arm_state.name.clear();
    current_left_arm_state.position.clear();
    current_right_arm_state.name.clear();
    current_right_arm_state.position.clear();
    current_head_legs_state.name.clear();
    current_head_legs_state.position.clear();

    current_left_arm_state.header.stamp = ros::Time::now();

    current_left_arm_state.name.push_back(jointState->name.at(2));
    current_left_arm_state.position.push_back(jointState->position.at(2));
    current_left_arm_state.name.push_back(jointState->name.at(3));
    current_left_arm_state.position.push_back(jointState->position.at(3));
    current_left_arm_state.name.push_back(jointState->name.at(4));
    current_left_arm_state.position.push_back(jointState->position.at(4));
    current_left_arm_state.name.push_back(jointState->name.at(5));
    current_left_arm_state.position.push_back(jointState->position.at(5));
    current_left_arm_state.name.push_back(jointState->name.at(6));
    current_left_arm_state.position.push_back(jointState->position.at(6));

    current_right_arm_state.name.push_back(jointState->name.at(20));
    current_right_arm_state.position.push_back(jointState->position.at(20));
    current_right_arm_state.name.push_back(jointState->name.at(21));
    current_right_arm_state.position.push_back(jointState->position.at(21));
    current_right_arm_state.name.push_back(jointState->name.at(22));
    current_right_arm_state.position.push_back(jointState->position.at(22));
    current_right_arm_state.name.push_back(jointState->name.at(23));
    current_right_arm_state.position.push_back(jointState->position.at(23));
    current_right_arm_state.name.push_back(jointState->name.at(24));
    current_right_arm_state.position.push_back(jointState->position.at(24));

    current_head_legs_state.name.push_back(jointState->name.at(0));
    current_head_legs_state.position.push_back(jointState->position.at(0));
    current_head_legs_state.name.push_back(jointState->name.at(1));
    current_head_legs_state.position.push_back(jointState->position.at(1));
    for (int i = 7; i < 20; i++) {
        current_head_legs_state.name.push_back(jointState->name.at(i));
        current_head_legs_state.position.push_back(jointState->position.at(i));
    }
    current_head_legs_state.name.push_back(jointState->name.at(25));
    current_head_legs_state.position.push_back(jointState->position.at(25));
}


// this is main loop which should send commands to the nao arms.
void NaoControl::publish_joint_states() {
    /*
     * TODO tutorial
     */

    //example of moving LShoulderPitch joint to -0.56 angle
    //Prepare the goal message
    naoqi_bridge_msgs::JointAnglesWithSpeedGoal action_execute;
    action_execute.joint_angles.speed = 0.05;
    //this variable controls if the angles are relative (1) or not relative (0).
    action_execute.joint_angles.relative = 0;
    action_execute.joint_angles.joint_names.push_back("LShoulderPitch");
    action_execute.joint_angles.joint_angles.push_back(-0.56);
    //send goal message to the robot using actionlib client
    std::cout << "adada" << std::endl;

    ExecuteAction(action_execute);
}

void NaoControl::ExecuteActionAsync(naoqi_bridge_msgs::JointAnglesWithSpeedGoal action) {
    jointAnglesClient.sendGoal(action);
}

void NaoControl::ExecuteAction(
        naoqi_bridge_msgs::JointAnglesWithSpeedGoal action,
        ros::Duration timeOut) {
    ExecuteActionAsync(action);
    ros::Rate r_sleep(20);
    while (!jointAnglesClient.waitForResult(timeOut) && ros::ok()) {
        r_sleep.sleep();
    }
}