#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include <string.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HandTouch.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <std_srvs/Empty.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedGoal.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include <actionlib/client/simple_action_client.h>
using namespace std;

typedef actionlib::SimpleActionClient<naoqi_bridge_msgs::JointAnglesWithSpeedAction> JointAnglesClient;

bool stop_thread=false;

void spinThread()
{
    ros::Rate r(30);
    while(!stop_thread)
    {
        ros::spinOnce();
        r.sleep();
    }
}


class Nao_control
{
public:
    // ros handler
    ros::NodeHandle nh_;

    // subscriber to joint states
    ros::Subscriber sensor_data_sub;

    // subscriber to bumpers states
    ros::Subscriber bumper_sub;

    // subscriber to head tactile states
    ros::Subscriber tactile_sub;

    // variables which store current states of the joints
    sensor_msgs::JointState current_left_arm_state;
    sensor_msgs::JointState current_right_arm_state;
    sensor_msgs::JointState current_head_legs_state;

    //define actionlib client for joint angles control
    JointAnglesClient jointAnglesClient;

    boost::thread *spin_thread;

    Nao_control():jointAnglesClient("/joint_angles_action",true)
    {
        // subscribe to topic joint_states and specify that all data will be processed by function sensorCallback
        sensor_data_sub=nh_.subscribe("/joint_states",1, &Nao_control::sensorCallback, this);

        // subscribe to topic bumper and specify that all data will be processed by function bumperCallback
        bumper_sub=nh_.subscribe("/bumper",1, &Nao_control::bumperCallback, this);

        // subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
        tactile_sub=nh_.subscribe("/tactile_touch",1, &Nao_control::tactileCallback, this);

        stop_thread=false;
        spin_thread=new boost::thread(&spinThread);
    }
    ~Nao_control()
    {
        stop_thread=true;
        sleep(1);
        spin_thread->join();
    }

    // this callback function provides information about nao feet bumpers
    void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr& bumperState)
    {

    }

    // this callback provides information about current head tactile buttons.
    void tactileCallback(const naoqi_bridge_msgs::HandTouch::ConstPtr& tactileState)
    {


    }

    // this function checks joint limits of the left arm. You need to provide JointState vector
    bool check_joint_limits_left_arm(sensor_msgs::JointState joints)
    {
        bool check=true;

        /*
         * TODO tutorial
         */

        return check;
    }

    // this function checks joint limits of the right arm. You need to provide JointState vector
    bool check_joint_limits_right_arm(sensor_msgs::JointState joints)
    {
        bool check=true;
        /*
         * TODO tutorial
         */
        return check;
    }

    // this callback recives info about current joint states
    void sensorCallback(const sensor_msgs::JointState::ConstPtr& jointState)
    {
        current_left_arm_state.name.clear();
        current_left_arm_state.position.clear();
        current_right_arm_state.name.clear();
        current_right_arm_state.position.clear();
        current_head_legs_state.name.clear();
        current_head_legs_state.position.clear();

        current_left_arm_state.header.stamp=ros::Time::now();

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
        for(int i=7; i<20;i++)
        {
            current_head_legs_state.name.push_back(jointState->name.at(i));
            current_head_legs_state.position.push_back(jointState->position.at(i));
        }
        current_head_legs_state.name.push_back(jointState->name.at(25));
        current_head_legs_state.position.push_back(jointState->position.at(25));
    }


    // this is main loop which should send commands to the nao arms.
    void publish_joint_states()
    {
        /*
         * TODO tutorial
         */

        //example of moving LShoulderPitch joint to -0.56 angle
        //Prepare the goal message
        naoqi_bridge_msgs::JointAnglesWithSpeedGoal action_execute;
        action_execute.joint_angles.speed=0.05;
        //this variable controls if the angles are relative (1) or not relative (0).
        action_execute.joint_angles.relative=0;
        action_execute.joint_angles.joint_names.push_back("LShoulderPitch");
        action_execute.joint_angles.joint_angles.push_back(-0.56);
        //send goal message to the robot using actionlib client
        cout<<"adada"<<endl;
        jointAnglesClient.sendGoal(action_execute);

        //wait until the robot finish execution of current goal. The timeout is 2 seconds
        ros::Rate r_sleep(20);
        while((!jointAnglesClient.waitForResult(ros::Duration(2.0)))&& (ros::ok()))
        {
            r_sleep.sleep();
        }
    }

};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "nao_tutorial6");
    Nao_control ic;
    sleep(2);
    ic.publish_joint_states();

    return 0;

}
