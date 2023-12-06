#ifndef TLTNODE_H
#define TLTNODE_H

#include "joint_trajectory_action_server.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "serial_com_tlt.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include <string>
#include <thread>
#include <chrono>


class TltNode
{
    public:
        TltNode(ros::NodeHandle);
        ~TltNode();

    private:
        SerialComTlt srl_;
        thread com_thread_;
        thread join_states_thread_;
        // Actions
        JointTrajectoryActionServer* srv_follow_joint_trajectory_;

        // Publishers
        ros::Publisher pub_joint_states_;
        ros::Publisher pub_column_position_;

        // Services
        ros::ServiceServer srv_init_sequence_;

        // Subscribers
        ros::Subscriber sub_column_stop_;
        ros::Subscriber sub_column_position_;
        ros::Subscriber sub_column_duration_down_;
        ros::Subscriber sub_column_duration_up_;
        ros::Subscriber sub_joy_;
        ros::Subscriber sub_motor1_ticks_;
        ros::Subscriber sub_motor2_ticks_;

        // Subscriber Callback
        void cbStop(std_msgs::Empty);
        void cbPosition(std_msgs::Float32);
        void cbDurationUp(std_msgs::Float32);
        void cbDurationDown(std_msgs::Float32);
        void cbMotor1Ticks(std_msgs::Int16);
        void cbMotor2Ticks(std_msgs::Int16);
        void cbJoy(sensor_msgs::Joy);
        void publishJoinStates();

        // Service Calls
        bool srvInitSequence(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

};

#endif //TLTNODE_H
