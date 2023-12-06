#ifndef JTA_SERVER
#define JTA_SERVER

#include "ros/ros.h"
#include "serial_com_tlt.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

#include <chrono>
#include <mutex>

class JointTrajectoryActionServer
{
    public:
        enum ActionServerState
        {
            INIT=0,
            IDLE,
            SETUP,
            MOTION,
            COUNT
        };

        JointTrajectoryActionServer() = delete;
        JointTrajectoryActionServer(const std::string server_name, ros::NodeHandle &nh, SerialComTlt &srl);
        ~JointTrajectoryActionServer();

        ActionServerState getState() {return server_state_;};

        const char* ACTION_SERVER_STATE_NAMES[int(ActionServerState::COUNT)] =
        {
            "INIT",
            "IDLE",
            "SETUP",
            "MOTION"
        };

    private:
        // Action Server
        ros::NodeHandle nh_;
        std::string server_name_;
        SerialComTlt* srl_;

        actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> server_;
        actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_;
        control_msgs::FollowJointTrajectoryFeedback feedback_;

        std::chrono::system_clock::time_point start_time_;
        std::chrono::system_clock::time_point end_time_;

        std::mutex server_state_lock_;
        std::mutex action_notification_thread_lock_;
        ActionServerState server_state_;

        // Params
        double default_goal_time_tolerance_;
        double default_goal_tolerance_;
        std::vector<std::string> joint_names_ = {"ewellix_lift_top_joint"};
        std::string prefix_;

        // Action Server Callbacks
        void goalReceivedCb(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle);
        void preemptReceivedCb(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle);

        // Checks
        bool isGoalAcceptable(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle);
        bool isGoalToleranceRespected(bool check_time_tolerance);
        void stopAllMovement();

        void setServerState(ActionServerState s);
};
#endif
