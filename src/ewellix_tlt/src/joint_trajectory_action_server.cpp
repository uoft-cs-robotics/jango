#include "ewellix_tlt/joint_trajectory_action_server.h"

JointTrajectoryActionServer::JointTrajectoryActionServer(const std::string server_name, ros::NodeHandle &nh, SerialComTlt &srl):
    server_name_(server_name),
    nh_(nh),
    server_(nh, server_name, boost::bind(&JointTrajectoryActionServer::goalReceivedCb, this, _1), boost::bind(&JointTrajectoryActionServer::preemptReceivedCb, this, _1), false),
    server_state_(ActionServerState::INIT)
{
    srl_ = &srl;
    server_.start();
    setServerState(ActionServerState::IDLE);
}

JointTrajectoryActionServer::~JointTrajectoryActionServer(){
    return;
}

void JointTrajectoryActionServer::goalReceivedCb(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle){
    ROS_INFO("Goal Received!");
    // check goal
    if (!isGoalAcceptable(goal_handle)){
        goal_handle.setRejected();
        return;
    }
    // stop previous goal
    if(server_state_ != ActionServerState::IDLE)
    {
        ROS_WARN("Already processing another goal. Cancelling previous.");
        stopAllMovement();
    }
    setServerState(ActionServerState::SETUP);
    // accept goal
    ROS_INFO("Goal Accepted!");
    goal_ = goal_handle;
    goal_.setAccepted();
    // push trajectory to queue
    auto points = goal_.getGoal()->trajectory.points;
    for(auto it = points.begin(); it != points.end(); it++){
        srl_->motionQueuePositionGoal(it->positions[0], 100);
    }
    srl_->motionQueuePrune();
    // wait to return back to idle
    bool processing = true;
    control_msgs::FollowJointTrajectoryResult result;
    sleep(1);
    while(processing){
        sleep(1);
        switch(srl_->state_){
            case SerialComTlt::State::MOTION:
                //ROS_INFO("Moving...");
                break;
            case SerialComTlt::State::IDLE:
                ROS_INFO("Finished.");
                result.error_code = result.SUCCESSFUL;
                goal_.setSucceeded(result);
                processing = false;
                break;
            case SerialComTlt::State::FAILURE:
                ROS_INFO("Failed.");
                result.error_code = result.INVALID_GOAL;
                goal_.setAborted(result);
                processing = false;
                break;
        }

    }
    setServerState(ActionServerState::IDLE);
}

void JointTrajectoryActionServer::preemptReceivedCb(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle){
    ROS_INFO("Goal Preempted!");
    //stopAllMovement();
}

bool JointTrajectoryActionServer::isGoalAcceptable(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle){
    if(!goal_handle.isValid()){
        ROS_INFO("Unaccetable goal. Goal is invalid.");
        return false;
    }

    control_msgs::FollowJointTrajectoryGoalConstPtr goal = goal_handle.getGoal();

    if(goal->trajectory.joint_names.size() != joint_names_.size()){
        ROS_INFO("Unacceptable goal. Joint number mismatch.");
        return false;
    }

    if (joint_names_ != goal->trajectory.joint_names)
    {
        ROS_INFO("Unacceptable goal. Joint names mismatch.");
        return false;
    }
    return true;
}

bool JointTrajectoryActionServer::isGoalToleranceRespected(bool check_time_tolerance){
    return true;
}

void JointTrajectoryActionServer::stopAllMovement(){
    srl_->motionStop();
}

void JointTrajectoryActionServer::setServerState(ActionServerState s){
    server_state_lock_.lock();
    server_state_ = s;
    server_state_lock_.unlock();
}
