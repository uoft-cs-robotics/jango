#include "ewellix_tlt/tlt_node.h"


TltNode::TltNode(ros::NodeHandle private_nh)
{
    // Publishers
    pub_joint_states_ = private_nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

    // Variables
    string port;
    int baudrate;

    // Parameters
    private_nh.param<string>("port", port, "/dev/ttyUSB0");
    private_nh.param<int>("baudrate", baudrate, 38400);

    // Services
    srv_init_sequence_ = private_nh.advertiseService("actions/init_sequence", &TltNode::srvInitSequence, this);

    // Subscribers
    sub_column_stop_ = private_nh.subscribe("actions/stop", 1, &TltNode::cbStop,this);
    sub_column_position_ = private_nh.subscribe("actions/position", 1, &TltNode::cbPosition,this);
    sub_column_duration_up_ = private_nh.subscribe("actions/duration_up", 1, &TltNode::cbDurationUp,this);
    sub_column_duration_down_ = private_nh.subscribe("actions/duration_down", 1, &TltNode::cbDurationDown,this);
    sub_motor1_ticks_ = private_nh.subscribe("actions/motor1_ticks", 1, &TltNode::cbMotor1Ticks, this);
    sub_motor2_ticks_ = private_nh.subscribe("actions/motor2_ticks", 1, &TltNode::cbMotor2Ticks, this);

    sub_joy_ = private_nh.subscribe("joy", 1, &TltNode::cbJoy,this);

    ROS_INFO("Connecting to port '%s' at baud rate '%d'", port.c_str(), baudrate);
    if(srl_.startSerialCom(port,baudrate)){

        vector<unsigned char> params;
        com_thread_ = thread(&SerialComTlt::comLoop,&srl_); //  RC thread
        join_states_thread_ = thread(&TltNode::publishJoinStates,this); //  RC thread

        if(srl_.startRs232Com()){        // Com started
            ROS_INFO("Remote function activated.");
        }
        else{
            ROS_ERROR("Remote function activation failed!");
            abort();
        }
    }
    else{
        ROS_ERROR("Failed to establish serial communication!");
        abort();
    }

    srv_follow_joint_trajectory_ = new JointTrajectoryActionServer("joint_trajectory_controller/follow_joint_trajectory", private_nh, srl_);
}

TltNode::~TltNode(){
    srl_.stopRs232Com();    //Com stopped
    srl_.run_= false;       // stop RC thread loop
    com_thread_.join();
    join_states_thread_.join();
}


void TltNode::publishJoinStates(){
    sensor_msgs::JointState joint_states;
    ros::Rate rateController = ros::Rate(20);
    while(ros::ok()) {
        joint_states.header.frame_id = "";
        joint_states.header.stamp = ros::Time::now();
        int size = 1;
        joint_states.name.resize(size);
        joint_states.position.resize(size);
        joint_states.velocity.resize(size);
        joint_states.effort.resize(size);
        joint_states.name[0] = "ewellix_lift_top_joint";
        joint_states.position[0] = srl_.getPosition();
        joint_states.velocity[0] = 0.0;
        joint_states.effort[0] = 0.1;
        pub_joint_states_.publish(joint_states);
        rateController.sleep();
    }
}

void TltNode::cbStop(std_msgs::Empty msg){
    srl_.motionStop();
    ROS_INFO("Stopping all motion.");
}

void TltNode::cbPosition(std_msgs::Float32 msg){
    srl_.motionQueuePositionGoal(msg.data, 100);
    ROS_INFO("Moving to '%0.2f' [m]", msg.data);
}

void TltNode::cbDurationUp(std_msgs::Float32 msg){
    srl_.motion_directed_ = 1;
    srl_.motion_duration_ = chrono::milliseconds(static_cast<int>(msg.data*1000));
    ROS_INFO("Moving up for '%2.2f' [sec]", msg.data);
}

void TltNode::cbDurationDown(std_msgs::Float32 msg){
    srl_.motion_directed_ = -1;
    srl_.motion_duration_ = chrono::milliseconds(static_cast<int>(msg.data*1000));
    ROS_INFO("Moving down for '%2.2f' [sec]", msg.data);
}

void TltNode::cbMotor1Ticks(std_msgs::Int16 msg){
    srl_.motionQueueTickGoal(msg.data, 1, 100);
    ROS_INFO("Moving motor 1 to '%d' ticks", msg.data);
}

void TltNode::cbMotor2Ticks(std_msgs::Int16 msg){
    srl_.motionQueueTickGoal(msg.data, 2, 100);
    ROS_INFO("Moving motor 2 to '%d' ticks", msg.data);
}

void TltNode::cbJoy( sensor_msgs::Joy msg){
    //srl_.go_up_ = msg.buttons[13];
    //srl_.go_down_ = msg.buttons[14];
}

bool TltNode::srvInitSequence(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res){
    ROS_INFO("srvInitSequence: beginning calibration.");
    srl_.calibrate_ = true;
    sleep(5);
    while(srl_.state_ == SerialComTlt::State::CALIB){
        sleep(10);
    }
    res.success = true;
    res.message = "Moved arm to max and min positions.";
    return true;
}


int main(int argc, char *argv[]){

    ros::init(argc, argv, "tlt_node");
    ros::NodeHandle nh, private_nh("~");

    TltNode n(private_nh);

    ros::spin();
    return 0;
}
