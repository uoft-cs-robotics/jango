#include "ewellix_tlt/serial_com_tlt.h"

SerialComTlt::SerialComTlt(){
    // Comm Loop Control
    run_ = true;
    debug_ = false;
    stop_loop_ = false;
    com_started_ = false;

    // States
    state_ = SerialComTlt::State::IDLE;
    micro_state_ = SerialComTlt::MicroState::INIT;
    calib_state_ = SerialComTlt::CalibState::INIT;
    motion_state_ = SerialComTlt::MotionState::INIT;

    // Calib State Control
    calibrate_ = false;

    // Motion State Control
    motion_stop_ = false;
    motion_directed_ = 0;
    motion_duration_ = chrono::milliseconds(0);

    // Micro Variables
    mot1_ticks_ = 0;
    mot2_ticks_ = 0;
    mot1_status_ = std::vector<bool>(8, false);
    mot2_status_ = std::vector<bool>(8, false);
    mot1_percent_speed_ = 0;
    mot2_percent_speed_ = 0;
}


SerialComTlt::~SerialComTlt(){
    stopAll();
    com_started_=false;
    stop_loop_ = true;
    run_= false;
    if(serial_tlt_.isOpen()){
        serial_tlt_.close();
        cout << "SerialComTlt::~SerialComTlt - COM CLOSED !" << endl;
    }
}

/*Setup serial socket.*/
bool SerialComTlt::startSerialCom(string port, int baud_rate){
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);

    serial_tlt_.setPort(port);
    serial_tlt_.setBaudrate(baud_rate);
    serial_tlt_.setTimeout(timeout);

    try{
        serial_tlt_.open();
        cout << "SerialComTlt::startSerialCom - COM OPEN !" << endl;
        return true;
    }
    catch (serial::IOException e){
        cout << "SerialComTlt::startSerialCom - serial::IOException: " << e.what() << endl;
        return false;
    }

}

/*Activate the remote function.*/
bool SerialComTlt::startRs232Com(){

    vector<unsigned char> params = {0x01};
    sendCmd("RO",params);
    bool result = sendCmd("RO",params); // double call to wake up the column after long delay without com
    if(result){
        com_started_=true;
        stop_loop_=true;
        cout << "SerialComTlt::startRs232Com - Remote function activated" << endl;
        return true;
    }
    else{
        return false;
    }

}

/*Deactivate the remote function.*/
bool SerialComTlt::stopRs232Com(){
    com_started_=false;
    usleep(100);
    vector<unsigned char> params = {};
    bool result = sendCmd("RA",params);

    if(result){
        cout << "SerialComTlt::stopRs232Com - Remote function deactivated" << endl;
        return true;
    }
    else{
        return false;
    }
}

/*Serial command to get motor 1 encoder pose.*/
void SerialComTlt::getPoseM1(){
    sendCmd("RG",GET_POSE_M1);
    if (debug_) cout << "SerialComTlt::getPoseM1: " << mot1_ticks_ << endl;
}

/*Serial command to get motor 2 encoder pose.*/
void SerialComTlt::getPoseM2(){
    sendCmd("RG",GET_POSE_M2);
    if (debug_) cout << "SerialComTlt::getPoseM2: " << mot2_ticks_ << endl;
}

/*Serial command to get motor 1 percent speed.*/
void SerialComTlt::getPercentSpeedM1(){
    sendCmd("RG",GET_SPEED_M1);
    if (debug_) cout << "SerialComTlt::getPercentSpeedM1: " << mot1_percent_speed_ << endl;
}

/*Serial command to get motor 2 percent speed.*/
void SerialComTlt::getPercentSpeedM2(){
    sendCmd("RG",GET_SPEED_M2);
    if (debug_) cout << "SerialComTlt::getPercentSpeedM2: " << mot2_percent_speed_ << endl;
}

/*Serial command to get status of motor 1.*/
void SerialComTlt::getStatusM1(){
    sendCmd("RG",GET_STATUS_M1);
    if (debug_){
        cout << "SerialComTlt::getStatusM1: [";
        for(auto it = mot1_status_.begin(); it != mot1_status_.end(); it++){
            cout << *it << " ";
        }
        cout << "]" << endl;
    }
}

/*Serial command to get status of motor 2.*/
void SerialComTlt::getStatusM2(){
    sendCmd("RG",GET_STATUS_M2);
    if (debug_){
        cout << "SerialComTlt::getStatusM2: [";
        for(auto it = mot2_status_.begin(); it != mot2_status_.end(); it++){
            cout << *it << " ";
        }
        cout << "]" << endl;
    }
}

/*Serial command to set motor 1 encoder target pose.*/
void SerialComTlt::setPoseM1(unsigned int pose){
    std::vector<unsigned char> params;
    // Flip Integer
    std::vector<unsigned char> pose_hex = intToBytes(pose);
    // Append Command and Pose
    params.insert(params.end(), SET_POSE_M1.begin(), SET_POSE_M1.end());
    params.insert(params.end(), pose_hex.begin(), pose_hex.end());
    sendCmd("RT", params);
    if(debug_) cout << "setPoseM1: " << mot1_ticks_ << endl;
}

/*Serial command to set motor 2 encoder target pose.*/
void SerialComTlt::setPoseM2(unsigned int pose){
    std::vector<unsigned char> params;
    // Flip Integer
    std::vector<unsigned char> pose_hex = intToBytes(pose);
    // Append Command and Pose
    params.insert(params.end(), SET_POSE_M2.begin(), SET_POSE_M2.end());
    params.insert(params.end(), pose_hex.begin(), pose_hex.end());
    sendCmd("RT", params);
    if(debug_) cout << "setPoseM2: " << mot2_ticks_ << endl;
}

/*Serial command to set motor 1 percent speed.*/
void SerialComTlt::setPercentSpeedM1(unsigned int percent){
    //if(percent > 100) percent = 100;
    if(percent < 0) percent = 0;
    std::vector<unsigned char> params;
    std::vector<unsigned char> percent_hex = intToBytes(percent);
    // Append Command and Percent Speed
    params.insert(params.end(), SET_SPEED_M1.begin(), SET_SPEED_M1.end());
    params.insert(params.end(), percent_hex.begin(), percent_hex.begin() + 2);
    sendCmd("RT",params);
    if(debug_) cout << "setPercentSpeedM1: " << mot1_percent_speed_ << endl;
}

/*Serial command to set motor 2 percent speed.*/
void SerialComTlt::setPercentSpeedM2(unsigned int percent){
    //if(percent > 100) percent = 100;
    if(percent < 0) percent = 0;
    std::vector<unsigned char> params;
    std::vector<unsigned char> percent_hex = intToBytes(percent);
    // Append Command and Percent Speed
    params.insert(params.end(), SET_SPEED_M2.begin(), SET_SPEED_M2.end());
    params.insert(params.end(), percent_hex.begin(), percent_hex.begin() + 2);
    sendCmd("RT",params);
    if(debug_) cout << "setPercentSpeedM2: " << mot2_percent_speed_ << endl;
}

/*Serial commands to set both motor percent speeds.*/
void SerialComTlt::setPercentSpeedAll(unsigned int percent){
    setPercentSpeedM1(percent);
    setPercentSpeedM2(percent);
}

/*Serial command to retract motor 1.*/
void SerialComTlt::moveM1Down(){
    sendCmd("RE",MOVE_M1_DOWN);
    if(debug_) cout << "moveM1Down" << endl;
}

/*Serial command to extend motor 1.*/
void SerialComTlt::moveM1Up(){
    sendCmd("RE",MOVE_M1_UP);
    if(debug_) cout << "moveM1Up" << endl;
}

/*Serial command to move motor 1 to previously set user pose.*/
void SerialComTlt::moveM1Pose(){
    sendCmd("RE",MOVE_M1_POSE);
    if(debug_) cout << "moveM1Pose" << endl;
}

/*Serial command to retract motor 2*/
void SerialComTlt::moveM2Down(){
    sendCmd("RE",MOVE_M2_DOWN);
    if(debug_) cout << "moveM2Down" << endl;
}

/*Serial command to extend motor 2.*/
void SerialComTlt::moveM2Up(){
    sendCmd("RE",MOVE_M2_UP);
    if(debug_) cout << "moveM2Up" << endl;
}

/*Serial command to move motor 1 to previously set user pose.*/
void SerialComTlt::moveM2Pose(){
    sendCmd("RE",MOVE_M2_POSE);
    if(debug_) cout << "moveM2Pose" << endl;
}

/*Serial command to retract all motors.*/
void SerialComTlt::moveAllDown(){
    sendCmd("RE",MOVE_ALL_DOWN);
    if(debug_) cout << "moveAllDown" << endl;
}

/*Serial command to extend all motors.*/
void SerialComTlt::moveAllUp(){
    sendCmd("RE",MOVE_ALL_UP);
    if(debug_) cout << "moveAllUp" << endl;
}

/*Serial command to move all motors to previously set user pose.*/
void SerialComTlt::moveAllPose(){
    sendCmd("RE",MOVE_ALL_POSE);
    if(debug_) cout << "moveAllPose" << endl;
}

/*Serial command to stop motor 1.*/
void SerialComTlt::stopM1(){
    sendCmd("RS",STOP_M1_FAST);
    if (debug_) cout << "SerialComTlt:stopMot1" << endl;
}

/*Serial command to stop motor 2.*/
void SerialComTlt::stopM2(){
    sendCmd("RS",STOP_M2_FAST);
    if (debug_) cout << "SerialComTlt:stopMot2" << endl;
}

/*Serial command to stop all motors.*/
void SerialComTlt::stopAll(){
    sendCmd("RS",STOP_ALL_FAST);
    if (debug_)cout << "SerialComTlt::stopMotAll" << endl;
}

/*Compute checksum.*/
unsigned short SerialComTlt::calculateChecksum(vector<unsigned char> *cmd){
    unsigned short crc = 0;
    for(vector<unsigned char>::iterator it=cmd->begin(); it!=cmd->end(); ++it){
        crc = static_cast<unsigned short>(CRC_TABLE[((crc >> 8) ^ *it) & 0xFF] ^ (crc << 8));
    }
    return crc;
}

/*Compare response checksum with calculated checksum.*/
bool SerialComTlt::checkResponseChecksum(vector<unsigned char> *response){
    unsigned short response_checksum_lsb = *(response->end()-2);
    unsigned short response_checksum_msb = *(response->end()-1);

    if (debug_){
        unsigned short checksum = (response_checksum_lsb<<8) | response_checksum_msb;
        stringstream checksum_hex;
        checksum_hex << hex << checksum;
        cout << "SerialComTlt::checkResponseChecksum - Response Checksum = "<< checksum_hex.str() << endl;
    }

    vector<unsigned char> response_msg = *response;
    response_msg.resize(response_msg.size()-2);

    unsigned short checksum = calculateChecksum(&response_msg);
    unsigned short computed_lsb = checksum &0x00FF;
    unsigned short computed_msb = checksum >> 8;

    if (debug_){
        unsigned short checksum2 = (computed_lsb<<8) | computed_msb;
        stringstream checksum_hex;
        checksum_hex << hex << checksum2;
        cout << "SerialComTlt::checkResponseChecksum - Computed Checksum = "<< checksum_hex.str() << endl;
    }

    if(response_checksum_lsb == computed_lsb && response_checksum_msb == computed_msb){
        return true;
    }
    else{
        return false;
    }

    return false;
}

/*Check response error code.*/
bool SerialComTlt::checkResponseAck(vector<unsigned char> *response){
    if(response->size()>4){
        unsigned short cmd_status = *(response->begin()+2);
        if(cmd_status == 0x06 ){
            return true;
        }
        else if (cmd_status == 0x81 ){
            cout << "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 81 Parameter data error "<< endl;
        }
        else if (cmd_status == 0x82 ){
            cout << "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 82 Parameter count error "<< endl;
        }
        else if (cmd_status == 0x83 ){
            cout << "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 83 Command error "<< endl;
        }
        else if (cmd_status == 0x84 ){
            cout << "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 84 Permission error "<< endl;
        }
    }
    else{
        return false;
    }
    return false;
}

/*Read response byte by byte.*/
vector<unsigned char> SerialComTlt::feedback(vector<unsigned char> sent_data){
    int i = 0;
    int timeout =0;
    vector<unsigned char> received_data;
    string last_data;
    string command_type="";
    string sent_command_type({static_cast<char>(sent_data[1]), '\0'});
    int msg_size = -1;
    bool success = false;
    bool first_byte = false;

    while (!stop_loop_)
    {
        // Check if serial available
        if (serial_tlt_.available()){
            last_data = serial_tlt_.read();

            if(!first_byte && last_data =="R"){  // Beginning of the message
                first_byte = true;
                for (const auto &item : last_data) {
                    received_data.push_back(int(item));  // convert cmd string in hex value
                }
            }
            if(first_byte){
                i++;
                if(i>1) {
                    for (const auto &item : last_data) {
                        received_data.push_back(int(item));  // convert cmd string in hex value
                    }
                }

                // command detector
                if(i==2){
                    command_type = last_data;
                    if (debug_) cout << "SerialComTlt::feedback::ResponseCommand: R"<< command_type << endl;
                    if(command_type[0] != sent_command_type[0]){
                       cout << "SerialComTlt::feedback::Error: Response command type: " << command_type << " does not match sent command: " << sent_command_type << endl;
                    }
                }

                // success detector
                if( i==3 && last_data==""){ // ACK
                    success = true;
                }

                if(command_type =="G")  {
                    if (i == 4 && success) {

                        for (const auto &item : last_data) {
                            msg_size = 7 + int(item);
                        }
                    }
                    else if(i == 4 && !success) {
                        msg_size = 5;
                    }
                }
                else if(command_type =="T" ||command_type =="C"||command_type =="E"||command_type =="S"||command_type =="O"||command_type =="A")  {
                    msg_size = 5;
                }
                else{
                    msg_size = 5;
                }

                if(msg_size > 0 && i == msg_size){
                    stop_loop_ = true;
                    break;
                }
            }
        }
        else{
            usleep(10);
            timeout++;
            if( timeout > 5000){
                stop_loop_ = true;
                return received_data;
            }
        }
    }
    return received_data;
}

/*Extract pose from comm response*/
bool SerialComTlt::extractPose(vector<unsigned char> response, int mot){
    unsigned int position = response[6] << 8 | response[5];

    if(mot == 1) mot1_ticks_ = position;
    else if(mot == 2) mot2_ticks_ = position;
    else return false;

    return true;
}
/*Extract percent speed from comm response.*/
bool SerialComTlt::extractPercentSpeed(vector<unsigned char> response, int mot){
    unsigned int speed = response[6] << 8 | response[5];

    if(mot == 1) mot1_percent_speed_ = speed;
    else if(mot == 2) mot2_percent_speed_ = speed;
    else return false;

    return true;
}
/*Extract status from serial comm response.*/
bool SerialComTlt::extractStatus(vector<unsigned char> response, int mot){
    if(!(mot == 1 || mot == 2)) return false;
    unsigned char status = response[5];

    for(int i=0; i<8; i++){
        if(mot == 1) mot1_status_[i] = static_cast<bool>(status & (0x01 << i));
        if(mot == 2) mot2_status_[i] = static_cast<bool>(status & (0x01 << i));
    }

    return true;
}

/*Converte integer to bytes. Big Endian.*/
vector<unsigned char> SerialComTlt::intToBytes(unsigned int paramInt){
    vector<unsigned char> arrayOfByte(4);
    for (int i = 0; i < 4; i++)
        arrayOfByte[i] = static_cast<unsigned char>(paramInt >> (i * 8));
    return arrayOfByte;
}

/*
Send Command:
 - convert command to bytes
 - compute checksum
 - write to serial com
 - extract results
*/
bool SerialComTlt::sendCmd(string cmd, vector<unsigned char> param){
    // Lock
    lock_.lock();
    if(debug_) cout << endl << "--------------------" << "sendCmd::Begin" << "--------------------" << endl;

    // Convert String Command to Hex
    vector<unsigned char> final_cmd;
    if (debug_) cout << "SerialComTlt::sendCmd::InputCommand: ";
    for (const auto &item : cmd) {
        if (debug_) cout << item;
        final_cmd.push_back(int(item));  // convert cmd string in hex value
    }

    // Add Parameters to Command
    if (debug_) cout <<" [";
    if (!param.empty()){
        for(vector<unsigned char>::iterator it=param.begin(); it!=param.end(); ++it){
            if (debug_) cout <<' ' <<  int(*it);
            final_cmd.push_back(*it);       // add params to the cmd
        }
    }
    if (debug_) cout <<" ]"<< endl;

    // Compute checksum
    unsigned short checksum = calculateChecksum(&final_cmd);
    unsigned short lsb = checksum &0x00FF;
    unsigned short msb = checksum >> 8;

    final_cmd.push_back(lsb);
    final_cmd.push_back(msb);

    if (debug_){
        stringstream final_cmd_hex;
        for (unsigned short j: final_cmd){
            final_cmd_hex <<  hex << j << ' ';
        }
        cout << "SerialComTlt::sendCmd::OutputCommand: " << final_cmd_hex.str()<< endl;
    }

    // Send Command
    if ( serial_tlt_.isOpen() ){
        try{
            serial_tlt_.write(final_cmd);
            usleep( 1 );
            serial_tlt_.flush();
            stop_loop_= false;
        }
        catch (serial::IOException e){
            cout << "SerialComTlt::sendCmd - serial::IOException: " << e.what() << endl;
        }
    }
    //usleep(10); // wait for response

    // Receive Response
    vector<unsigned char> output = feedback(final_cmd);
     if(output.size() == 0){
        cout << "SerialComTlt::sendCmd::ResponseError: Empty" << endl;
        lock_.unlock();
        return false;
    }
    if (debug_){
        stringstream output_hex;
        for (unsigned short j: output){
            output_hex <<  hex << j << ' ';
        }
        cout << "SerialComTlt::sendCmd::Response: " << output_hex.str()<< endl;
    }

    // Checksum Failed, Try Again
    if(!checkResponseChecksum(&output) || !checkResponseAck(&output)){
        cout << "SerialComTlt::sendCmd::Failed: retrying..." << endl;
        serial_tlt_.flush();
        usleep(300);
        serial_tlt_.write(final_cmd);

        output = feedback(final_cmd);
        if (debug_){
            stringstream output_hex;
            for (unsigned short j: output){
                output_hex <<  hex << j << ' ';
            }
            cout << "SerialComTlt::sendCmd::Response: " << output_hex.str()<< endl;
        }
        if(output.size() == 0 || !checkResponseChecksum(&output) || !checkResponseAck(&output)){
            cout << "SerialComTlt::sendCmd::Failed: AGAIN!" << endl;
            lock_.unlock();
            startRs232Com();
            return false;
        }
    }

    // Extract Information
    if(cmd =="RG"){
        // get pose
        if(param == GET_POSE_M1) extractPose(output,1);
        else if (param == GET_POSE_M2) extractPose(output,2);
        // get speed
        else if (param == GET_SPEED_M1) extractPercentSpeed(output,1);
        else if (param == GET_SPEED_M2) extractPercentSpeed(output,2);
        // get status
        else if (param == GET_STATUS_M1) extractStatus(output,1);
        else if (param == GET_STATUS_M2) extractStatus(output,2);
    }
    if(debug_) cout << "--------------------" << "sendCmd::End" << "--------------------" << endl << endl;

    // Unlock
    lock_.unlock();
    return true;
}

/*Convert position [m] to encoder ticks.*/
void SerialComTlt::convertPosition2Ticks(float position, unsigned int* mot1_ticks, unsigned int* mot2_ticks){
    // check bounds
    if(position > ALL_MOTOR_METERS) position = ALL_MOTOR_METERS;
    if(position < 0) position = 0;
    // convert position to ticks
    float mot1_meters = MOTOR1_METER_RATIO * position;
    float mot2_meters = MOTOR2_METER_RATIO * position;

    *mot1_ticks = static_cast<int>(mot1_meters * MOTOR1_METERS_TO_TICKS);
    *mot2_ticks = static_cast<int>(mot2_meters * MOTOR2_METERS_TO_TICKS);

    *mot1_ticks = *mot1_ticks + MOTOR1_TICK_OFFSET;
    *mot2_ticks = *mot2_ticks + MOTOR2_TICK_OFFSET;

    if(*mot1_ticks > MOTOR1_TICKS) *mot1_ticks = MOTOR1_TICKS;
    if(*mot2_ticks > MOTOR2_TICKS) *mot2_ticks = MOTOR2_TICKS;
}

/*Convert encoder ticks to position [m].*/
void SerialComTlt::convertTicks2Position(unsigned int mot1_ticks, unsigned int mot2_ticks, float* position){
    int mot1_norm_pose = mot1_ticks - MOTOR1_TICK_OFFSET;
    if(mot1_norm_pose < 0) mot1_norm_pose = 0;

    int mot2_norm_pose = mot2_ticks - MOTOR2_TICK_OFFSET;
    if(mot2_norm_pose < 0) mot2_norm_pose = 0;

    float mot1_meters = static_cast<float>(mot1_norm_pose * MOTOR1_TICKS_TO_METERS);
    float mot2_meters = static_cast<float>(mot2_norm_pose * MOTOR2_TICKS_TO_METERS);

    *position = mot1_meters + mot2_meters;
}

/*Get current lift position in meters*/
float SerialComTlt::getPosition(){
    float position;
    convertTicks2Position(mot1_ticks_, mot2_ticks_, &position);
    return position;
}

/*Set goal lift position in meters*/
void SerialComTlt::setPosition(float position){
    unsigned int mot1_ticks;
    unsigned int mot2_ticks;
    convertPosition2Ticks(position, &mot1_ticks, &mot2_ticks);
    setPoseM1(mot1_ticks);
    setPoseM2(mot2_ticks);
}

/*Check if goal has been reached*/
bool SerialComTlt::isAtGoal(SerialComTlt::MotionGoal goal){
    return isAtTicks(goal.mot1_ticks, goal.mot2_ticks);
}

/*Check if motors are at specified position*/
bool SerialComTlt::isAtPosition(float position){
    unsigned int mot1_ticks;
    unsigned int mot2_ticks;
    convertPosition2Ticks(position, &mot1_ticks, &mot2_ticks);
    return isAtTicks(mot1_ticks, mot2_ticks);
}

/*Check if motors are at specified ticks*/
bool SerialComTlt::isAtTicks(unsigned int ticks1, unsigned int ticks2){
    bool mot1_at_ticks = (
        mot1_ticks_ >= (ticks1 - TICK_ERROR_MARGIN)) && (
        mot1_ticks_ <= (ticks1 + TICK_ERROR_MARGIN));
    bool mot2_at_ticks = (
        mot2_ticks_ >= (ticks2 - TICK_ERROR_MARGIN)) && (
        mot2_ticks_ <= (ticks2 + TICK_ERROR_MARGIN));
    return mot1_at_ticks && mot2_at_ticks;
}

/*Check if motors full retracted*/
bool SerialComTlt::isRetracted(){
    return isAtTicks(MOTOR1_TICK_OFFSET, MOTOR2_TICK_OFFSET);
}

/*Check if motors full extended*/
bool SerialComTlt::isExtended(){
    return isAtTicks(MOTOR1_TICKS, MOTOR2_TICKS);
}

/*Check if above specified ticks*/
bool SerialComTlt::isBelowTicks(unsigned int ticks1, unsigned int ticks2){
    bool mot1_below_ticks = mot1_ticks_ <= ticks1;
    bool mot2_below_ticks = mot2_ticks_ <= ticks2;
    return mot1_below_ticks && mot2_below_ticks;
}

/*Check if below specified ticks*/
bool SerialComTlt::isAboveTicks(unsigned int ticks1, unsigned int ticks2){
    bool mot1_above_ticks = mot1_ticks_ >= ticks1;
    bool mot2_above_ticks = mot2_ticks_ >= ticks2;
    return mot1_above_ticks && mot2_above_ticks;
}

/*Check if Drive 1 Available*/
bool SerialComTlt::isDriveAvailableM1(){
    return(mot1_status_[0]);
}

/*Check if Drive 2 Available*/
bool SerialComTlt::isDriveAvailableM2(){
    return(mot2_status_[0]);
}

/*Check if Drives Available*/
bool SerialComTlt::isDriveAvailable(){
    return(mot1_status_[0] && mot2_status_[0]);
}

/*Check if Motor 1 Motion Active*/
bool SerialComTlt::isMotionActiveM1(){
    return(mot1_status_[4]);
}

/*Check if Motor 2 Motion Active*/
bool SerialComTlt::isMotionActiveM2(){
    return(mot2_status_[4]);
}

/*Check if Motors Motion Active*/
bool SerialComTlt::isMotionActive(){
    return(mot1_status_[4] || mot2_status_[4]);
}

/*Check if Motor 1 Position Reached*/
bool SerialComTlt::isPositionReachedM1(){
    return(mot1_status_[5]);
}

/*Check if Motor 2 Position Reached*/
bool SerialComTlt::isPositionReachedM2(){
    return(mot2_status_[5]);
}

/*Check if Position Reached*/
bool SerialComTlt::isPositionReached(){
    return(mot1_status_[5] && mot2_status_[5]);
}

/*FSM initialization state. Requires calibration to move to next step.*/
SerialComTlt::State SerialComTlt::initState(){
    // Calibration
    if(calibrate_){
        calibrate_ = false;
        return SerialComTlt::State::CALIB;
    }
    return SerialComTlt::State::INIT;
}

/*Calibration procedure to find speed in [m/s]*/
bool SerialComTlt::calibProcedure(unsigned int direction,unsigned int speed,unsigned int m1_goal,unsigned int m2_goal,float* speed_result){
    m1_goal = static_cast<unsigned int>(std::round(m1_goal));
    m2_goal = static_cast<unsigned int>(std::round(m2_goal));
    switch(micro_state_){
        case SerialComTlt::MicroState::INIT:
            // Set Speed
            setPercentSpeedAll(speed);
            // Start Timer
            calib_start_time_ = std::chrono::steady_clock::now();
            // Start Ticks
            calib_start_ticks_ = mot1_ticks_ + mot2_ticks_;
            // Start Moving
            if(direction) moveAllUp();
            else moveAllDown();
            // Next State
            next_micro_state_ = SerialComTlt::MicroState::WAIT;
            break;
        case SerialComTlt::MicroState::WAIT:
            // Check
            if(isAtTicks(m1_goal, m2_goal) ||
                (direction && isAboveTicks(m1_goal, m2_goal)) ||
                (!direction && isBelowTicks(m1_goal, m2_goal))){
                // Stop Moving
                stopAll();
                // Stop Timer;
                calib_end_time_ = std::chrono::steady_clock::now();
                // End Ticks
                calib_end_ticks_ = mot1_ticks_ + mot2_ticks_;
                // Next State
                next_micro_state_ = SerialComTlt::MicroState::END;
                break;
            }
            else{
                next_micro_state_ = SerialComTlt::MicroState::WAIT;
                break;
            }
        case SerialComTlt::MicroState::END:
            // Calculate time elapsed
            int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(calib_end_time_ - calib_start_time_).count();
            // Calculate ticks elapsed
            int tick_elapsed = calib_end_ticks_ - calib_start_ticks_;
            // Calculate Speed
            if(speed_result) *speed_result = std::abs(tick_elapsed*1000/time_elapsed);
            // Exit
            micro_state_ = next_micro_state_ = SerialComTlt::MicroState::INIT;
            return true;
    }
    micro_state_ = next_micro_state_;
    return false;
}

/*FSM calibration state; fully retracts, fully extends, then retracts again.*/
SerialComTlt::State SerialComTlt::calibState(){
    switch(calib_state_)
    {
        case SerialComTlt::CalibState::INIT:
            next_calib_state_ = SerialComTlt::CalibState::RETRACT;
            break;
        case SerialComTlt::CalibState::RETRACT:
            if(calibProcedure(0, 65535, MOTOR1_TICK_OFFSET, MOTOR2_TICK_OFFSET, nullptr)){
                next_calib_state_ = SerialComTlt::CalibState::EXTEND_LOWER;
            }
            else
                next_calib_state_ = calib_state_;
            break;
        case SerialComTlt::CalibState::EXTEND_LOWER:
            if(calibProcedure(1, 1, MOTOR1_TICKS/2, MOTOR2_TICKS/2, &min_speed_up_)){
                next_calib_state_ = SerialComTlt::CalibState::EXTEND_UPPER;
            }
            else
                next_calib_state_ = calib_state_;
            break;
        case SerialComTlt::CalibState::EXTEND_UPPER:
            if(calibProcedure(1, 999, MOTOR1_TICKS, MOTOR2_TICKS, &max_speed_up_)){
                next_calib_state_ = SerialComTlt::CalibState::RETRACT_UPPER;
            }
            else
                next_calib_state_ = calib_state_;
            break;
        case SerialComTlt::CalibState::RETRACT_UPPER:
            if(calibProcedure(0, 1, MOTOR1_TICKS/2, MOTOR2_TICKS/2, &min_speed_down_)){
                next_calib_state_ = SerialComTlt::CalibState::RETRACT_LOWER;
            }
            else
                next_calib_state_ = calib_state_;
            break;
        case SerialComTlt::CalibState::RETRACT_LOWER:
            if(calibProcedure(0, 999, MOTOR1_TICK_OFFSET, MOTOR2_TICK_OFFSET, &max_speed_down_)){
                next_calib_state_ = SerialComTlt::CalibState::EXIT;
            }
            else
                next_calib_state_ = calib_state_;
            break;
        case SerialComTlt::CalibState::EXIT:
            calib_state_ = next_calib_state_ = SerialComTlt::CalibState::INIT;
            return SerialComTlt::State::IDLE;
    }
    calib_state_ = next_calib_state_;
    return SerialComTlt::State::CALIB;
}

/*FSM idle state, moves to motion or calibration states.*/
SerialComTlt::State SerialComTlt::idleState(){
    // Calibration
    if(calibrate_){
        calibrate_ = false;
        return SerialComTlt::State::CALIB;
    }
    // Motion
    if(!motion_queue_.empty() || motion_directed_){
        return SerialComTlt::State::MOTION;
    }
    return SerialComTlt::State::IDLE;
}

/*Immediately Stop Motion*/
void SerialComTlt::motionStop(){
    stopAll();
    motionQueueClear();
    motion_stop_ = true;
}

/*Queue up a MotionGoal using motor ticks and [m/s].*/
void SerialComTlt::motionQueueGoal(unsigned int mot1_ticks, unsigned int mot2_ticks, float speed){
    MotionGoal goal;
    goal.mot1_ticks = mot1_ticks;
    goal.mot2_ticks = mot2_ticks;
    goal.speed = speed;
    motion_queue_lock_.lock();
    motion_queue_.push(goal);
    motion_queue_lock_.unlock();
}

/*Queue up a MotionGoal using a single motor's ticks.*/
void SerialComTlt::motionQueueTickGoal(unsigned int mot_ticks, unsigned int mot, float speed){
    MotionGoal goal;
    if(mot == 1){
        goal.mot1_ticks = mot_ticks;
        goal.mot2_ticks = -1;
    }
    else if (mot == 2){
        goal.mot1_ticks = -1;
        goal.mot2_ticks = mot_ticks;
    }
    else return;
    goal.speed = speed;
    motion_queue_lock_.lock();
    motion_queue_.push(goal);
    motion_queue_lock_.unlock();
}

/*Queue up a MotionGoal using a position in meters.*/
void SerialComTlt::motionQueuePositionGoal(float position, float speed){
    unsigned int mot1_ticks, mot2_ticks;
    convertPosition2Ticks(position, &mot1_ticks, &mot2_ticks);
    MotionGoal goal;
    goal.mot1_ticks = mot1_ticks;
    goal.mot2_ticks = mot2_ticks;
    goal.speed = speed;
    motion_queue_lock_.lock();
    motion_queue_.push(goal);
    motion_queue_lock_.unlock();
}

/*Clear queue*/
void SerialComTlt::motionQueueClear(){
    std::queue<SerialComTlt::MotionGoal> empty;
    motion_queue_lock_.lock();
    std::swap(motion_queue_, empty);
    motion_queue_lock_.unlock();
}

/*Prune queue*/
void SerialComTlt::motionQueuePrune(){
    motion_queue_lock_.lock();
    std::queue<SerialComTlt::MotionGoal> pruned;
    SerialComTlt::MotionGoal prev = motion_queue_.front();
    motion_queue_.pop();
    SerialComTlt::MotionGoal curr = motion_queue_.front();
    bool direction = curr.mot1_ticks >= prev.mot1_ticks;
    bool new_direction;
    pruned.push(prev);
    while(!motion_queue_.empty()){
        curr = motion_queue_.front();
        if(curr.mot1_ticks == prev.mot1_ticks){
            new_direction = direction;
        }
        else{
            curr.mot1_ticks >= prev.mot1_ticks;
        }
        // Same
        if(((
            curr.mot1_ticks <= (prev.mot1_ticks + TICK_ERROR_MARGIN)) && (
            curr.mot1_ticks >= (prev.mot1_ticks - TICK_ERROR_MARGIN))) && ((
            curr.mot2_ticks <= (prev.mot2_ticks + TICK_ERROR_MARGIN)) && (
            curr.mot2_ticks >= (prev.mot2_ticks - TICK_ERROR_MARGIN)))){
            motion_queue_.pop();
        }
        else if (direction && new_direction)
        {
            motion_queue_.pop();
        }
        else{
            pruned.push(curr);
            prev = curr;
            motion_queue_.pop();
            direction = new_direction;
        }
        if(motion_queue_.empty()){
            pruned.push(curr);
        }
    }
    std::swap(motion_queue_, pruned);
    motion_queue_lock_.unlock();
}

/*Move to a MotionGoal*/
bool SerialComTlt::motionPoseProcedure(MotionGoal goal){
    switch(micro_state_){
        case SerialComTlt::MicroState::INIT:
            // Set Position
            setPoseM1(goal.mot1_ticks);
            setPoseM2(goal.mot2_ticks);
            // Set Speed TODO
            // Start Moving
            moveAllPose();
            // Next State
            next_micro_state_ = SerialComTlt::MicroState::WAIT;
            break;
        case SerialComTlt::MicroState::WAIT:
            if(motion_stop_){
                next_micro_state_ = SerialComTlt::MicroState::END;
            }
            // Check
            if(isAtGoal(goal) || isPositionReached() || !isMotionActive()){
                // Stop Moving
                stopAll();
                // Next State
                next_micro_state_ = SerialComTlt::MicroState::END;
            }
            else{
                next_micro_state_ = SerialComTlt::MicroState::WAIT;
            }
            break;
        case SerialComTlt::MicroState::END:
            micro_state_ = next_micro_state_ = SerialComTlt::MicroState::INIT;
            motion_stop_ = false;
            return true;
    }
    micro_state_ = next_micro_state_;
    return false;
}

/*Move in direction for a given time.*/
bool SerialComTlt::motionDirectedProcedure(int direction, chrono::milliseconds duration){
    switch(micro_state_){
        case SerialComTlt::MicroState::INIT:
            motion_start_time_ = chrono::steady_clock::now();
            if(direction > 0) moveAllUp();
            if(direction < 0) moveAllDown();
            next_micro_state_ = SerialComTlt::MicroState::WAIT;
            break;
        case SerialComTlt::MicroState::WAIT:
            if(motion_stop_){
                next_micro_state_ = SerialComTlt::MicroState::END;
            }
            if(!duration.count()){
                break;
            }
            motion_end_time_ = chrono::steady_clock::now();
            chrono::milliseconds elapsed;
            elapsed = chrono::duration_cast<chrono::milliseconds>(motion_end_time_ - motion_start_time_);
            if(elapsed.count() >= duration.count()){
                stopAll();
                next_micro_state_ = SerialComTlt::MicroState::END;
            }
            else next_micro_state_ = micro_state_;
            break;
        case SerialComTlt::MicroState::END:
            micro_state_ = next_micro_state_ = SerialComTlt::MicroState::INIT;
            motion_stop_ = false;
            return true;
    }
    micro_state_ = next_micro_state_;
    return false;
}

/*FSM motion state: pops MotionGoals from queue or moves in a direction specified.*/
SerialComTlt::State SerialComTlt::motionState(){
    switch(motion_state_){
        case SerialComTlt::MotionState::INIT:
            if(motion_directed_){
                // Clear Queue
                motionQueueClear();
                next_motion_state_ = SerialComTlt::MotionState::MOTION_DIRECTED;
                break;
            }
            if(!motion_queue_.empty()){
                motion_goal_ = motion_queue_.front();
                if(motion_goal_.mot1_ticks < 0) motion_goal_.mot1_ticks = mot1_ticks_;
                if(motion_goal_.mot2_ticks < 0) motion_goal_.mot2_ticks = mot2_ticks_;
                motion_queue_.pop();
                next_motion_state_ = SerialComTlt::MotionState::MOTION_POSE;
                break;
            }
            else{
                next_motion_state_ = SerialComTlt::MotionState::EXIT;
            }
            break;
        case SerialComTlt::MotionState::MOTION_DIRECTED:
            if(motionDirectedProcedure(motion_directed_, motion_duration_)){
                motion_directed_ = 0;
                motion_duration_ = chrono::milliseconds(0);
                next_motion_state_ = SerialComTlt::MotionState::EXIT;
            }
            else next_motion_state_ = motion_state_;
            break;
        case SerialComTlt::MotionState::MOTION_POSE:
            if(motionPoseProcedure(motion_goal_)){
                if(!motion_queue_.empty())
                    next_motion_state_ = SerialComTlt::MotionState::INIT;
                else
                    next_motion_state_ = SerialComTlt::MotionState::EXIT;
            }
            break;
        case SerialComTlt::MotionState::EXIT:
            motion_state_ = next_motion_state_ = SerialComTlt::MotionState::INIT;
            return SerialComTlt::State::IDLE;
    }
    motion_state_ = next_motion_state_;
    return SerialComTlt::State::MOTION;
}

/*Failure state.*/
SerialComTlt::State SerialComTlt::failureState(){
    return SerialComTlt::State::FAILURE;
}

/*
Main communication loop:
- send RC heartbeat to keep communication alive.
- at every loop get current encoder ticks and status.
- use state machine to determine actions to take.
*/
void SerialComTlt::comLoop(){
    vector<unsigned char> params = {0x01, 0x00, 0xff};
    while(run_){
        while(com_started_){
            // Start/Maintain Communication
            sendCmd("RC",params);
            // Update Data
            getPoseM1();
            getPoseM2();
            // getPercentSpeedM1();
            // getPercentSpeedM2();
            getStatusM1();
            getStatusM2();
            // State Machine
            switch(state_)
            {
                case SerialComTlt::State::INIT:
                    next_state_ = initState();
                    break;
                case SerialComTlt::State::CALIB:
                    next_state_ = calibState();
                    break;
                case SerialComTlt::State::IDLE:
                    next_state_ = idleState();
                    break;
                case SerialComTlt::State::MOTION:
                    next_state_ = motionState();
                    break;
                case SerialComTlt::State::FAILURE:
                    next_state_ = failureState();
                    break;
            }
            if(state_ != next_state_){
                cout << "State Transition: " << STATE_NAMES[int(state_)] << " -> " << STATE_NAMES[int(next_state_)] << endl;
                if(next_state_ == State::CALIB){
                    cout << "Pre-calib Status: ";
                    for(auto it = mot1_status_.begin(); it != mot1_status_.end(); it++){
                        cout << *it;
                    }
                    cout << "|";
                    for(auto it = mot2_status_.begin(); it != mot2_status_.end(); it++){
                        cout << *it;
                    }
                    cout << endl;
                }
                if(state_ == State::CALIB && next_state_ == State::IDLE){
                    cout << "Pre-calib Status: ";
                    for(auto it = mot1_status_.begin(); it != mot1_status_.end(); it++){
                        cout << *it;
                    }
                    cout << "|";
                    for(auto it = mot2_status_.begin(); it != mot2_status_.end(); it++){
                        cout << *it;
                    }
                    cout << endl;
                }
            }
            state_ = next_state_;
        }
        usleep(1);
    }
}

