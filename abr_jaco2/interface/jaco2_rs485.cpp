#include "jaco2_rs485.h"

const unsigned char Jaco2::CONTROL_MODE = 0x01;
const unsigned char Jaco2::HAND_ADDRESS[3] = {0x16, 0x17, 0x18};
const float Jaco2::MAX_TORQUE[6] = {40.0, 80.0, 40.0, 20.0, 20.0, 20.0};  // in Nm
const unsigned char Jaco2::JOINT_ADDRESS[6] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15};
const unsigned char Jaco2::TORQUE_DAMPING[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const short Jaco2::TORQUE_KP[6] = {1000, 1500, 1000, 1750, 1750, 1750};

Jaco2::Jaco2(int a_display_error_level) {

    // get current date and time for error logging
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%d-%m-%Y %I:%M:%S",timeinfo);
    string currentdatetime(buffer);
    datetime = currentdatetime;

    // check the current log file size, if greater than 5Mb, delete it
    file_limit = 5e6;
    log_save_location = ".jaco2_log.txt";

    ifstream file(log_save_location.c_str(), ios::binary | ios::ate);
    if (file.tellg() > file_limit)
    {
       remove(log_save_location.c_str());
       log_msg(1, "File size exceeded, log file deleted");
    }

    ctr = 0;
    //set common variables
    delay = 1250;
    packets_sent = 6;
    packets_read = 18; //3 responses (14, 15, 16) expected per motor
    current_motor = 6; // only 6 joints so if source address is not < 6 after
                      // reading should receive error due do array size
    display_error_level = a_display_error_level;
    types.push_back("DEBUG");
    types.push_back("INFO");
    types.push_back("WARNING");
    types.push_back("ERROR");

    // get the current date and time
    // TODO: get date and time with ctime for first line of log and name of
    // file

    memset(updated, 0, (size_t)sizeof(int)*6);
    memset(updated_hand, 0, (size_t)sizeof(int)*3);
    memset(pos_finger, 0.0, (size_t)sizeof(float)*3);

    write_count = 0;
    read_count = 0;
    unsigned short d = 0x00;

    // error messages from arm
    error_message.push_back("NO");
    error_message.push_back("TEMPERATURE");
    error_message.push_back("TEMPERATURE");
    error_message.push_back("VELOCITY");
    error_message.push_back("POSITION LIMITATION");
    error_message.push_back("ABSOLUTE POSITION");
    error_message.push_back("RELATIVE POSITION");
    error_message.push_back("COMMAND");
    error_message.push_back("CURRENT");
    error_message.push_back("TORQUE");

    //We load the API.
    commLayer_Handle = dlopen(
        "./Kinova.API.CommLayerUbuntu.so",
        RTLD_NOW|RTLD_GLOBAL);

    if (!commLayer_Handle) {
        fprintf(stderr, "dlopen failed: %s\n", dlerror());
        exit(EXIT_FAILURE);
    }

    //Initialization of the function pointers.
    fptrInitCommunication = (int (*)()) dlsym(commLayer_Handle,
                                              "InitCommunication");
    fptrCloseCommunication = (int (*)()) dlsym(commLayer_Handle,
                                              "CloseCommunication");
    MyRS485_Activate = (int (*)()) dlsym(commLayer_Handle,"RS485_Activate");
    MyRS485_Read = (int (*)(RS485_Message* PackagesIn, int QuantityWanted,
                            int &ReceivedQtyIn)) dlsym(commLayer_Handle,
                            "RS485_Read");
    MyRS485_Write = (int (*)(RS485_Message* PackagesOut, int QuantityWanted,
                               int &ReceivedQtyIn)) dlsym(commLayer_Handle,
                               "RS485_Write");

    // TODO: make a message initialization function

    // Set up static parts of messages sent across
    // Set up the message used by SendTargetAngles
    for (int ii = 0; ii<6; ii++) {
        target_angles_message[ii].Command = POSITION_COMMAND;
        target_angles_message[ii].SourceAddress = SOURCE_ADDRESS;
        target_angles_message[ii].DestinationAddress = JOINT_ADDRESS[ii];
        // target_angles_message[ii].DataLong[2] = 0x1;
        target_angles_message[ii].DataLong[2] = 0x00000000;
        target_angles_message[ii].DataLong[3] = 0x00000000;
    }

    for (int ii = 0; ii<3; ii++) {
        target_angles_hand_message[ii].Command = POSITION_COMMAND;
        target_angles_hand_message[ii].SourceAddress = SOURCE_ADDRESS;
        target_angles_hand_message[ii].DestinationAddress = HAND_ADDRESS[ii];
        // target_angles_message[ii].DataLong[2] = 0x1;
        target_angles_hand_message[ii].DataLong[2] = 0x00000000;
        target_angles_hand_message[ii].DataLong[3] = 0x00000000;
    }


    // Set up the message used by SendTargetAngles
    for (int ii = 0; ii<6; ii++) {
        clear_error_message[ii].Command = CLEAR_ERROR_FLAG;
        clear_error_message[ii].SourceAddress = SOURCE_ADDRESS;
        clear_error_message[ii].DestinationAddress = JOINT_ADDRESS[ii];
        clear_error_message[ii].DataFloat[0] = 0x00;
        clear_error_message[ii].DataLong[1] = 0; // 0 to clear all errors
        clear_error_message[ii].DataLong[2] = 0x00000000;
        clear_error_message[ii].DataLong[3] = 0x00000000;
    }

    // set constants in force message to increase loop speed
    for (int ii=0; ii<6; ii++) {
        force_message[ii].Command =
            RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
        force_message[ii].DestinationAddress = JOINT_ADDRESS[ii];
        force_message[ii].SourceAddress = SOURCE_ADDRESS;
        force_message[ii].DataLong[1] = 0x00000000; //not used
        force_message[ii].DataLong[3] = //U16|U8|U8
            ((unsigned long) TORQUE_KP[ii] << 16) |
            ((unsigned long) CONTROL_MODE << 8) |
            ((unsigned long) TORQUE_DAMPING[ii]);
    }

    // Set up get position message
    for (int ii=0; ii<6; ii++) {
        get_position_message[ii].Command = 0x0001;
        get_position_message[ii].SourceAddress = SOURCE_ADDRESS;
        get_position_message[ii].DestinationAddress = JOINT_ADDRESS[ii];
        get_position_message[ii].DataFloat[0] = 0x00000000;
        get_position_message[ii].DataLong[1] = 0x00000000;
        get_position_message[ii].DataFloat[2] = 0x00000000;
        get_position_message[ii].DataLong[3] = 0x00000000;
     }

    // Set up get position message
    for (int ii=0; ii<3; ii++) {
        get_position_hand_message[ii].Command = 0x0001;
        get_position_hand_message[ii].SourceAddress = SOURCE_ADDRESS;
        get_position_hand_message[ii].DestinationAddress = HAND_ADDRESS[ii];
        get_position_hand_message[ii].DataFloat[0] = 0x00000000;
        get_position_hand_message[ii].DataLong[1] = 0x00000000;
        get_position_hand_message[ii].DataFloat[2] = 0x00000000;
        get_position_hand_message[ii].DataLong[3] = 0x00000000;
     }

    // Set up robot initialization message
    for (int ii = 0; ii<6; ii++) {
        //Initialize the INIT message
        init_message[ii].Command = RS485_MSG_GET_ACTUALPOSITION;
        init_message[ii].SourceAddress = SOURCE_ADDRESS;//0 means the API
        init_message[ii].DestinationAddress = JOINT_ADDRESS[ii];

        //Those value are not used for this command.
        init_message[ii].DataLong[0] = 0x00000000;
        init_message[ii].DataLong[1] = 0x00000000;
        init_message[ii].DataLong[2] = 0x00000000;
        init_message[ii].DataLong[3] = 0x00000000;
    }

    // Set up initialize position mode message
    for (int ii=0; ii<6; ii++) {
        init_position_message[ii].Command = SWITCH_CONTROL_MODE_REQUEST;
        init_position_message[ii].SourceAddress = SOURCE_ADDRESS;
        init_position_message[ii].DestinationAddress = JOINT_ADDRESS[ii];
        init_position_message[ii].DataLong[0] = ((unsigned short)0x00 |
          ((unsigned short) d << 8) | ((unsigned short) d << 16 |
          ((unsigned short) d << 24))); //U24|U8
        init_position_message[ii].DataLong[1] = 0x00000000;
        init_position_message[ii].DataLong[2] = 0x00000000;
        init_position_message[ii].DataLong[3] = 0x00000000;
    }

    // Set up the initialize torque mode message
    for(int ii=0; ii<6; ii++) {
        init_torque_message[ii].Command = SWITCH_CONTROL_MODE_REQUEST;
        init_torque_message[ii].SourceAddress = SOURCE_ADDRESS;
        init_torque_message[ii].DestinationAddress = JOINT_ADDRESS[ii];//DESTINATION_ADDRESS;
        init_torque_message[ii].DataLong[0] = ((unsigned short) 0x01 |
            ((unsigned short) d << 8) | ((unsigned short) d << 16 |
            ((unsigned short) d << 24))); //U24|U8
        init_torque_message[ii].DataLong[1]=0x00000000;
        init_torque_message[ii].DataLong[3]=0x00000000;
    }

    // Set torque safety parameters
    for (int ii=0; ii<6; ii++) {
        safety_message[ii].Command = SEND_TORQUE_CONFIG_SAFETY;
        safety_message[ii].SourceAddress = SOURCE_ADDRESS;
        safety_message[ii].DestinationAddress = JOINT_ADDRESS[ii];
        safety_message[ii].DataFloat[0] = MAX_TORQUE[ii]; // Nm maximum torque
        // Data1: fraction of max speed before error sent, 1=off
        safety_message[ii].DataFloat[1] = 1.0; //0.75 safety factor
        safety_message[ii].DataFloat[2] = 0.0; //not used
        safety_message[ii].DataFloat[3] = 0.0; //not used
    }

    // Set up the test torque message
    for (int ii=0; ii<6; ii++) {
        test_torques_message[ii].Command =
            RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
        test_torques_message[ii].SourceAddress = SOURCE_ADDRESS;
        test_torques_message[ii].DestinationAddress = JOINT_ADDRESS[ii];
        test_torques_message[ii].DataLong[1] = 0x00000000;  // not used
        test_torques_message[ii].DataFloat[2] = 0; //32F torque command [Nm]
        test_torques_message[ii].DataLong[3] = //U16|U8|U8
            ((unsigned long) TORQUE_KP[ii] << 16) |
            ((unsigned long) CONTROL_MODE << 8) |
            ((unsigned long) TORQUE_DAMPING[ii]);
    }

    // Set up the torque config feedforward advanced message
    for (int ii=0; ii<6; ii++) {
        torques_config_feedforward_advanced_message[ii].Command =
            SEND_TORQUE_CONFIG_FEEDFORWARD_ADVANCED;
        torques_config_feedforward_advanced_message[ii].SourceAddress = SOURCE_ADDRESS;
        torques_config_feedforward_advanced_message[ii].DestinationAddress = JOINT_ADDRESS[ii];
        torques_config_feedforward_advanced_message[ii].DataFloat[0] = 0.8;  // feed_velocity_under_gain;
        torques_config_feedforward_advanced_message[ii].DataFloat[1] = 125.0;  // feed_current_voltage_conversion;
        torques_config_feedforward_advanced_message[ii].DataFloat[2] = 2.0;  // static friction;
        torques_config_feedforward_advanced_message[ii].DataFloat[3] = 2.0;  // max static friction;
    }
    //torques_config_feedforward_advanced_message[0].DataFloat[2] = 1.9;

    // Set up the torque config filters
    for (int ii=0; ii<6; ii++) {
        torque_config_filters_message[ii].Command =
            SEND_TORQUE_CONFIG_FILTERS;
        torque_config_filters_message[ii].SourceAddress = SOURCE_ADDRESS;
        torque_config_filters_message[ii].DestinationAddress = JOINT_ADDRESS[ii];
        torque_config_filters_message[ii].DataFloat[0] = 100.0;  // velocity_filter;
        torque_config_filters_message[ii].DataFloat[1] = 400.0;  //torque_measured_filter;
        torque_config_filters_message[ii].DataFloat[2] = 2.5;  // torque_error_filter;
        torque_config_filters_message[ii].DataFloat[3] = 50.0;  // control_effort_filter;
    }
    //joint1 is different from the rest, for now to avoid array...
    torque_config_filters_message[1].DataFloat[3] = 20.0;

    // Set up the torque config parameters 1
    for (int ii=0; ii<6; ii++) {
        torque_config_parameters_message1[ii].Command = 0x214;
        torque_config_parameters_message1[ii].SourceAddress = SOURCE_ADDRESS;
        torque_config_parameters_message1[ii].DestinationAddress = JOINT_ADDRESS[ii];
        torque_config_parameters_message1[ii].DataFloat[0] = 50.0;  // velocity safety limit filter
        torque_config_parameters_message1[ii].DataFloat[1] = 1.0;  // feedforward filter
        torque_config_parameters_message1[ii].DataFloat[2] = 501.0;  // inactivity time message
        torque_config_parameters_message1[ii].DataFloat[3] = 200.0;  // error resend time
    }

    // Set up the torque config parameters 2
    for (int ii=0; ii<6; ii++) {
        torque_config_parameters_message2[ii].Command =
            SEND_TORQUE_CONFIG_CONTROL_PARAM_2;
        torque_config_parameters_message2[ii].SourceAddress = SOURCE_ADDRESS;
        torque_config_parameters_message2[ii].DestinationAddress = JOINT_ADDRESS[ii];
        torque_config_parameters_message2[ii].DataFloat[0] = 4.0;  // switch_threshold;
        torque_config_parameters_message2[ii].DataFloat[1] = 5.0;  // pos_lim_distance;
        torque_config_parameters_message2[ii].DataFloat[2] = 1.0;  // error_deadband;
        torque_config_parameters_message2[ii].DataFloat[3] = 0.0;  // torque_brake;
    }
}

Jaco2::~Jaco2() { }

void Jaco2::Connect() {
    log_msg(1, "Initializing RS-485 Communication...");
    //Flag used during initialization.
    int result;

    //If all functions are loaded correctly.
    if(fptrInitCommunication != NULL || MyRS485_Activate != NULL ||
       MyRS485_Read != NULL || MyRS485_Write != NULL) {
        //Initialization of the API
        result = fptrInitCommunication();

        //If API's initialization is correct.
        if(result == NO_ERROR_KINOVA) {
            log_msg(2, "RS485 initialization completed");

            /*We activate the RS-485 comm API. From here you cannot control the
              robot with the Joystick or with the normal USB function. Only
              RS-485 command will be accepted. Reboot the robot to get back to
              normal control.*/

            MyRS485_Activate();

            // If we did not receive the answer, continue reading until done
            log_msg(2, "Activating Jaco2...");
            SendAndReceive(init_message, true);
        }
        else {
            log_msg(4, string("Error " + result) + " while Initializing");
        }
    }
    else {
        log_msg(4, "Could not load API's functions");
    }
    log_msg(2, "Connection successful");
}

void Jaco2::Disconnect() {
    fptrCloseCommunication();
    log_msg(2, "Connection closed");

    ofstream myfile;
    myfile.open (log_save_location.c_str(), fstream::app);
    myfile << "------------------------------------------------------\n";
    myfile.close();
}

void Jaco2::InitPositionMode() {
    log_msg(1, "Initializing position control mode...");
    SendAndReceive(init_position_message, true);
    log_msg(2, "Position control mode activated");
}

void Jaco2::InitForceMode() {
    // STEP 1: Get current position
    log_msg(1, "Initializing force mode");
    log_msg(1, "STEP 1/7: Getting current position...");
    SendAndReceive(get_position_message, true);

    // STEP 2-4: set control parameters
    // Let's also try setting the static friction parameter
    log_msg(1,"STEP 2/7: Setting torque config feedforward advanced parameters...");
    // no need for a response, because I have no idea what's supposed to be
    // returned, this is lacking a lot of documentation
    SendAndReceive(torques_config_feedforward_advanced_message, false);

    // Set advanced torque parameters 1
    log_msg(1, "STEP 3/7: Setting advanced torque parameters 1...");
    // no need for a response, because I have no idea what's supposed to be
    // returned, this is lacking a lot of documentation
    SendAndReceive(torque_config_parameters_message1, false);

    // Set advanced torque parameters 2
    log_msg(1, "STEP 4/7: Setting advanced torque parameters 2...");
    // no need for a response, because I have no idea what's supposed to be
    // returned, this is lacking a lot of documentation
    SendAndReceive(torque_config_parameters_message2, false);

    // STEP 5: Set torque safety parameters
    log_msg(1, "STEP 5/7: Setting torque safety parameters...");
    SendAndReceive(safety_message, true);

    int joints_updated0 = 0;
    while (joints_updated0 < 6) {
        // STEP 6: Send torque values to compare with sensor readings
        log_msg(1, "STEP 6/7: Checking torque sensor calibration...");
        int joints_updated1 = 0;
        while(joints_updated1 < 6) {
            // NOTE: motor 1 is flipped so input torques need to be * - 1
            torque_load[1] *= -1;
            for (int ii=0; ii<6; ii++) {
                test_torques_message[ii].DataFloat[0] = pos[ii];
                test_torques_message[ii].DataLong[1] = torque_load[ii];
            }
            joints_updated1 = SendAndReceive(test_torques_message, false);
        }

        // STEP 7: Send request to switch to torque control mode
        log_msg(1, "STEP 7/7: Sending request to switch to torque control mode...");
        joints_updated0 = SendAndReceive(init_torque_message, false);
    }

    log_msg(2, "Force control mode activated");
}

void Jaco2::SendTargetAnglesSetup() {
    SendAndReceive(get_position_message, true);
    for (int ii = 0; ii<6; ii++) {
        target_angle[ii] = pos[ii];
        target_angles_message[ii].DataFloat[0] = target_angle[ii];
        target_angles_message[ii].DataFloat[1] = target_angle[ii];
    }
}

int Jaco2::SendTargetAngles(float q_target[6]) {
    int TargetReached = 0;
    float mod_pos;
    float q_diff;

    // increment joint command by 1 degree until target reached
    TargetReached = 0;
    for (int ii = 0; ii<6; ii++) {
        mod_pos = (int(pos[ii]) % 360 + 360) % 360 ;
        pos_rad[ii] = mod_pos * 3.14159 / 180.0;
        q_diff = q_target[ii] - mod_pos;
        // compare target to current angle to see if should add or subtract
        if (abs(mod_pos - q_target[ii]) < 2.0 ) {
            TargetReached += 1;
            target_angle[ii] += 0.0;
        }
        else if(q_diff < (q_diff/abs(q_diff) * 180)) {
            target_angle[ii] += 0.05;
        }
        else if (q_diff >= (q_diff/abs(q_diff) * 180)) {
            target_angle[ii] -= 0.05;
        }
        if (ctr % 1000 == 0) {
            char buffer [100];
            sprintf(buffer, "Actuator: %d position is: %f with target: %f mod 360: %f",
                    ii, pos[ii], q_target[ii], mod_pos);
            log_msg(1, buffer);
        }
        // assign the new target (increment added)
        target_angles_message[ii].DataFloat[0] = target_angle[ii];
        target_angles_message[ii].DataFloat[1] = target_angle[ii];
    }
    ctr += 1;
    SendAndReceive(target_angles_message, true);

    return(TargetReached);
}

void Jaco2::SendTargetAnglesHand(bool open) {

    // STEP 1: move to rest position
    // increment joint command by 1 degree until target reached
    for (int ii = 0; ii<3; ii++) {
        if (open == true) {
            pos_finger[ii] -= 100;
        }
        else {
            pos_finger[ii] += 100;
        }
        //We assign the new command (increment added)
        target_angles_hand_message[ii].DataFloat[0] = pos_finger[ii];
        target_angles_hand_message[ii].DataFloat[1] = pos_finger[ii];
    }
    SendAndReceiveHand(target_angles_hand_message, true);
}

// Wraps the set of input torques u up into a message and sends it to the Jaco2
void Jaco2::SendForces(float u[6]) {
    // load the torque signal into outbound message
    for (int ii=0; ii<6; ii++) {
        force_message[ii].DataFloat[0] = pos[ii];
        force_message[ii].DataFloat[2] = u[ii]; //32F torque command [1Nm]
    }

    MyRS485_Write(force_message, packets_sent, write_count);
    usleep(1250); // TODO: EXPERIMENT WITH DIFFERENT DELAY
    MyRS485_Read(feedback_message, packets_read, read_count);

    // The response for a SEND_POSITION_AND_TORQUE (0x0014) command is 3
    // messages per motor [0x0015, 0x0016, 0x0017]. Read through all
    // feedback, and if data for a motor hasn't been received yet update
    // then update the position and velocity variables for that motor. Use
    // reduced processing here rather than through ProcessFeedback for speed.
    // NOTE: torque_load is not updated while reading this feedback
    memset(updated, 0, (size_t)sizeof(int)*6);
    for(int ii = 0; ii < read_count; ii++) {
        if(feedback_message[ii].Command == RS485_MSG_SEND_ALL_VALUES_1) {
            //actuator 0 is 16
            current_motor = feedback_message[ii].SourceAddress - 16;
            if (updated[current_motor] == 0) {
                pos[current_motor] = feedback_message[ii].DataFloat[1];
                vel[current_motor] = feedback_message[ii].DataFloat[2];
                torque_load[current_motor] = feedback_message[ii].DataFloat[3];
                updated[current_motor] = 1;
            }
        }
        else if(feedback_message[ii].Command == REPORT_ERROR) {
            //cout << "PRINTING ERROR" << endl;
            PrintError(ii, current_motor);
            //cout << "SEND CLEAR ERROR MESSAGE" << endl;
            SendAndReceive(clear_error_message, false);
            //updated[current_motor] = 0;
        }
    }
}

// Sends a message out to the arm, reads and processes the feedback,
// and has the option to loop repeatedly until message returns successfully
// for each of the motors. Returns the number of motors message
// was successfully sent to and received from.
int Jaco2::SendAndReceive(RS485_Message message[6], bool loop) {
    int joints_updated = 0;
    while(joints_updated < 6) {
        joints_updated = 0;
        MyRS485_Write(message, packets_sent, write_count);
        usleep(delay);
        MyRS485_Read(feedback_message, packets_read, read_count);
        ProcessFeedback();

        for (int ii = 0; ii < 6; ii++) {
            joints_updated += updated[ii];
            if (updated[ii] == 0) {
                char buffer [100];
                sprintf(buffer, "Data for joint %d not updated, checking again...", ii);
                log_msg(1, buffer);
            }
        }

        if (loop == false) {
            break;
        }
    }
    return joints_updated;
}

// TODO: can this function use updated instead of updated_hand?
int Jaco2::SendAndReceiveHand(RS485_Message message[3], bool loop) {
    int hand_updated = 0;
    while(hand_updated < 3) {
        hand_updated = 0;
        MyRS485_Write(message, packets_sent, write_count);
        usleep(delay);
        MyRS485_Read(feedback_message, packets_read, read_count);

        // reset variables for this time through
        memset(updated_hand, 0, (size_t)sizeof(int)*3);
        // cycle through all of the received messages and
        // assign the received values to the corresponding joint
        for(int ii = 0; ii < read_count; ii++) {
            // actuator 0 is 16
            if (feedback_message[ii].Command == 0x02 ||
                    feedback_message[ii].Command == 0x11){

                current_motor = feedback_message[ii].SourceAddress - 22;
                updated_hand[current_motor] = 1;
            }
        }

        for (int ii = 0; ii < 3; ii++) {
            hand_updated += updated_hand[ii];
            if (updated_hand[ii] == 0) {
                char buffer [100];
                sprintf(buffer, "Data for finger %d not updated, checking again...", ii);
                log_msg(1, buffer);
            }
        }

        if (loop == false) {
            break;
        }
    }
    return hand_updated;
}

// Reads the class variable feedback_message, checks for errors and then
// processes appropriately based of Command type. Updates class variables
// when appropriate and sets the elements of updated = 1 if data for the
// corresponding motor was read successfully, and = 0 if there was a
// problem reading data for that motor.
void Jaco2::ProcessFeedback() {

    // reset variables for this time through
    memset(updated, 0, (size_t)sizeof(int)*6);
    // buffer for error logging
    char buffer [100];
    // cycle through all of the received messages and
    // assign the received values to the corresponding joint
    for(int ii = 0; ii < read_count; ii++) {
        // actuator 0 is 16
        current_motor = feedback_message[ii].SourceAddress - 16;
        if (current_motor > 5) {
            log_msg(1, "Joint feedback received for unknown joint");
            continue;
        }

        switch (feedback_message[ii].Command) {

            case RS485_MSG_SEND_ALL_VALUES_1:

                pos[current_motor] = feedback_message[ii].DataFloat[1];
                vel[current_motor] = feedback_message[ii].DataFloat[2];
                torque_load[current_motor] = feedback_message[ii].DataFloat[3];
                updated[current_motor] = 1;

                break;

            case POSITION_AND_CURRENT:

            case SEND_ACTUAL_POSITION:

                pos[current_motor] = feedback_message[ii].DataFloat[1];
                torque_load[current_motor] = feedback_message[ii].DataFloat[3];
                updated[current_motor] = 1;

                break;

            case REPORT_ERROR:
                PrintError(ii, current_motor);
                SendAndReceive(clear_error_message, true);
                updated[current_motor] = 0;
                break;

            case SEND_TORQUE_VALIDATION:

                // safety measure checks
                switch (feedback_message[ii].DataLong[0]) {

                    case 0:
                        sprintf(buffer, "Torque validation False for servo %d, Response: %u",
                                current_motor, feedback_message[ii].DataLong[0]);
                        log_msg(1, buffer);
                        break;

                    case 1:
                        sprintf(buffer, "Torque validation True for servo %d, Response: %u",
                                current_motor, feedback_message[ii].DataLong[0]);
                        log_msg(1, buffer);
                        updated[current_motor] = 1;
                        break;

                    default:
                        sprintf(buffer, "Torque validation not received for servo %d, Response: %u",
                                current_motor, feedback_message[ii].DataLong[0]);
                        log_msg(1, buffer);
                }

                break;

            case SWITCH_CONTROL_MODE_REPLY:

                switch (feedback_message[ii].DataLong[0]) {

                    case 0:
                        sprintf(buffer, "Switch control mode False for servo %d, Response: %u",
                                current_motor, feedback_message[ii].DataLong[0]);
                        log_msg(1, buffer);
                        break;

                    case 1:
                        sprintf(buffer, "Switch control mode to Position control True for servo %d, Response: %u",
                                current_motor, feedback_message[ii].DataLong[0]);
                        log_msg(1, buffer);
                        updated[current_motor] = 1;
                        break;

                    case 257:
                        sprintf(buffer, "Switch control mode to Force control True for servo %d, Response: %u",
                                current_motor, feedback_message[ii].DataLong[0]);
                        log_msg(1, buffer);
                        // TODO: should we not have updated and update2 to
                        // specify which modes we switched to? look into this
                        updated[current_motor] = 1;
                        break;

                    default:
                        sprintf(buffer, "No response to control mode switch request for servo %d, Response: %u",
                                current_motor, feedback_message[ii].DataLong[0]);
                        log_msg(1, buffer);
                }
                break;

            case GET_TORQUE_CONFIG_SAFETY:

                updated[current_motor] = 1;
                sprintf(buffer, "Torque config safety test passed for servo %d", current_motor);
                log_msg(1, buffer);
                break;
        }
    }
}

// Process the error message, if it's an actual error then print out
// information (it is possible that an error message is not an actual error)
void Jaco2::PrintError(int index, int current_motor) {
    char buffer [100];
    sprintf(buffer, "Message: %u %s for motor %d", feedback_message[index].DataLong[1],
            error_message[feedback_message[index].DataLong[1]].c_str(), current_motor);
    log_msg(4, buffer);
}

int Jaco2::log_msg(int type, string msg)
{
    // check if message level based off enum, is higher than warning level, if
    // so print out message
    if (type >= display_error_level){
        cout << types[type-1].c_str() << ": " << msg << endl;
    }
    // append message log to save later
    ofstream myfile;
    myfile.open (log_save_location.c_str(), fstream::app);
    myfile << datetime << " " << types[type-1].c_str() << ": " << msg << "\n";
    myfile.close();
    return 0;
}
