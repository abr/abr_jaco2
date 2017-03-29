#include "jaco2_rs485.h"

Jaco2::Jaco2() {

    ctr = 0;

    //set common variables
    delay = 1250;
    packets_sent = 6;
    packets_read = 18; //3 responses (14, 15, 16) expected per motor
    current_motor = 6; // only 6 joints so if source address is not < 6 after
                      // reading should receive error due do array size

    memset(updated, 0, (size_t)sizeof(int)*6);
    memset(updated_hand, 0, (size_t)sizeof(int)*3);

    write_count = 0;
    read_count = 0;

    //joint addresses from base to wrist
    joint_address[0] = 0x10;
    joint_address[1] = 0x11;
    joint_address[2] = 0x12;
    joint_address[3] = 0x13;
    joint_address[4] = 0x14;
    joint_address[5] = 0x15;

    hand_address[0] = 0x16;
    hand_address[1] = 0x17;
    hand_address[2] = 0x18;

    pos_finger[0] = 0.0;
    pos_finger[1] = 0.0;
    pos_finger[2] = 0.0;

    //set torque parameters
    //max torque in Nm
    max_torque[0] = 40.0;
    max_torque[1] = 80.0;
    max_torque[2] = 40.0;
    max_torque[3] = 20.0;
    max_torque[4] = 20.0;
    max_torque[5] = 20.0;

    control_mode = 0x01;

    torque_damping[0] = 0x00;
    torque_damping[1] = 0x00;
    torque_damping[2] = 0x00;
    torque_damping[3] = 0x00;
    torque_damping[4] = 0x00;
    torque_damping[5] = 0x00;

    torque_kp[0] = 1000;
    torque_kp[1] = 1500;
    torque_kp[2] = 1000;
    torque_kp[3] = 1750;
    torque_kp[4] = 1750;
    torque_kp[5] = 1750;

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

    unsigned short d1 = 0x00;
    unsigned short d2 = 0x00;
    unsigned short d3 = 0x00;

    // Set up static parts of messages sent across
    // Set up the message used by SendTargetAngles
    for (int ii = 0; ii<6; ii++) {
        target_angles_message[ii].Command = POSITION_COMMAND;
        target_angles_message[ii].SourceAddress = SOURCE_ADDRESS;
        target_angles_message[ii].DestinationAddress = joint_address[ii];
        // target_angles_message[ii].DataLong[2] = 0x1;
        target_angles_message[ii].DataLong[2] = 0x00000000;
        target_angles_message[ii].DataLong[3] = 0x00000000;
    }

    for (int ii = 0; ii<3; ii++) {
        target_angles_hand_message[ii].Command = POSITION_COMMAND;
        target_angles_hand_message[ii].SourceAddress = SOURCE_ADDRESS;
        target_angles_hand_message[ii].DestinationAddress = hand_address[ii];
        // target_angles_message[ii].DataLong[2] = 0x1;
        target_angles_hand_message[ii].DataLong[2] = 0x00000000;
        target_angles_hand_message[ii].DataLong[3] = 0x00000000;
    }


    // Set up the message used by SendTargetAngles
    for (int ii = 0; ii<6; ii++) {
        clear_error_message[ii].Command = CLEAR_ERROR_FLAG;
        clear_error_message[ii].SourceAddress = SOURCE_ADDRESS;
        clear_error_message[ii].DestinationAddress = joint_address[ii];
        clear_error_message[ii].DataLong[0] = ((unsigned long) 0);
        clear_error_message[ii].DataLong[2] = 0x00000000;
        clear_error_message[ii].DataLong[3] = 0x00000000;
    }

    // set constants in force message to increase loop speed
    for (int ii=0; ii<6; ii++) {
        force_message[ii].Command =
            RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
        force_message[ii].DestinationAddress = joint_address[ii];
        force_message[ii].SourceAddress = SOURCE_ADDRESS;
        force_message[ii].DataLong[1] = 0x00000000; //not used
        force_message[ii].DataLong[3] = //U16|U8|U8
            ((unsigned long) torque_kp[ii] << 16) |
            ((unsigned long) control_mode << 8) |
            ((unsigned long) torque_damping[ii]);
    }

    // Set up get position message
    for (int ii=0; ii<6; ii++) {
        get_position_message[ii].Command = 0x0001;
        get_position_message[ii].SourceAddress = SOURCE_ADDRESS;
        get_position_message[ii].DestinationAddress = joint_address[ii];
        get_position_message[ii].DataFloat[0] = 0x00000000;
        get_position_message[ii].DataLong[1] = 0x00000000;
        get_position_message[ii].DataFloat[2] = 0x00000000;
        get_position_message[ii].DataLong[3] = 0x00000000;
     }

    // Set up get position message
    for (int ii=0; ii<3; ii++) {
        get_position_hand_message[ii].Command = 0x0001;
        get_position_hand_message[ii].SourceAddress = SOURCE_ADDRESS;
        get_position_hand_message[ii].DestinationAddress = hand_address[ii];
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
        init_message[ii].DestinationAddress = joint_address[ii];

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
        init_position_message[ii].DestinationAddress = joint_address[ii];
        init_position_message[ii].DataLong[0] = ((unsigned short)0x00 |
          ((unsigned short)d2 << 8) | ((unsigned short)d3 << 16 |
          ((unsigned short)d1 << 24))); //U24|U8
        init_position_message[ii].DataLong[1] = 0x00000000;
        init_position_message[ii].DataLong[2] = 0x00000000;
        init_position_message[ii].DataLong[3] = 0x00000000;
    }

    // Set up the initialize torque mode message
    for(int ii=0; ii<6; ii++) {
        init_torque_message[ii].Command = SWITCH_CONTROL_MODE_REQUEST;
        init_torque_message[ii].SourceAddress = SOURCE_ADDRESS;
        init_torque_message[ii].DestinationAddress = joint_address[ii];//DESTINATION_ADDRESS;
        init_torque_message[ii].DataLong[0] = ((unsigned short) 0x01 |
            ((unsigned short) d2 << 8) | ((unsigned short) d3 << 16 |
            ((unsigned short) d1 << 24))); //U24|U8
        init_torque_message[ii].DataLong[1]=0x00000000;
        init_torque_message[ii].DataLong[3]=0x00000000;
    }

    // Set torque safety parameters
    for (int ii=0; ii<6; ii++) {
        safety_message[ii].Command = SEND_TORQUE_CONFIG_SAFETY;
        safety_message[ii].SourceAddress = SOURCE_ADDRESS;
        safety_message[ii].DestinationAddress = joint_address[ii];
        safety_message[ii].DataFloat[0] = max_torque[ii]; // Nm maximum torque
        safety_message[ii].DataFloat[1] = 1.0; //0.75 safety factor
        safety_message[ii].DataFloat[2] = 0.0; //not used
        safety_message[ii].DataFloat[3] = 0.0; //not used
    }

    // Set up the test torque message
    for (int ii=0; ii<6; ii++) {
        test_torques_message[ii].Command =
            RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
        test_torques_message[ii].SourceAddress = SOURCE_ADDRESS;
        test_torques_message[ii].DestinationAddress = joint_address[ii];
        test_torques_message[ii].DataLong[1] = 0x00000000;  // not used
        test_torques_message[ii].DataFloat[2] = 0; //32F torque command [Nm]
        test_torques_message[ii].DataLong[3] = //U16|U8|U8
            ((unsigned long) torque_kp[ii] << 16) |
            ((unsigned long) control_mode << 8) |
            ((unsigned long) torque_damping[ii]);
    }

    // Set up the torque config feedforward advanced message
    for (int ii=0; ii<6; ii++) {
        torques_config_feedforward_advanced_message[ii].Command =
            SEND_TORQUE_CONFIG_FEEDFORWARD_ADVANCED;
        torques_config_feedforward_advanced_message[ii].SourceAddress = SOURCE_ADDRESS;
        torques_config_feedforward_advanced_message[ii].DestinationAddress = joint_address[ii];
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
        torque_config_filters_message[ii].DestinationAddress = joint_address[ii];
        torque_config_filters_message[ii].DataFloat[0] = 100.0;  // velocity_filter;
        torque_config_filters_message[ii].DataFloat[1] = 400.0;  //torque_measured_filter;
        torque_config_filters_message[ii].DataFloat[2] = 2.5;  // torque_error_filter;
        torque_config_filters_message[ii].DataFloat[3] = 50.0;  // control_effort_filter;
    }
    //joint1 is different from the rest, for now to avoid array...
    torque_config_filters_message[1].DataFloat[3] = 20.0;


    // Set up the torque config parameters 2
    for (int ii=0; ii<6; ii++) {
        torque_config_parameters_message[ii].Command =
            SEND_TORQUE_CONFIG_CONTROL_PARAM_2;
        torque_config_parameters_message[ii].SourceAddress = SOURCE_ADDRESS;
        torque_config_parameters_message[ii].DestinationAddress = joint_address[ii];
        torque_config_parameters_message[ii].DataFloat[0] = 4.0;  // switch_threshold;
        torque_config_parameters_message[ii].DataFloat[1] = 5.0;  // pos_lim_distance;
        torque_config_parameters_message[ii].DataFloat[2] = 1.0;  // error_deadband;
        torque_config_parameters_message[ii].DataFloat[3] = 0.0;  // torque_brake;
    }
}

Jaco2::~Jaco2() { }

void Jaco2::Connect() {
    cout << "RS-485 communication Initialization" << endl;
    //Flag used during initialization.
    int result;

    //If all functions are loaded correctly.
    if(fptrInitCommunication != NULL || MyRS485_Activate != NULL ||
       MyRS485_Read != NULL || MyRS485_Write != NULL) {
        //Initialization of the API
        result = fptrInitCommunication();

        //If API's initialization is correct.
        if(result == NO_ERROR_KINOVA) {
            cout << "USB initialization completed" << endl << endl;

            /*We activate the RS-485 comm API. From here you cannot control the
              robot with the Joystick or with the normal USB function. Only
              RS-485 command will be accepted. Reboot the robot to get back to
              normal control.*/

            MyRS485_Activate();

            // If we did not receive the answer, continue reading until done
            cout << "Initializing Jaco2...";
            SendAndReceive(init_message, true);
        }
        else {
            cout << "Error " << result << " while Initializing" << endl;
        }
    }
    else {
        cout << "Errors while loading API's function" << endl;
    }
    cout << "Connection successful";
}

void Jaco2::Disconnect() {
    fptrCloseCommunication();
    cout << "Connection closed" << endl;
}

void Jaco2::InitPositionMode() {
    cout << "Initialize position control mode..." << endl;
    SendAndReceive(init_position_message, true);
    cout << "Position control mode activated" << endl;
}

void Jaco2::InitForceMode() {
    // STEP 0: Get current position
    cout << "STEP 0/4: Get current position" << endl;
    SendAndReceive(get_position_message, true);

    // Let's also try setting the static friction parameter
    cout << "STEP 1a/4: Set torque config feedforward advanced parameters" << endl;
    // no need for a response, because I have no idea what's supposed to be
    // returned, this is lacking a lot of documentation
    SendAndReceive(torques_config_feedforward_advanced_message, false);

    // Set advanced torque parameters 2
    cout << "STEP 1b/4: Set advanced torque parameters 2" << endl;
    // no need for a response, because I have no idea what's supposed to be
    // returned, this is lacking a lot of documentation
    SendAndReceive(torque_config_parameters_message, false);

    // Set torque config filters
    //cout << "STEP 1c/4: Set advanced torque parameters 2" << endl;
    // no need for a response, because I have no idea what's supposed to be
    // returned, this is lacking a lot of documentation
    //SendAndReceive(torque_config_filters_message, false);

    // STEP 1: Set torque safety parameters
    cout << "STEP 2/4: Set torque safety parameters" << endl;
    SendAndReceive(safety_message, true);

    int joints_updated0 = 0;
    while (joints_updated0 < 6) {
        // STEP 2: Send torque values to compare with sensor readings
        cout << "STEP 3/4: Send torque values to compare with sensor readings"
             << endl;
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

        // STEP 3: Send request to switch to torque control mode
        cout << "STEP 4/4: Send request to switch to torque control mode"
             << endl;
        joints_updated0 = SendAndReceive(init_torque_message, false);
    }

    cout << "SUCCESS: Switching to Torque Mode" << endl;
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
            cout << "Actuator: " << ii << " position is: " << pos[ii]
                 << " with target: " << q_target[ii]
                 << " mod 360: " << mod_pos << endl;
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
    usleep(100); // TODO: EXPERIMENT WITH DIFFERENT DELAY
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
            PrintError(ii, current_motor);
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
                cout << "Warning: Data for joint " << ii << " not updated."
                    << endl;
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
                cout << "Warning: Data for finger " << ii << " not updated."
                    << endl;
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
    // cycle through all of the received messages and
    // assign the received values to the corresponding joint
    for(int ii = 0; ii < read_count; ii++) {
        // actuator 0 is 16
        current_motor = feedback_message[ii].SourceAddress - 16;
        if (current_motor > 5) {
            cout << "JOINT FEEDBACK RECEIVED FOR UNKNOWN JOINT "
                << current_motor << endl;
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
                // in case of an error, go on to the next packet

                for (int jj = 0; jj < 6; jj++) {
                    clear_error_message[jj].DataLong[1] =
                        ((unsigned long) feedback_message[ii].DataLong[0]);
                }
                SendAndReceive(clear_error_message, false);
                updated[current_motor] = 0;
                break;

            // case ACK_MESSAGE:
            //     // a clear error acknowledgement was received
            //     break;

            case SEND_TORQUE_VALIDATION:

                // safety measure checks
                switch (feedback_message[ii].DataLong[0]) {

                    case 0:
                        cout << "Torque Validation False for Servo "
                            << current_motor << " , Response: "
                            << feedback_message[ii].DataLong[0] << endl;
                        break;

                    case 1:
                        cout << "Torque Validation True for Servo "
                            << current_motor << " , Response: "
                            << feedback_message[ii].DataLong[0] << endl;
                        updated[current_motor] = 1;
                        break;

                    default:
                        cout << "ERROR READING TORQUE VALIDATION REPLY FOR SERVO: "
                            << current_motor << " , RESPONSE: "
                            << feedback_message[ii].DataLong[0] << endl;
                }

                break;

            case SWITCH_CONTROL_MODE_REPLY:

                switch (feedback_message[ii].DataLong[0]) {

                    case 0:
                        cout << "Switch Control Mode False for Servo "
                            << current_motor << " , Response: "
                            << feedback_message[ii].DataLong[0] << endl;
                        break;

                    case 1:
                        cout << "Switch Control Mode POSITION True for Servo "
                                << current_motor << " , Response: "
                                << feedback_message[ii].DataLong[0] << endl;
                        updated[current_motor] = 1;
                        break;

                    case 257:
                        cout << "Switch Control Mode TORQUE True for Servo "
                                << current_motor << " , Response: "
                                << feedback_message[ii].DataLong[0] << endl;
                        updated[current_motor] = 1;
                        break;

                    default:

                        cout << "ERROR READING SWITCH CONTROL MODE REPLY FOR SERVO "
                            << current_motor << " , RESPONSE: "
                            << feedback_message[ii].DataLong[0] << endl;
                }
                break;

            case GET_TORQUE_CONFIG_SAFETY:

                updated[current_motor] = 1;
                cout << "Safety passed for servo " << current_motor << endl;

                break;

            //default:

                //cout << "Unknown command: " << feedback_message[ii].Command
                //     << endl;
        }
    }
}

// Process the error message, if it's an actual error then print out
// information (it is possible that an error message is not an actual error)
void Jaco2::PrintError(int index, int current_motor) {
    if (feedback_message[index].DataLong[1] != 0) {
        cout << "\nERROR\n";
        cout << feedback_message[index].DataLong[1] << " ";
        cout << error_message[feedback_message[index].DataLong[1]] << " ";
        cout << "for motor " << current_motor << endl;
    }
}
