/*
AUTHORS: Pawel Jaworski, Travis DeWolf, Martine Blouin
*/

#include "jaco2_rs485.h"

Jaco2::Jaco2() {

    //set common variables
    delay = 1250;
    packets_sent = 6;
    packets_read = 18; //3 responses (14, 15, 16) expected per motor
    currentMotor = 6; // only 6 joints so if source address is not < 6 after
                      // reading should receive error due do array size

    memset(updated, 0, (size_t)sizeof(int)*6);

    WriteCount = 0;
    ReadCount = 0;

    //joint addresses from base to wrist
    joint[0] = 0x10;
    joint[1] = 0x11;
    joint[2] = 0x12;
    joint[3] = 0x13;
    joint[4] = 0x14;
    joint[5] = 0x15;

    //set torque parameters
    //max torque in Nm
    memset(maxT, 100.0, (size_t)sizeof(float)*6);

    controlMode = 0x01;

    torqueDamping[0] = 0x00;
    torqueDamping[1] = 0x00;
    torqueDamping[2] = 0x00;
    torqueDamping[3] = 0x00;
    torqueDamping[4] = 0x00;
    torqueDamping[5] = 0x00;

    torqueKp[0] = 1000;
    torqueKp[1] = 1500;
    torqueKp[2] = 1000;
    torqueKp[3] = 1750;
    torqueKp[4] = 1750;
    torqueKp[5] = 1750;

    staticFriction = 0.0;
    maxStaticFriction = 2.0;
    feed_current_voltage_conversion = 125.0;
    feed_velocity_under_gain = 0.8;

    // error messages from arm
    errorMessage.push_back("NO");
    errorMessage.push_back("TEMPERATURE");
    errorMessage.push_back("TEMPERATURE");
    errorMessage.push_back("VELOCITY");
    errorMessage.push_back("POSITION LIMITATION");
    errorMessage.push_back("ABSOLUTE POSITION");
    errorMessage.push_back("RELATIVE POSITION");
    errorMessage.push_back("COMMAND");
    errorMessage.push_back("CURRENT");
    errorMessage.push_back("TORQUE");

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
    // Set up the message used by ApplyQ
    for (int ii = 0; ii<6; ii++) {
        ApplyQMessage[ii].Command =
            POSITION_COMMAND;
        ApplyQMessage[ii].SourceAddress = SOURCE_ADDRESS;
        ApplyQMessage[ii].DestinationAddress = joint[ii];
        // ApplyQMessage[ii].DataLong[2] = 0x1;
        ApplyQMessage[ii].DataLong[2] = 0x00000000;
        ApplyQMessage[ii].DataLong[3] = 0x00000000;
    }

    // set constants in force message to increase loop speed
    for (int ii=0; ii<6; ii++) {
        ForceMessage[ii].Command =
            RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
        ForceMessage[ii].DestinationAddress = joint[ii];
        ForceMessage[ii].SourceAddress = SOURCE_ADDRESS;
        ForceMessage[ii].DataLong[1] = 0x00000000; //not used
        ForceMessage[ii].DataLong[3] = //U16|U8|U8
            ((unsigned long) torqueKp[ii] << 16) |
            ((unsigned long) controlMode << 8) |
            ((unsigned long) torqueDamping[ii]);
    }

    // Set up get position message
    for (int ii=0; ii<6; ii++) {
        GetPositionMessage[ii].Command = 0x0001;
        GetPositionMessage[ii].SourceAddress = SOURCE_ADDRESS;
        GetPositionMessage[ii].DestinationAddress = joint[ii];
        GetPositionMessage[ii].DataFloat[0] = 0x00000000;
        GetPositionMessage[ii].DataLong[1] = 0x00000000;
        GetPositionMessage[ii].DataFloat[2] = 0x00000000;
        GetPositionMessage[ii].DataLong[3] = 0x00000000;
     }

    // Set up robot initialization message
    for (int ii = 0; ii<6; ii++) {
        //Initialize the INIT message
        InitMessage[ii].Command = RS485_MSG_GET_ACTUALPOSITION;
        InitMessage[ii].SourceAddress = SOURCE_ADDRESS;//0 means the API
        InitMessage[ii].DestinationAddress = joint[ii];

        //Those value are not used for this command.
        InitMessage[ii].DataLong[0] = 0x00000000;
        InitMessage[ii].DataLong[1] = 0x00000000;
        InitMessage[ii].DataLong[2] = 0x00000000;
        InitMessage[ii].DataLong[3] = 0x00000000;
    }

    // Set up initialize position mode message
    for (int ii=0; ii<6; ii++) {
        InitPositionMessage[ii].Command = SWITCH_CONTROL_MODE_REQUEST;
        InitPositionMessage[ii].SourceAddress = SOURCE_ADDRESS;
        InitPositionMessage[ii].DestinationAddress = joint[ii];
        InitPositionMessage[ii].DataLong[0] = ((unsigned short)0x00 |
          ((unsigned short)d2 << 8) | ((unsigned short)d3 << 16 |
          ((unsigned short)d1 << 24))); //U24|U8
        InitPositionMessage[ii].DataLong[1] = 0x00000000;
        InitPositionMessage[ii].DataLong[2] = 0x00000000;
        InitPositionMessage[ii].DataLong[3] = 0x00000000;
    }

    // Set up the initialize torque mode message
    for(int ii=0; ii<6; ii++) {
        InitTorqueMessage[ii].Command = SWITCH_CONTROL_MODE_REQUEST;
        InitTorqueMessage[ii].SourceAddress = SOURCE_ADDRESS;
        InitTorqueMessage[ii].DestinationAddress = joint[ii];//DESTINATION_ADDRESS;
        InitTorqueMessage[ii].DataLong[0] = ((unsigned short) 0x01 |
            ((unsigned short) d2 << 8) | ((unsigned short) d3 << 16 |
            ((unsigned short) d1 << 24))); //U24|U8
        InitTorqueMessage[ii].DataLong[1]=0x00000000;
        InitTorqueMessage[ii].DataLong[3]=0x00000000;
    }

    // Set torque safety parameters
    for (int ii=0; ii<6; ii++) {
        SafetyMessage[ii].Command = SEND_TORQUE_CONFIG_SAFETY;
        SafetyMessage[ii].SourceAddress = SOURCE_ADDRESS;
        SafetyMessage[ii].DestinationAddress = joint[ii];
        SafetyMessage[ii].DataFloat[0] = maxT[ii]; // Nm maximum torque
        SafetyMessage[ii].DataFloat[1] = 1.0; //0.75 safety factor
        SafetyMessage[ii].DataFloat[2] = 0.0; //not used
        SafetyMessage[ii].DataFloat[3] = 0.0; //not used
    }

    // Set up the test torque message
    for (int ii=0; ii<6; ii++) {
        TestTorquesMessage[ii].Command =
            RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
        TestTorquesMessage[ii].SourceAddress = SOURCE_ADDRESS;
        TestTorquesMessage[ii].DestinationAddress = joint[ii];
        TestTorquesMessage[ii].DataLong[1] = 0x00000000;  // not used
        TestTorquesMessage[ii].DataFloat[2] = 0; //32F torque command [Nm]
        TestTorquesMessage[ii].DataLong[3] = //U16|U8|U8
            ((unsigned long) torqueKp[ii] << 16) |
            ((unsigned long) controlMode << 8) |
            ((unsigned long) torqueDamping[ii]);
    }

    // Set up the torque config feedforward advanced message
    for (int ii=0; ii<6; ii++) {
        TorquesConfigFeedforwardAdvanced[ii].Command =
            SEND_TORQUE_CONFIG_FEEDFORWARD_ADVANCED;
        TorquesConfigFeedforwardAdvanced[ii].SourceAddress = SOURCE_ADDRESS;
        TorquesConfigFeedforwardAdvanced[ii].DestinationAddress = joint[ii];
        TorquesConfigFeedforwardAdvanced[ii].DataFloat[0] = feed_velocity_under_gain;
        TorquesConfigFeedforwardAdvanced[ii].DataFloat[1] = feed_current_voltage_conversion;
        TorquesConfigFeedforwardAdvanced[ii].DataFloat[2] = staticFriction;
        TorquesConfigFeedforwardAdvanced[ii].DataFloat[3] = maxStaticFriction;
    }

    // Set up the validate torque message
    for (int ii=0; ii<6; ii++) {
        ValidateTorquesMessage[ii].Command = GET_TORQUE_VALIDATION_REQUEST;
        ValidateTorquesMessage[ii].SourceAddress = SOURCE_ADDRESS;
        ValidateTorquesMessage[ii].DestinationAddress = joint[ii];  //DESTINATION_ADDRESS;
        ValidateTorquesMessage[ii].DataLong[1] = 0x00000000; //not used
        ValidateTorquesMessage[ii].DataFloat[2] = 0x00000000; //not used
        ValidateTorquesMessage[ii].DataLong[3] = 0x00000000; //not used
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
            SendAndReceive(InitMessage, true);
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
    SendAndReceive(InitPositionMessage, true);
    cout << "Position control mode activated" << endl;
}

void Jaco2::InitForceMode() {
    // STEP 0: Get current position
    cout << "STEP 0/4: Get current position" << endl;
    SendAndReceive(GetPositionMessage, true);

    // Let's also try setting the static friction parameter
    cout << "STEP 1/4: Set torque config feedforward advanced parameters" << endl;
    // no need for a response, because I have no idea what's supposed to be
    // returned, this is lacking a lot of documentation
    SendAndReceive(TorquesConfigFeedforwardAdvanced, false);

    // STEP 1: Set torque safety parameters
    cout << "STEP 2/4: Set torque safety parameters" << endl;
    SendAndReceive(SafetyMessage, true);

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
                TestTorquesMessage[ii].DataFloat[0] = pos[ii];
                TestTorquesMessage[ii].DataLong[1] = torque_load[ii];
            }
            joints_updated1 = SendAndReceive(TestTorquesMessage, false);
        }

        // STEP 3: Send request to switch to torque control mode
        cout << "STEP 4/4: Send request to switch to torque control mode"
             << endl;
        joints_updated0 = SendAndReceive(InitTorqueMessage, false);
    }

    cout << "SUCCESS: Switching to Torque Mode" << endl;
}

void Jaco2::ApplyQ(float q_target[6]) {
    // STEP 0: Get initial position
    cout << "STEP 0: Get current position" << endl;
    SendAndReceive(GetPositionMessage, true);

    // STEP 1: move to rest position
    int TargetReached = 0;
    int ctr = 0;
    float Joint6Command[6];
    for (int ii = 0; ii<6; ii++) {
        Joint6Command[ii] = pos[ii];
        ApplyQMessage[ii].DataFloat[0] = Joint6Command[ii];
        ApplyQMessage[ii].DataFloat[1] = Joint6Command[ii];
    }

    while(TargetReached < 6) {
        TargetReached = 0;
        // increment joint command by 1 degree until target reached
        for (int ii = 0; ii<6; ii++) {
            // compare target to current angle to see if should add or subtract
            if(q_target[ii] > (fmod(int(pos[ii]),360))) {
                Joint6Command[ii] += 0.05;
            }
            else if(q_target[ii] < (fmod(int(pos[ii]),360))) {
                Joint6Command[ii] -= 0.05;
            }

            //We assign the new command (increment added)
            ApplyQMessage[ii].DataFloat[0] = Joint6Command[ii];
            ApplyQMessage[ii].DataFloat[1] = Joint6Command[ii];
        }

        SendAndReceive(ApplyQMessage, true);

        for (int jj = 0; jj < 6; jj++) {
            if (abs(fmod(int(pos[jj]),360) - q_target[jj]) < 2.0 ) {
                TargetReached += 1;
            }
            else if (ctr == 1000) {
                cout << "Actuator: " << jj << " position is: " << pos[jj]
                     << " with target: " << q_target[jj]
                     << " mod 360: " << fmod(int(pos[jj]),360) << endl;
                ctr = 0;
            }
        }
        ctr += 1;
    }
}
//
//
// void Jaco2::ApplyQ(float q_target[6]) {
//     // STEP 0: Get initial position
//     cout << "STEP 0: Get current position" << endl;
//     SendAndReceive(GetPositionMessage, true);
//
//     // STEP 1: calculate which direction each joint is going
//     int offsets[6];
//
//     // STEP 2: incrementally move each motor to target
//     int ctr = 0;
//     int TargetReached = 0;
//     while(TargetReached < 6) {
//         TargetReached = 0;
//
//         memset(offsets, 0.05, (size_t)sizeof(float)*6);
//         for (int ii = 0; ii < 6; ii++) {
//             if(q_target[ii] < (fmod(int(pos[ii]),360))) {
//                 offsets[ii] *= -1;
//             }
//         }
//
//         // increment joint command by 1 degree until target reached
//         for (int ii = 0; ii < 6; ii++) {
//             // We assign the new command (increment added)
//             ApplyQMessage[ii].DataFloat[0] = pos[ii] + offsets[ii];
//             ApplyQMessage[ii].DataFloat[1] = pos[ii] + offsets[ii];
//         }
//         SendAndReceive(ApplyQMessage, true);
//
//         for (int ii = 0; ii < 6; ii++) {
//             // if target has been reached
//             if (abs(fmod(int(pos[ii]),360) - q_target[ii]) < 2.0) {
//                 TargetReached += 1; // update the counter
//                 offsets[ii] = 0.0; // set the offset for this motor = 0
//             }
//             else if (ctr == 1000) {
//                 cout << "Motor " << ii << " position is: " << pos[ii]
//                      << " with target: " << q_target[ii]
//                      << " mod 360: " << fmod(int(pos[ii]),360) << endl;
//                 ctr = 0;
//             }
//         }
//         ctr += 1;
//     }
// }

// Wraps the set of input torques u up into a message and sends it to the Jaco2
void Jaco2::ApplyU(float u[6]) {
    // load the torque signal into outbound message
    for (int ii=0; ii<6; ii++) {
        ForceMessage[ii].DataFloat[0] = pos[ii];
        ForceMessage[ii].DataFloat[2] = u[ii]; //32F torque command [1Nm]
    }

    MyRS485_Write(ForceMessage, packets_sent, WriteCount);
    usleep(1250); // TO DO: EXPERIMENT WITH DIFFERENT DELAY
    MyRS485_Read(MessageListIn, packets_read, ReadCount);

    // The response for a SEND_POSITION_AND_TORQUE (0x0014) command is 3
    // messages per motor [0x0015, 0x0016, 0x0017]. Read through all
    // feedback, and if data for a motor hasn't been received yet update
    // then update the position and velocity variables for that motor. Use
    // reduced processing here rather than through ProcessFeedback for speed.
    // NOTE: torque_load is not updated while reading this feedback
    memset(updated, 0, (size_t)sizeof(int)*6);
    for(int ii = 0; ii < ReadCount; ii++) {
        if(MessageListIn[ii].Command == RS485_MSG_SEND_ALL_VALUES_1) {
            //actuator 0 is 16
            currentMotor = MessageListIn[ii].SourceAddress - 16;
            if (updated[currentMotor] == 0) {
                pos[currentMotor] = MessageListIn[ii].DataFloat[1];
                vel[currentMotor] = MessageListIn[ii].DataFloat[2];
                updated[currentMotor] = 1;
            }
        }
        else if(MessageListIn[ii].Command == REPORT_ERROR) {
            PrintError(ii, currentMotor);
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
        MyRS485_Write(message, packets_sent, WriteCount);
        usleep(delay);
        MyRS485_Read(MessageListIn, packets_read, ReadCount);
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

// Reads the class variable MessageListIn, checks for errors and then
// processes appropriately based of Command type. Updates class variables
// when appropriate and sets the elements of updated = 1 if data for the
// corresponding motor was read successfully, and = 0 if there was a
// problem reading data for that motor.
void Jaco2::ProcessFeedback() {

    // reset variables for this time through
    memset(updated, 0, (size_t)sizeof(int)*6);
    // cycle through all of the received messages and
    // assign the received values to the corresponding joint
    for(int ii = 0; ii < ReadCount; ii++) {
        // actuator 0 is 16
        currentMotor = MessageListIn[ii].SourceAddress - 16;
        if (currentMotor > 5) {
            cout << "JOINT FEEDBACK RECEIVED FOR UNKNOWN JOINT "
                 << currentMotor << endl;
            continue;
        }

        switch (MessageListIn[ii].Command) {

            case REPORT_ERROR :
                PrintError(ii, currentMotor);
                // in case of an error, go on to the next packet
                break;

            case POSITION_AND_CURRENT :

            case SEND_ACTUAL_POSITION :

                pos[currentMotor] = MessageListIn[ii].DataFloat[1];
                torque_load[currentMotor] = MessageListIn[ii].DataFloat[3];
                updated[currentMotor] = 1;

                break;

            case RS485_MSG_SEND_ALL_VALUES_1:

                pos[currentMotor] = MessageListIn[ii].DataFloat[1];
                vel[currentMotor] = MessageListIn[ii].DataFloat[2];
                torque_load[currentMotor] = MessageListIn[ii].DataFloat[3];
                updated[currentMotor] = 1;

                break;

            case SEND_TORQUE_VALIDATION :

                // safety measure checks
                if (MessageListIn[ii].DataLong[0] == 1) {
                    cout << "Torque Validation True for Servo "
                        << currentMotor << " , Response: "
                        << MessageListIn[ii].DataLong[0] << endl;
                    updated[currentMotor] = 1;
                }
                else if (MessageListIn[ii].DataLong[0] == 0) {
                    cout << "Torque Validation False for Servo "
                        << currentMotor << " , Response: "
                        << MessageListIn[ii].DataLong[0] << endl;
                }
                else {
                    cout << "ERROR READING TORQUE VALIDATION REPLY FOR SERVO: "
                        << currentMotor << " , RESPONSE: "
                        << MessageListIn[ii].DataLong[0] << endl;
                }

                break;

            case SWITCH_CONTROL_MODE_REPLY :

                if (MessageListIn[ii].DataLong[0] == 257) {
                    cout << "Switch Control Mode TORQUE True for Servo "
                            << currentMotor << " , Response: "
                            << MessageListIn[ii].DataLong[0] << endl;
                    updated[currentMotor] = 1;
                }
                else if (MessageListIn[ii].DataLong[0] == 1) {
                    cout << "Switch Control Mode POSITION True for Servo "
                            << currentMotor << " , Response: "
                            << MessageListIn[ii].DataLong[0] << endl;
                    updated[currentMotor] = 1;
                }
                else if (MessageListIn[ii].DataLong[0] == 0) {
                    cout << "Switch Control Mode False for Servo "
                        << currentMotor << " , Response: "
                        << MessageListIn[ii].DataLong[0] << endl;
                }
                else {
                    cout << "ERROR READING SWITCH CONTROL MODE REPLY FOR SERVO "
                        << currentMotor << " , RESPONSE: "
                        << MessageListIn[ii].DataLong[0] << endl;
                }
                break;

            case GET_TORQUE_CONFIG_SAFETY :

                updated[currentMotor] = 1;
                cout << "Safety passed for servo " << currentMotor << endl;

                break;

            //default :

                //cout << "Unknown command : " << MessageListIn[ii].Command
                //     << endl;
        }
    }
}

// Process the error message, if it's an actual error then print out
// information (it is possible that an error message is not an actual error)
void Jaco2::PrintError(int index, int currentMotor) {
    if (MessageListIn[index].DataLong[1] != 0) {
        cout << "\nERROR\n";
        cout << MessageListIn[index].DataLong[1] << " ";
        cout << errorMessage[MessageListIn[index].DataLong[1]] << " ";
        cout << "for motor " << currentMotor << endl;
    }
}
