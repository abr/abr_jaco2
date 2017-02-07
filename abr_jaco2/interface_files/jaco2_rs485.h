//INCLUDE SECTION
#include <iostream>
#include <dlfcn.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include "kinova-api/Kinova.API.CommLayerUbuntu.h"
#include "kinova-api/Kinova.API.UsbCommandLayerUbuntu.h"
#include "kinova-api/KinovaTypes.h"
#include <string.h>
#include <vector>
#include <math.h>

using namespace std;

#define SOURCE_ADDRESS 0x00

#define SEND_ACTUAL_POSITION 0x002
#define POSITION_COMMAND 0x0010
#define POSITION_AND_CURRENT 0x0011
#define RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND 0x0014
#define GET_TORQUE_VALIDATION_REQUEST 0x0201
#define SEND_TORQUE_VALIDATION 0x0202
#define SWITCH_CONTROL_MODE_REQUEST 0x0203
#define SWITCH_CONTROL_MODE_REPLY 0x0204
#define SEND_TORQUE_CONFIG_SAFETY 0x0208
#define SEND_TORQUE_CONFIG_FEEDFORWARD_ADVANCED 0x0213
#define POSITION_LIMIT 0x0021
#define REPORT_ERROR 0x0030
#define GET_TORQUE_CONFIG_SAFETY 0x003F
#define CLEAR_ERROR_FLAG 0x0033

class Jaco2 {
    public:
        // misc variables
        int delay;
        int messageReceived;
        int messageReceived2;
        int ActuatorInitialized;
        int updated[6];
        int updated2[6]; // for switching to position mode since updated is used to switch to torque mode
        int currentMotor;
        vector<string> errorMessage;

        // main functions
        void ApplyQ(float q_target[6]);
        void ApplyU(float u[6]);
        void Connect();
        void Disconnect();
        void PrintError(int index, int currentMotor);
        void ProcessFeedback();
        void InitForceMode();
        void InitPositionMode();
        int SendAndReceive(RS485_Message message[6], bool loop);

        // read variables
        float pos[6]; //From Halls sensor
        float vel[6];
        float torque_load[6];
        bool read_input;
        int packets_sent;
        int packets_read;

        // torque variables
        unsigned char torqueDamping[6];
        unsigned char controlMode;
        unsigned short torqueKp[6];
        float staticFriction;
        float maxStaticFriction;
        float feed_current_voltage_conversion;
        float feed_velocity_under_gain;

        float maxT[6];

        // variables used during the communication process.
        int WriteCount;
        int ReadCount;
        unsigned char joint[6];

        // RS485 arrays of structs
        RS485_Message ApplyQMessage[6];
        RS485_Message ForceMessage[6];
        RS485_Message GetPositionMessage[6];
        RS485_Message InitMessage[6];
        RS485_Message InitPositionMessage[6];
        RS485_Message InitTorqueMessage[6];
        RS485_Message MessageListIn [50];
        RS485_Message ReceivedInitMessage[18];
        RS485_Message SafetyMessage[6];
        RS485_Message TestTorquesMessage[6];
        RS485_Message TorquesConfigFeedforwardAdvanced[6];
        RS485_Message ValidateTorquesMessage[6];

        // A handle needed to open the API(library).
        void *commLayer_Handle;

        // function pointers
        int (*fptrInitCommunication)();
        int (*fptrCloseCommunication)();
        int (*MyRS485_Activate)();
        int (*MyRS485_Read)(RS485_Message*, int, int&);
        int (*MyRS485_Write)(RS485_Message*, int, int&);

        Jaco2(); //constructor
        ~Jaco2(); // deconstructor
};
