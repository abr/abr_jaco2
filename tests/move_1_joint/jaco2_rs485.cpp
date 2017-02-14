// TO COMPILE RUN g++ jaco2_rs485.cpp -Wl,--no-as-needed -ldl
#include "jaco2_rs485.h"
#include <math.h>

Jaco2::Jaco2(void) {
    //set common variables
    flag = 0;
    torqueValidation = false;
    switchValidation = false;
    read_input = true;
    delay = 2000;
    qtyWanted = 1;
    packets_sent = 6;
    packets_read = 18;

    torqueDamping = 0x00;
    controlMode = 0x01;
    torqueKp = 1750; // torque kp 1.75 * 1000

    WriteCount = 0;
    ReadCount = 0;
    //joint addresses from base to wrist
    joint[0] = 0x10;
    joint[1] = 0x11;
    joint[2] = 0x12;
    joint[3] = 0x13;
    joint[4] = 0x14;
    joint[5] = 0x15;

    //We load the API.
	commLayer_Handle = dlopen("./kinova-api/Kinova.API.CommLayerUbuntu.so",
	                          RTLD_NOW|RTLD_GLOBAL);

	//Initialization of the fucntion pointers.
	fptrInitCommunication = (int (*)()) dlsym(commLayer_Handle,
	                                          "InitCommunication");
	MyRS485_Activate = (int (*)()) dlsym(commLayer_Handle,"RS485_Activate");
	MyRS485_Read = (int (*)(RS485_Message* PackagesIn, int QuantityWanted,
	                        int &ReceivedQtyIn)) dlsym(commLayer_Handle,
	                        "RS485_Read");
	MyRS485_Write = (int (*)(RS485_Message* PackagesOut, int QuantityWanted,
                             int &ReceivedQtyIn)) dlsym(commLayer_Handle,
                             "RS485_Write");

    float switch_threshold = 2.0;
    float pos_lim_distance = 5.0;
    float error_deadband = 1.0;
    float torque_brake = 0.0;
    // Set up the torque config parameters 2
    for (int ii=0; ii<6; ii++) {
        TorqueConfigParameters2[ii].Command =
            SEND_TORQUE_CONFIG_CONTROL_PARAM_2;
        TorqueConfigParameters2[ii].SourceAddress = SOURCE_ADDRESS;
        TorqueConfigParameters2[ii].DestinationAddress = joint[ii];
        TorqueConfigParameters2[ii].DataFloat[0] = switch_threshold;
        TorqueConfigParameters2[ii].DataFloat[1] = pos_lim_distance;
        TorqueConfigParameters2[ii].DataFloat[2] = error_deadband;
        TorqueConfigParameters2[ii].DataFloat[3] = torque_brake;
    }
}

int main()
{   
    cout << "T H R E E" << endl;
    usleep(1000000);
    cout << "T W O" << endl;
    usleep(1000000);
    cout << "O N E" << endl;
    usleep(1000000);
    
    Jaco2 j2 = Jaco2();

    float u[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
    
    // ---------- select joint and params ----------
    j2.joint[0] = 0x15;
    //j2.torqueKp = 1000;
    //j2.torqueKp = 1500;
    j2.torqueKp = 1750;
    u[0] = 0.74;
    // ---------------------------------------------

    j2.Connect(j2.joint[0]); 
    
    j2.InitForceMode(j2.joint[0]);
    
    for (int i = 0; i<1000; i++)
        {
            //u[0] = 4.0 * sin(2.0*3.14159 * i/1000);
            //cout << u[0] << endl;
            j2.ApplyU(j2.joint[0], u);
        }

    j2.Disconnect(j2.joint[0]);
    //j2.Disconnect(j2.joint[5]);
}

void Jaco2::Connect(unsigned char DESTINATION_ADDRESS)
{
	cout << "RS-485 communication Initialization" << endl;
	//Flag used during initialization.
	bool ActuatorInitialized1 = false;
	bool ActuatorInitialized2 = false;
    int result;

	//If all functions are loaded correctly.
	if(fptrInitCommunication != NULL || MyRS485_Activate != NULL ||
	   MyRS485_Read != NULL || MyRS485_Write != NULL)
	{
		//Initialization of the API
		int result = fptrInitCommunication();

		//If API's initialization is correct.
		if(result == NO_ERROR_KINOVA)
		{
			cout << "U S B   I N I T I A L I Z A T I O N   C O M P L E T E D"
			     << endl << endl;

			/*We activate the RS-485 comm API. From here you cannot control the
			  robot with the Joystick or with the normal USB function. Only
			  RS-485 command will be accepted. Reboot the robot to get back to
			  normal control.*/

			MyRS485_Activate();

			//Initialize the INIT message
			InitMessage[0].Command = RS485_MSG_GET_ACTUALPOSITION; //Set command ID
			InitMessage[0].SourceAddress = SOURCE_ADDRESS;        //0 means the API
			InitMessage[0].DestinationAddress = DESTINATION_ADDRESS;//DESTINATION_ADDRESS;//destinaiton

			//Those value are not used for this command.
			InitMessage[0].DataLong[0] = 0x00000000;
			InitMessage[0].DataLong[1] = 0x00000000;
			InitMessage[0].DataLong[2] = 0x00000000;
			InitMessage[0].DataLong[3] = 0x00000000;
			
			//Initialize the INIT message
			/*InitMessage[1].Command = RS485_MSG_GET_ACTUALPOSITION; //Set command ID
			InitMessage[1].SourceAddress = SOURCE_ADDRESS;        //0 means the API
			InitMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;//destinaiton

			//Those value are not used for this command.
			InitMessage[1].DataLong[0] = 0x00000000;
			InitMessage[1].DataLong[1] = 0x00000000;
			InitMessage[1].DataLong[2] = 0x00000000;
			InitMessage[1].DataLong[3] = 0x00000000;*/


			//If we did not receive the answer, continue reading until done
			while(ReadCount != 1 && !ActuatorInitialized1)
			{
				MyRS485_Write(InitMessage, 1, WriteCount);
				usleep(2000);
				//GetFeedback(&ReceiveInitMessage);
				MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
		        
		        cout << "W R I T E  C O U N T  " << WriteCount << endl;
                cout << "R E A D  C O U N T  " << ReadCount << endl;

				/*We make sure that the mesage come from actuator 6(0x15) and
				that the command ID is RS485_MSG_SEND_ACTUALPOSITION
				which is the answer of our message. (See document Kinova RS485
				Communication protocol).*/
				if(ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS &&
				   ReceiveInitMessage[0].Command == RS485_MSG_SEND_ACTUALPOSITION)
				   //&& ReceiveInitMessage[1].SourceAddress == 0x15 &&
				   //ReceiveInitMessage[1].Command == RS485_MSG_SEND_ACTUALPOSITION)
				{
					ActuatorInitialized1 = true;
					cout << "actuator 1 initialized" << endl;
					cout << "base " << ReceiveInitMessage[0].SourceAddress << endl;
					//cout << "wrist" << ReceiveInitMessage[1].SourceAddress << endl;
				}
				else
				{
				    cout << "Error while initializing actuator 1" << endl;
				}
				
				/*if(ReceiveInitMessage[1].SourceAddress == 0x15 &&
				   ReceiveInitMessage[1].Command == RS485_MSG_SEND_ACTUALPOSITION)
				{
					ActuatorInitialized2 = true;
					cout << "actuator 2 initialized" << endl;
					cout << "wrist " << ReceiveInitMessage[0].SourceAddress << endl;
					//cout << "wrist" << ReceiveInitMessage[1].SourceAddress << endl;
				}
				else
				{
				    cout << "Error while initializing actuator 2" << endl;
				}*/
			}
		}
		else
		{
		    cout << "Errors while Initializing" << endl;
		}
    }
	else
	{
		cout << "Errors while loading API's function" << endl;
	}
}

void Jaco2::InitForceMode(unsigned char DESTINATION_ADDRESS)
{    
    // ========== BEGIN MAIN COMM ==========
    SendAndReceive(TorqueConfigParameters2, false);
    // STEP 0: Get initial position
    TrajectoryMessage[0].Command = 0x0001;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = DESTINATION_ADDRESS;//DESTINATION_ADDRESS;
    TrajectoryMessage[0].DataFloat[0] = 0x00000000;
    TrajectoryMessage[0].DataLong[1] = 0x00000000;
    TrajectoryMessage[0].DataFloat[2] = 0x00000000;
    TrajectoryMessage[0].DataLong[3] = 0x00000000;
    
    /*TrajectoryMessage[1].Command = 0x0001;
    TrajectoryMessage[1].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;
    TrajectoryMessage[1].DataFloat[0] = 0x00000000;
    TrajectoryMessage[1].DataLong[1] = 0x00000000;
    TrajectoryMessage[1].DataFloat[2] = 0x00000000;
    TrajectoryMessage[1].DataLong[3] = 0x00000000;*/
    //pthread_mutex_lock (&APIMutex);
    MyRS485_Write(TrajectoryMessage, 1, WriteCount);
    //pthread_mutex_unlock (&APIMutex);
    usleep(delay);



    MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
    if (ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS &&
	    ReceiveInitMessage[0].Command == 0x0002)
    {
	    pos[0] = ReceiveInitMessage[0].DataFloat[1];
	    cout << "Current position base is: "
		    << ReceiveInitMessage[0].DataFloat[1] << endl;
		    
    }

    bool ack = false;

    SafetyMessage[0].Command =
	    0x0208;
    SafetyMessage[0].SourceAddress = SOURCE_ADDRESS;
    SafetyMessage[0].DestinationAddress = DESTINATION_ADDRESS;
    SafetyMessage[0].DataFloat[0] = 100.0; //10 Nm maximum torque
    SafetyMessage[0].DataFloat[1] = 1.0; //0.75 safety factor
    SafetyMessage[0].DataFloat[2] = 0.0; //not used
    SafetyMessage[0].DataFloat[3] = 0.0; //not used
    
    /*SafetyMessage[1].Command =
	    0x0208;
    SafetyMessage[1].SourceAddress = SOURCE_ADDRESS;
    SafetyMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;
    SafetyMessage[1].DataFloat[0] = 100.0; //10 Nm maximum torque
    SafetyMessage[1].DataFloat[1] = 1.0; //0.75 safety factor
    SafetyMessage[1].DataFloat[2] = 0.0; //not used
    SafetyMessage[1].DataFloat[3] = 0.0; //not used*/

    MyRS485_Write(SafetyMessage, 1, WriteCount);
    /*while (ack = false)
    {
	    MyRS485_Write(SafetyMessage, 1, WriteCount);
	    usleep(delay);
	    MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
	    if (ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS &&
		    ReceiveInitMessage[0].Command == 0x003E)
	    {
		    ack = true;
		    cout << "safety passed" << endl;
	    }

    }*/



    // STEP 1: SEND TORQUE COMMAND FOR VERIFICATION
    //send position and torque command
    TrajectoryMessage[0].Command =
        RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = DESTINATION_ADDRESS;
    //32F position command [deg]
    TrajectoryMessage[0].DataFloat[0] = pos[0];
    //not used
    TrajectoryMessage[0].DataLong[1] = 0x00000000;
    //32F torque command [Nm]
    TrajectoryMessage[0].DataFloat[2] = 0;
    TrajectoryMessage[0].DataLong[3] = ((unsigned long) torqueDamping |
        ((unsigned long) controlMode << 8) | ((unsigned long)
        torqueKp << 16)); //U16|U8|U8
        
    /*TrajectoryMessage[1].Command =
        RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
    TrajectoryMessage[1].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;
    //32F position command [deg]
    TrajectoryMessage[1].DataFloat[0] = pos[1];
    //not used
    TrajectoryMessage[1].DataLong[1] = 0x00000000;
    //32F torque command [Nm]
    TrajectoryMessage[1].DataFloat[2] = 0;
    TrajectoryMessage[1].DataLong[3] = ((unsigned long) torqueDamping |
        ((unsigned long) controlMode << 8) | ((unsigned long)
        torqueKp << 16)); //U16|U8|U8*/

    //We send the command and we protect the process with a mutex
    /*
     * Param1(IN):  The buffer that contains all the messages.
     * Param2(IN):  Messages count.
     * Param3(OUT): Message sent count.
     */
    cout << "STEP 1: Send Torque Command for Verification" << endl;
    cout << "Initializing base position to: " << pos[0] << endl;
    cout << "Initializing wrist position to: " << pos[1] << endl;
    //for (int ii = 0; ii<500; ii++)
    //{
        //Joint6Command -= 40 * (0.0025);
        TrajectoryMessage[0].DataFloat[0] = pos[0];
        //pthread_mutex_lock (&APIMutex);
        MyRS485_Write(TrajectoryMessage, 1, WriteCount);
        //pthread_mutex_unlock (&APIMutex);
        usleep(delay);
    //}

    // STEP 2: Validate the torque command

    torqueValidation = false;

    //send position and torque command
    TrajectoryMessage[0].Command = GET_TORQUE_VALIDATION_REQUEST;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = DESTINATION_ADDRESS;
    //32F torque command to be tested [Nm]
    TrajectoryMessage[0].DataFloat[0] = 0;
    TrajectoryMessage[0].DataLong[1] = 0x00000000; //not used
    TrajectoryMessage[0].DataFloat[2] = 0x00000000; //not used
    TrajectoryMessage[0].DataLong[3] = 0x00000000; //not used
    
    /*TrajectoryMessage[1].Command = GET_TORQUE_VALIDATION_REQUEST;
    TrajectoryMessage[1].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;
    //32F torque command to be tested [Nm]
    TrajectoryMessage[1].DataFloat[0] = 0;
    TrajectoryMessage[1].DataLong[1] = 0x00000000; //not used
    TrajectoryMessage[1].DataFloat[2] = 0x00000000; //not used
    TrajectoryMessage[1].DataLong[3] = 0x00000000; //not used*/

    cout << "STEP 2: Request Torque Command Verification" << endl;

    while (torqueValidation == false)
    {
        //pthread_mutex_lock (&APIMutex);
        MyRS485_Write(TrajectoryMessage, 1, WriteCount);
       // pthread_mutex_unlock (&APIMutex);
        usleep(delay);
        MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
	    if (ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS &&
		    ReceiveInitMessage[0].Command == SEND_TORQUE_VALIDATION)
	    {
		    if (ReceiveInitMessage[0].DataLong[0] == 1)
		    {
			    cout << "Torque Validation True base : "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    torqueValidation = true;
		    }

		    else if (ReceiveInitMessage[0].DataLong[0] == 0)
		    {
			    cout << "Torque Validation False base : "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
		    }
		    else
		    {
			    cout << "ERROR READING TORQUE VALIDATION REPLY BASE: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
		    }
	    }
    }

    // STEP 3: Switch to torque control mode

    switchValidation = false;

    TrajectoryMessage[0].Command = SWITCH_CONTROL_MODE_REQUEST;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = DESTINATION_ADDRESS;
    unsigned short d1 = 0x00;
    unsigned short d2 = 0x00;
    unsigned short d3 = 0x00;
    TrajectoryMessage[0].DataLong[0] = ((unsigned short) 0x01 |
        ((unsigned short) d2 << 8) | ((unsigned short) d3 << 16 |
        ((unsigned short) d1 << 24))); //U24|U8
    TrajectoryMessage[0].DataLong[1]=0x00000000;
    TrajectoryMessage[0].DataLong[2]=0x00000000;
    TrajectoryMessage[0].DataLong[3]=0x00000000;
    
    

    cout << "STEP 3: Waiting for Torque Verification" << endl;

    do
    {
        cout << "waiting on reply for mode switch" << endl;
        //pthread_mutex_lock (&APIMutex);
        MyRS485_Write(TrajectoryMessage, 1, WriteCount);
        //pthread_mutex_unlock (&APIMutex);
        usleep(delay);
        MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
	    if (ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS &&
		    ReceiveInitMessage[0].Command == SWITCH_CONTROL_MODE_REPLY)
	    {
		    if (ReceiveInitMessage[0].DataLong[0] == 257)
		    {
			    cout << "Switch Control Mode TORQUE True Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    switchValidation = true;
		    }

		    else if (ReceiveInitMessage[0].DataLong[0] == 1)
		    {
			    cout << "Switch Control Mode POSITION True Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    switchValidation = true;
		    }

		    else if (ReceiveInitMessage[0].DataLong[0] == 0)
		    {
			    cout << "Switch Control Mode False Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    flag = 11;
		    }
		    else
		    {
			    cout << "ERROR READING SWITCH CONTROL MODE REPLY Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
		    }
	    }
    } while (switchValidation == false);

     cout << "Verified: Switching to Torque Mode" << endl;
}

void* Jaco2::ApplyU(unsigned char DESTINATION_ADDRESS, float us[6])
{
    // Step 4: Enjoy torque control mode!

    TrajectoryMessage[0].Command =
        RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = DESTINATION_ADDRESS;
    //32F position command [deg]
    //TrajectoryMessage[0].DataFloat[0] = pos[0];
    TrajectoryMessage[0].DataLong[1] = 0x00000000; //not used
    TrajectoryMessage[0].DataFloat[2] = us[0]; //32F torque command [1Nm]
    TrajectoryMessage[0].DataLong[3] = ((unsigned long) torqueDamping |
        ((unsigned long) controlMode << 8) |
        ((unsigned long) torqueKp << 16)); //U16|U8|U8



    //cout << "STEP 4: Enjoy Torque Mode" << endl;
    //for(int i=0; i<2000; i++)
    //{
        //TrajectoryMessage[0].DataFloat[0] = pos[0];
        //TrajectoryMessage[1].DataFloat[0] = pos[1];
        MyRS485_Write(TrajectoryMessage, 1, WriteCount);

        usleep(delay);
        MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
        //cout << "R E A D  C O U N T  " << ReadCount << endl;
        /*cout << "Base source address: " << ReceiveInitMessage[0].SourceAddress << endl;
        cout << "Wrist source address: " << ReceiveInitMessage[1].SourceAddress << endl;
        cout << "Base command: " << ReceiveInitMessage[0].Command << endl;
        cout << "Wrist command: " << ReceiveInitMessage[1].Command << endl;
        cout << "expected command: " << RS485_MSG_SEND_ALL_VALUES_1 << endl;
        cout << "base pos: " << ReceiveInitMessage[0].DataFloat[1] << endl;
        cout << "wrist pos: " << ReceiveInitMessage[1].DataFloat[1] << endl;*/
	    if (ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS)
	    {
		    //pos[0] = ReceiveInitMessage[0].DataFloat[1];
		    //pos[1] = ReceiveInitMessage[1].DataFloat[1];
		    //cout << "base pos: " << pos[0] << endl;
		    //cout << "wrist pos: " << pos[1] << endl;
		    //cout << "base source correct: " << ReceiveInitMessage[0].SourceAddress << endl;
	    }
	    else
	    {
	        cout << "base source incorrect: " << ReceiveInitMessage[0].SourceAddress << endl;
	    }
	    
	    if (ReceiveInitMessage[0].Command == RS485_MSG_SEND_ALL_VALUES_1)
	    {
		    pos[0] = ReceiveInitMessage[0].DataFloat[1];
		    //pos[1] = ReceiveInitMessage[1].DataFloat[1];
		    //cout << "base pos: " << pos[0] << endl;
		    //cout << "wrist pos: " << pos[1] << endl;
	    }
	    else
	    {
	        cout << "base response type incorrect: " << ReceiveInitMessage[0].Command << endl;
	    }
	    
    //}
}

void* Jaco2::GetFeedback(RS485_Message* args)
{
    
}

void Jaco2::Disconnect(unsigned char DESTINATION_ADDRESS)
{
    switchValidation = false;

    TrajectoryMessage[0].Command = SWITCH_CONTROL_MODE_REQUEST;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = DESTINATION_ADDRESS;
    unsigned short d1 = 0x00;
    unsigned short d2 = 0x00;
    unsigned short d3 = 0x00;
    TrajectoryMessage[0].DataLong[0] = ((unsigned short)0x00 |
	    ((unsigned short)d2 << 8) | ((unsigned short)d3 << 16 |
	    ((unsigned short)d1 << 24))); //U24|U8
    TrajectoryMessage[0].DataLong[1] = 0x00000000;
    TrajectoryMessage[0].DataLong[2] = 0x00000000;
    TrajectoryMessage[0].DataLong[3] = 0x00000000;    
    

    cout << "STEP 3: Waiting for Torque Verification" << endl;

    while (switchValidation == false)
    {
	    cout << "waiting on reply for mode switch" << endl;
	    //pthread_mutex_lock(&APIMutex);
	    MyRS485_Write(TrajectoryMessage, 1, WriteCount);
	    //pthread_mutex_unlock(&APIMutex);
	    usleep(delay);
	    MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
	    if (ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS &&
		    ReceiveInitMessage[0].Command == SWITCH_CONTROL_MODE_REPLY)
	    {
		    if (ReceiveInitMessage[0].DataLong[0] == 257)
		    {
			    cout << "Switch Control Mode TORQUE True Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    switchValidation = true;
		    }

		    else if (ReceiveInitMessage[0].DataLong[0] == 1)
		    {
			    cout << "Switch Control Mode POSITION True Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    switchValidation = true;
		    }

		    else if (ReceiveInitMessage[0].DataLong[0] == 0)
		    {
			    cout << "Switch Control Mode False Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    flag = 11;
		    }
		    else
		    {
			    cout << "ERROR READING SWITCH CONTROL MODE REPLY Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
		    }
	    }

    }
    cout << "Verified: Switching to position Mode" << endl;


    cout << "Exiting Main Control Loop" << endl;
}

int Jaco2::SendAndReceive(RS485_Message message[6], bool loop) {

    int joints_updated = 0;
    while(joints_updated < 6) {
        MyRS485_Write(message, packets_sent, WriteCount);
        usleep(delay);
        MyRS485_Read(MessageListIn, packets_read, ReadCount);
        //ProcessFeedback();

        if (loop == false) {
            break;
        }
    }
}
