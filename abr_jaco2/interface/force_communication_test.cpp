#include "jaco2_rs485.h"
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#define MILLION 1E6

using namespace std;

int main ()
{
    struct timespec requestStart, requestEnd;
    vector< double > times;
    // clock_t start;
    double duration;
    cout << "Instantiating..." << endl;
    //Jaco2()* arm = new Jaco2(int 4);
    Jaco2 arm(3);
    cout << "Connecting.." << endl;
    arm.Connect();
    cout << "Initializing Position Mode" << endl;
    arm.InitPositionMode();
    // arm.SendTargetAnglesSetup();
    // float q[6] = {0.0, 2.79, 2.62, 4.71, 0.0, 3.04};
    // arm.SendTargetAngles(q);
    cout << "Initializing Force Mode" << endl;
    arm.InitForceMode();
    cout << "Sending forces" << endl;
    float u[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int ii=0; ii<10000; ii++){
        // start = clock();
        clock_gettime(CLOCK_REALTIME, &requestStart);
        arm.SendForces(u);
        clock_gettime(CLOCK_REALTIME, &requestEnd);
        duration = ( requestEnd.tv_sec - requestStart.tv_sec )
          + ( requestEnd.tv_nsec - requestStart.tv_nsec )
          / MILLION;

        // time in ms
        // duration = double(clock()-start)*1000 / CLOCKS_PER_SEC;
         times.push_back(duration);
    }
    cout << "Initializing Position Mode" << endl;
    arm.InitPositionMode();
    cout << "Disconnecting..." << endl;
    arm.Disconnect();

    // print out time vector
    // for (vector<double>::const_iterator i = times.begin(); i != times.end(); ++i)
    //     cout << *i << ' ';

    // write time vector to file
    ofstream output_file("./loop_times.txt");
    ostream_iterator<double> output_iterator(output_file, "\n");
    copy(times.begin(), times.end(), output_iterator);

    cout << "Done." << endl;
}
