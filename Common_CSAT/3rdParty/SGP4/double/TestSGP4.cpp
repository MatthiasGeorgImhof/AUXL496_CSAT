// #define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
// #include "doctest/doctest.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "SGP4.h"

using namespace std;
using namespace SGP4Funcs;

int main() {
    // 1. Define constants
    gravconsttype whichconst = wgs84; // Choose the gravity model (wgs72old, wgs72, wgs84)
    char opsmode = 'i'; // Operation mode ('a' for AFSPC, 'i' for improved)
    double startmfe, stopmfe, deltamin;

    // 2. Define TLE strings (example for NOAA 15, replace with your own TLE)
    char longstr1[] = "1 25338U 98030A   23334.81383711  .00000145  00000-0  12345-4 0  9991";
    char longstr2[] = "2 25338  98.7193  84.1645 0012514  75.0464 285.1163 14.25939947349222";
    
    // 3. Create an elsetrec structure
    elsetrec satrec;

    // Initialize satrec with some default values (these will be overwritten by twoline2rv)
    satrec.error = 0; // Initialize error code
    satrec.operationmode = opsmode;
    
    // Dummy values for start, stop, delta time for twoline2rv (not used here, but required by the function)
    startmfe = 0.0;  
    stopmfe  = 1440.0; 
    deltamin = 10.0;   

    // 4. Parse the TLE data using twoline2rv
    twoline2rv(longstr1, longstr2, ' ', 'i', opsmode, whichconst, startmfe, stopmfe, deltamin, satrec);

    if (satrec.error > 0) {
        cerr << "Error parsing TLE data. Error code: " << satrec.error << endl;
        return 1;
    }

    // 5. Define the time span for the prediction
    double start_time = 0.0; // Start time in minutes past epoch
    double end_time = 60.0;   // End time in minutes past epoch (e.g., 60 minutes)
    double step_size = 5.0;  // Time step in minutes (e.g., predict every 5 minutes)

    // 6. Loop through the time span and predict the satellite's position
    for (double tsince = start_time; tsince <= end_time; tsince += step_size) {
        double r[3]; // Position vector (km)
        double v[3]; // Velocity vector (km/s)

        // Call the SGP4 function to propagate the satellite's state
        bool result = sgp4(satrec, tsince, r, v);

        if (result) {
            // Print the results
            cout << "Time (minutes past epoch): " << tsince << endl;
            cout << "Position (km): " << r[0] << " " << r[1] << " " << r[2] << endl;
            cout << "Velocity (km/s): " << v[0] << " " << v[1] << " " << v[2] << endl;
            cout << endl;
        } else {
            cerr << "Error during propagation at time " << tsince << ". Error code: " << satrec.error << endl;
            break; // Exit the loop if an error occurs
        }
    }

    cout << "SGP4 propagation complete." << endl;

    return 0;
}