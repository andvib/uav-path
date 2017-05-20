#include <acado_optimal_control.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <string>

#include "mpc_script.hpp"
#include "path.hpp"

using namespace std;
USING_NAMESPACE_ACADO


int main(int argc, char **argv){

    /* Read path from file */
    int path_length = 15315;
    char* file_in = argv[1];
    cout << file_in << "\n";
    ifstream file(file_in);
    double **path_data;
    
    path_data = readPathFile(file, path_length);


    /* Initialize MPC variables */
    int horizon_length = atoi(argv[3]);     // Number of timesteps in the horizon
    int section_length = 5;      // Number of timesteps in the section
    double timestep    = 0.2;    // Duration of timestep [s]
    int no_sections    = 40;     // Number of sections to cover path


    string temp = (string) argv[2];
    string file_out = "results/" + temp + "_" + to_string(horizon_length) + ".m";
    cout << file_out << "\n";

    //_________________________________________________________________
    /* Start Configuration */

    DVector X0(12);
    DVector U0(4);
    DVector DU0(4);

    X0(0)  = 0.0; // p_N
    X0(1)  = 0.0; // p_E
    X0(2)  = 0.0; // h
    X0(3)  = 0.0; // u
    X0(4)  = 0.0; // v
    X0(5)  = 0.0; // w
    X0(6)  = 0.0; // phi
    X0(7)  = 0.0; // theta
    X0(8)  = 0.0; // psi
    X0(9)  = 0.0; // p
    X0(10) = 0.0; // q
    X0(11) = 0.0; // r

    U0(0)  = 0.0; // Elevator
    U0(1)  = 0.0; // Aileron
    U0(2)  = 0.0; // Rudder
    U0(3)  = 0.0; // Throttle

    DU0(0) = 0.0; // D_Elevator
    DU0(1) = 0.0; // D_Aileron
    DU0(2) = 0.0; // D_Rudder
    DU0(3) = 0.0; // D_Throttle



    //_________________________________________________________________
    /* Trim Conditions */

    DVector X0_trim(12);
    DVector U0_trim(4);
    DVector DU0_trim(4);

    X0_trim(0)  =   0.0;   // p_N
    X0_trim(1)  =   0.0;   // p_E
    X0_trim(2)  = 150.0;   // h
    X0_trim(3)  =  25.0;   // u
    X0_trim(4)  =   0.0;   // v
    X0_trim(5)  =   0.0;   // w
    X0_trim(6)  =   0.0;   // phi
    X0_trim(7)  =   0.066; // theta
    X0_trim(8)  =   0.0;   // psi
    X0_trim(9)  =   0.0;   // p
    X0_trim(10) =   0.0;   // q
    X0_trim(11) =   0.0;   // r

    U0_trim(0) = -0.15; // Elevator
    U0_trim(1) =  0.0;  // Aileron
    U0_trim(2) =  0.0;  // Rudder
    U0_trim(3) =  0.1;  // Throttle

    DU0_trim(0) = 0.0; // D_Elevator
    DU0_trim(1) = 0.0; // D_Aileron
    DU0_trim(2) = 0.0; // D_Rudder
    DU0_trim(3) = 0.0; // D_Throttle



    //_________________________________________________________________
    /* Initialize result storage */

    double ** result;
    result = new double*[no_sections*section_length];
    for( int i = 0 ; i < no_sections*section_length ; i++){
        result[i] = new double[21];
    }

    result[0][0] = 0.0;
    for(int k = 1 ; k < 13 ; k++){
        result[0][k] = X0(k-1) + X0_trim(k-1);
    }
    result[0][13] = U0(0) + U0_trim(0);
    result[0][14] = U0(1) + U0_trim(1);
    result[0][15] = U0(2) + U0_trim(2);
    result[0][16] = U0(3) + U0_trim(3);


    //_________________________________________________________________
    /* For progress indication */

    int increment5 = 0.05 * no_sections;
    int countdown = increment5;
    int percent5 = 0;

    double elapsed_secs;
    
    //_________________________________________________________________
    /* MPC Loop */
    cout << "Starting MPC.\n";
    clock_t begin = clock();
    for(int i = 0 ; i < no_sections ; i++){
        int index = findClosestPoint(X0(0), X0(1), X0(2)+X0_trim(2),
                                     X0(6)+X0_trim(6), X0(7)+X0_trim(7),
                                     X0(8)+X0_trim(8),
                                     path_data, path_length);

        double closest_x = path_data[index][0];
        double closest_y = path_data[index][1];

        VariablesGrid path = generateHorizon(path_data, timestep,
                                             horizon_length, path_length,
                                             closest_x, closest_y);

        DMatrix states = optimize_path(path, X0, U0, DU0);

        // Prepare initial conditions for next step
        for(int l = 0 ; l < 12 ; l++){
            X0(l) = states(section_length-1, l);
        }

        U0(0)  = states(section_length-1, 12);
        U0(1)  = states(section_length-1, 13);
        U0(2)  = states(section_length-1, 14);
        U0(3)  = states(section_length-1, 15);

        DU0(0) = states(section_length-1, 16);
        DU0(1) = states(section_length-1, 17);
        DU0(2) = states(section_length-1, 18);
        DU0(3) = states(section_length-1, 19);


        // Save results
        for(int j = 0 ; j < section_length ; j++){
            int idx = i*section_length + j;
            result[idx][0] = idx*timestep;
            
            for(int k = 1 ; k < 13 ; k++){
                result[idx][k]  = states(j, k-1) + X0_trim(k-1);
            }

            result[idx][13]  = states(j, 12) + U0_trim(0);
            result[idx][14]  = states(j, 13) + U0_trim(1);
            result[idx][15]  = states(j, 14) + U0_trim(2);
            result[idx][16]  = states(j, 15) + U0_trim(3);

            result[idx][17]  = states(j, 16);
            result[idx][18]  = states(j, 17);
            result[idx][19]  = states(j, 18);
            result[idx][20]  = states(j, 19);
            
        }

        clearAllStaticCounters();

        cout << "Section no.: " << i << "/" << no_sections << "\n";
    }
    cout << "\n";

    clock_t end = clock();
    elapsed_secs = double(end-begin) / CLOCKS_PER_SEC;
    
    cout << "Time elapsed: " << elapsed_secs << "\n\n";

    saveResults(result, no_sections*section_length, path_data, path_length, elapsed_secs, file_out);



    //_________________________________________________________________
    /* Delete path array */
    for( int i = 0 ; i < path_length ; i++){
        delete [] path_data[i];
    }
    delete [] path_data;
    
    return 0;
}
