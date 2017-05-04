#include <acado_optimal_control.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

#include "mpc_script.hpp"
#include "path.hpp"

using namespace std;
USING_NAMESPACE_ACADO


int main(){

    /* Read path from file */
    int path_length = 120001;
    ifstream file("./../path_curved2.txt");
    double **path_data;
    
    path_data = readPathFile(file, path_length);


    /* Initialize MPC variables */
    int horizon_length = 10;    // Number of timesteps in the horizon
    int section_length = 8;    // Number of timesteps in the section
    double timestep    = 0.2;  // Duration of timestep [s]
    int no_sections    = 40;     // Number of sections to cover path


    //_________________________________________________________________
    /* Start Configuration */

    DVector X0(12);
    DVector U0(4);

    X0(0)  =  0.0;   // p_N
    X0(1)  =  0.0;   // p_E
    X0(2)  =150.0;   // h
    X0(3)  = 25.0;   // u
    X0(4)  =  0.0;   // v
    X0(5)  =  0.0;   // w
    X0(6)  =  0.0;   // phi
    X0(7)  =  0.066; // theta
    X0(8)  =  0.0;   // psi
    X0(9)  =  0.0;   // p
    X0(10) =  0.0;   // q
    X0(11) =  0.0;   // r

    U0(0) = -0.15; // Elevator
    U0(1) = 0.0; // Aileron
    U0(2) = 0.0; // Rudder
    U0(3) = 0.15; // Throttle



    //_________________________________________________________________
    /* Initialize result storage */

    double ** result;
    result = new double*[no_sections*section_length];
    for( int i = 0 ; i < no_sections*section_length ; i++){
        result[i] = new double[17];
    }

    result[0][0] = 0.0;
    for(int k = 1 ; k < 13 ; k++){
        result[0][k] = X0(k-1);
    }
    result[0][13] = U0(0);
    result[0][14] = U0(1);
    result[0][15] = U0(2);
    result[0][16] = U0(3);


    //_________________________________________________________________
    /* For progress indication */

    int increment5 = 0.05 * no_sections;
    int countdown = increment5;
    int percent5 = 0;



    //_________________________________________________________________
    /* MPC Loop */
    cout << "Starting MPC.\n";
    for(int i = 0 ; i < no_sections ; i++){
        int index = findClosestPoint(X0(0), X0(1), path_data, path_length);

        double closest_x = path_data[index][0];
        double closest_y = path_data[index][1];

        VariablesGrid path = generateHorizon(path_data, timestep,
                                             horizon_length, path_length,
                                             closest_x, closest_y);
        //path.print();

        DMatrix states = optimize_path(path, X0, U0);

        for(int l = 0 ; l < 12 ; l++){
            X0(l)  =  states(section_length-1, l);
        }
        U0(0)  =  states(section_length-1, 12);
        U0(1)  =  states(section_length-1, 13);
        U0(2)  =  states(section_length-1, 14);
        U0(3)  =  states(section_length-1, 15);



        for(int j = 0 ; j < section_length ; j++){
            int idx = i*section_length + j;
            result[idx][0] = idx*timestep;
            
            for(int k = 1 ; k < 17 ; k++){
                result[idx][k]  = states(j, k-1);
            }
        }

        clearAllStaticCounters();


        //cout << "New Position: " << X0(0) << ", " << X0(1) << "\n";
        //cout << "Section no.: " << i << "\n";


        /* Progress bar */
        if( --countdown == 0){
            percent5++;
            cout << "\rOptimization progress: " << string(percent5, '|')
                                                << percent5*5 << "%";
            countdown = increment5;
            cout.flush();
        }
    }
    cout << "\n";
    saveResults(result, no_sections*section_length);



    //_________________________________________________________________
    /* Delete path array */
    for( int i = 0 ; i < path_length ; i++){
        delete [] path_data[i];
    }
    delete [] path_data;
    
    return 0;
}
