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
    ifstream file("./../pathgen.txt");
    double **path_data;
    
    path_data = readPathFile(file, path_length);


    /* Initialize MPC variables */
    int horizon_length = 15;    // Number of timesteps in the horizon
    int section_length = 10;    // Number of timesteps in the section
    double timestep    = 0.5;    // Duration of timestep [s]
    int no_sections    = 100;    // Number of sections to cover path

    double  x = 0.0;
    double  y = 0.0;
    double dx = 25.0;
    double dy = 25.0;
    

    /* Initialize result storage */
    double ** result;
    result = new double*[no_sections*section_length];
    for( int i = 0 ; i < no_sections*section_length ; i++){
        result[i] = new double[4];
    }


    /* For progress indication */
    int increment5 = 0.05 * no_sections;
    int countdown = increment5;
    int percent5 = 0;

    /* MPC Loop */
    cout << "Starting MPC.\n";
    for(int i = 0 ; i < no_sections ; i++){
        VariablesGrid path = generateHorizon(path_data, timestep,
                                             horizon_length, path_length,
                                             x, y, dx, dy);
        //path.print();

        DMatrix states = optimize_path(path, x, y, dx, dy);

        x =  states(section_length-1, 0);
        y =  states(section_length-1, 1);
        dx = states(section_length-1, 2);
        dy = states(section_length-1, 3);

        for(int j = 0 ; j < section_length ; j++){
            int idx = i*section_length + j;
            
            result[idx][0] = states(j, 0);
            result[idx][1] = states(j, 1);
            result[idx][2] = states(j, 2);
            result[idx][3] = states(j, 3);
        }

        clearAllStaticCounters();
        //cout << "New position: " << x << ", " << y << "\n";


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


    /* Delete path array */
    for( int i = 0 ; i < path_length ; i++){
        delete [] path_data[i];
    }
    delete [] path_data;
    
    return 0;
}
