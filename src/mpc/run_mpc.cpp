#include <acado_optimal_control.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

#include "mpc_script.hpp"

using namespace std;
USING_NAMESPACE_ACADO

double** readPathFile(ifstream& file, int path_length){

    /* Read file to array */
    double** path_data;
    path_data = new double*[path_length];
    for( int i = 0 ; i < path_length ; i++ ){
        path_data[i] = new double[2];
    }

    for( int row = 0 ; row < path_length ; ++row ){

        string line;
        getline(file, line);
        
        if( !file.good() ){
            cout << "readPathFile: Something went wrong when reading" \
                        " a line in the path file!\n";            
            break;
        }

        stringstream iss(line);

        string x_value;
 
        getline(iss, x_value, ',');
        if( !iss.good() ){
            cout << "readPathFile: Something went wrong when reading" \
                        " the x-value in the path file!\n";            
            break;
        }
        //cout << "X: " << x_value << "\n";
        
        stringstream x_convertor(x_value);
        x_convertor >> path_data[row][0];

        string y_value;
        getline(iss, y_value, '\n');
        /*if( !iss.good() ){
            cout << "readPathFile: Something went wrong when reading" \
                        " the y-value in the path file!\n";            
            break;
        }*/

        //cout << "y: " << y_value << "\n";

        stringstream y_convertor(y_value);
        y_convertor >> path_data[row][1];
    }
    
    return path_data;
}


int saveResults(double ** results, int length){
    ofstream file("./../results/results.m");
    if(file.is_open()){
        file << "STATES = [";
        for( int i = 0 ; i < length ; i++){
            file << results[i][0] << ", " << results[i][1] << ";\n";
        }
        file << "];\n";

        file << "CONTROLS = [";
        for( int i = 0 ; i < length ; i++){
            file << results[i][2] << ", " << results[i][3] << ";\n";
        }
        file << "];\n";

        return 0;    
    }else{
        cout << "saveResults: Could not create file!\n";
       return 1;
    }
}


VariablesGrid generateHorizon(double** path_data, int timestep,
                              int horizon_length, int path_length,
                              double x_start,  double y_start,
                              double dx_start, double dy_start){

    /* Initalize storage */
    int no_timesteps = horizon_length/timestep;
    VariablesGrid path(2, 0, horizon_length, no_timesteps+1);

    DVector points(2);


    /* Initialize variables */
    double  x =  x_start;
    double  y =  y_start;
    double dx = dx_start;
    double dy = dy_start;

    double speed    = sqrt(dx*dx + dy*dy); // [m/s]
    double distance = speed*timestep;      // [m]


    /* Generate path */
    for( int step = 0 ; step < no_timesteps ; step++ ){
        for( int i = path_length-1 ; i >= 0 ; --i ){            
            double x_dist = path_data[i][0] - x;
            double y_dist = path_data[i][1] - y;
            double radius = sqrt(x_dist*x_dist + y_dist*y_dist);

            if( (radius > distance-0.08) && (radius < distance+0.08)){
                x = path_data[i][0];
                y = path_data[i][1];
                
                points(0) = path_data[i][0];
                points(1) = path_data[i][1];
                path.setVector(step+1, points);
                break;
            }
        }
    }
    return path;
}



int main(){

    /* Read path from file */
    int path_length = 90001;
    ifstream file("./../pathgen.txt");
    double **path_data;
    
    path_data = readPathFile(file, path_length);


    /* Initialize MPC variables */
    int horizon_length = 20;    // Number of timesteps in the horizon
    int section_length = 10;    // Number of timesteps in the section
    int timestep       =  1;    // Duration of timestep [s]
    int no_sections    =  600;    // Number of sections to cover path

    double  x = 0.0;
    double  y = 0.0;
    double dx = 1.0;
    double dy = 0.0;
    

    /* Initialize result storage */
    double ** result;
    result = new double*[no_sections*section_length];
    for( int i = 0 ; i < no_sections*section_length ; i++){
        result[i] = new double[4];
    }


    for(int i = 0 ; i < no_sections ; i++){
        VariablesGrid path = generateHorizon(path_data, timestep,
                                             horizon_length, path_length,
                                             x, y, dx, dy);

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

    }

    saveResults(result, no_sections*section_length);



    /* Delete path array */
    for( int i = 0 ; i < path_length ; i++){
        delete [] path_data[i];
    }
    delete [] path_data;


    return 0;
}
