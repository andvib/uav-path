#include <acado_optimal_control.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

#include "mpc_script.hpp"

using namespace std;


float ** readFile(ifstream& file, int path_length){

    /* Read file to array */
    float** path_data;
    path_data = new float*[path_length];
    for( int i = 0 ; i < path_length ; i++ ){
        path_data[i] = new float[2];
    }

    for( int row = 0 ; row < path_length ; ++row ){

        string line;
        getline(file, line);
        
        if( !file.good() ){
            cout << "Something went wrong when reading" \
                        " a line in the path file!\n";            
            break;
        }

        stringstream iss(line);

        string x_value;
 
        getline(iss, x_value, ',');
        if( !iss.good() ){
            cout << "Something went wrong when reading" \
                        " the x-value in the path file!\n";            
            break;
        }
        //cout << "X: " << x_value << "\n";
        
        stringstream x_convertor(x_value);
        x_convertor >> path_data[row][0];

        string y_value;
        getline(iss, y_value, '\n');
        /*if( !iss.good() ){
            cout << "Something went wrong when reading" \
                        " the y-value in the path file!\n";            
            break;
        }*/

        //cout << "y: " << y_value << "\n";

        stringstream y_convertor(y_value);
        y_convertor >> path_data[row][1];
    }
    
    return path_data;

}



int main(){

    USING_NAMESPACE_ACADO

    /* Initialization Variables */
    int path_length = 1001;
    ifstream file("./../pathgen.txt");
    float **path_data;
    path_data = readFile(file, path_length);




    /* Delete path array */
    for( int i = 0 ; i < path_length ; i++){
        delete [] path_data[i];
    }
    delete [] path_data;


    return 0;
}
