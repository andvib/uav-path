#include <optimize_path.hpp>

#include <acado_optimal_control.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

using namespace std;


int main(){

    USING_NAMESPACE_ACADO

    /* Read file to array */
    float data[1001][2];
    ifstream file("./../pathgen.txt");

    for( int row = 0 ; row < 1001 ; ++row){
        string line;
        getline(file, line);

        if( !file.good() )
            break;

        stringstream iss(line);

        string val;
        getline(iss, val, ',');
        if( !iss.good() )
            break;

        stringstream convertor(val);
        convertor >> data[row][0];

        string val2;
        getline(iss, val2, '\n');

        stringstream convertor2(val2);
        convertor2 >> data[row][1];
    }

    /* Generate future path */
    float x = 0;
    float y = 0;
    int speed = 1;

    int horizon = 9;
    VariablesGrid path(2, 0, horizon, 10);
    DVector points(2);

    for( int j = 0 ; j < 10 ; j++ ){
        points(0) = x;
        points(1) = y;
        path.setVector(0, points);

        for( int timestep = 0 ; timestep < horizon ; timestep++ ){
            for( int i = 1001 ; i >= 0 ; --i){
                float x_dist = data[i][0] - x;
                float y_dist = data[i][1] - y;
                float radius = sqrt(x_dist*x_dist + y_dist*y_dist);

                if( (radius > speed-0.08) && (radius < speed+0.08) ){
                    //cout << "Found a point: " << data[i][0] << ", " << data[i][1] << "\n";  
                    x = data[i][0];
                    y = data[i][1];
                    points(0) = data[i][0];
                    points(1) = data[i][1];
                    path.setVector(timestep+1, points);
                    break;
                }
            }
        }
    
        path.print();

        DMatrix result = optimize_path( path, 0, 0, 1, 0); 

        //result.print();
        x = result(0,0);
        y = result(0,1);
    
        cout << "New position: " << x << ", " << y << "\n";
        clearAllStaticCounters();
    }

    return 0;
}
