#include <acado_optimal_control.hpp>
//#include <acado_gnuplot.hpp>
//#include <acado/variables_grid/matrix_variables_grid.hpp>

#include <iostream>
#include <fstream>

using namespace std;

int main(){

    USING_NAMESPACE_ACADO

    /* Introduce Variables */
    DifferentialState x, y;
        
    Control dx;
    Control dy;


    DifferentialEquation f;

    f << dot(x) == dx;
    f << dot(y) == dy;

    
    /* PATH */
    VariablesGrid path;
    path.read( "./../path.txt" );
    cout << "Path to follow: \n";    
    path.print();

    cout << "\n";

    cout << "Timepoints of the path: \n";
    Grid time = path.getTimePoints();
    time.print();

    /* LSQ */
    Function h;

    h << x;
    h << y;

    DMatrix Q(2,2); Q(0,0) = 1; Q(1,1) = 1;

    /* Constraints */
    OCP ocp( path.getTimePoints() );
    ocp.subjectTo( f );

    ocp.minimizeLSQ( Q, h, path );

    ocp.subjectTo( AT_START, x == 0 );
    ocp.subjectTo( AT_START, y == 0 );
    ocp.subjectTo( AT_START, dx == 0 );
    ocp.subjectTo( AT_START, dy == 0 );

    ocp.subjectTo( -10 <= dx <= 10 );
    ocp.subjectTo( -10 <= dy <= 10 );


    /* Results */
    OptimizationAlgorithm algorithm(ocp);
    
    LogRecord logRecord( LOG_AT_END);
    logRecord.setPrintScheme( PS_MATLAB );
    logRecord.addItem( LOG_DIFFERENTIAL_STATES, "STATES");
    logRecord.addItem( LOG_CONTROLS, "CONTROLS");

    algorithm << logRecord;

    algorithm.solve();

    algorithm.getLogRecord( logRecord );

    ofstream myfile;
    myfile.open("./../results/output.m");
    logRecord.print(myfile);
    myfile.close();

    return 0;
}
