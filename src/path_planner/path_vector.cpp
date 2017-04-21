#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado/variables_grid/matrix_variables_grid.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>

#include <iostream>
#include <fstream>

#include "path_vector.hpp"

using namespace std;


int optimize(){

    USING_NAMESPACE_ACADO

    /* Introduce Variables */
    DifferentialState x, y;
        
    Control dx;
    Control dy;
    Control k;

    AlgebraicState ref;

    DifferentialEquation f;

    f << dot(x) == dx;
    f << dot(y) == dy;

    
    /* PATH */
    VariablesGrid pathGrid;
    pathGrid.read( "./../vector_path.txt" );
    cout << "Path to follow: \n";    
    pathGrid.print();
    cout << "\n";

    cout << "Columns: " << pathGrid.getNumCols() << "\n" ;
    cout << "Rows: " << pathGrid.getNumRows() << "\n\n" ;

    Curve c1;
    
    c1.add( pathGrid, IM_LINEAR );

    GnuplotWindow window;
    window.addSubplot( c1, 0.0, 8.0, "Linear Path" );
    window.plot();

    double res;
    c1.evaluate(4.0, &res);
    cout << "Evaluate: " << res << "\n";


    /* LSQ */
    Function h;

    h << x;
    //h << y;

    DMatrix Q(1,1); Q(0,0) = 100;
    DVector ref(1);
    c1.evaluate(k.getComponent(0), ref);


    /* Constraints */
    OCP ocp( 0.0, 10.0, 100 );
    ocp.subjectTo( f );

    ocp.minimizeLSQ( Q, h, ref );


    ocp.subjectTo( AT_START,  x == 0 );
    ocp.subjectTo( AT_START,  y == 0 );
    ocp.subjectTo( AT_START, dx == 0 );
    ocp.subjectTo( AT_START, dy == 0 );
    ocp.subjectTo( AT_START,  k == 0 );

    ocp.subjectTo(  -1 <= dx <= 1 );
    ocp.subjectTo(  -1 <= dy <= 1 );
    ocp.subjectTo( 0.0 <= k  <= 9.0 );



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
