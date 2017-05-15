#include <acado_optimal_control.hpp>
#include <iostream>
#include <fstream>

#include "optimize_path.hpp"

using namespace std;

ACADO::DMatrix optimize_path(ACADO::VariablesGrid path, int xstart, int ystart, int dxstart, int dystart){

    USING_NAMESPACE_ACADO


    /* Introduce Variables */
    DifferentialState x, y;

    Control dx;
    Control dy;


    /* Differential Equation */
    DifferentialEquation f;
    
    f << dot(x) == dx;
    f << dot(y) == dy;


    /* Least Squares Problem */
    Function h;

    h << x;
    h << y;

    DMatrix Q(2,2); Q.setIdentity();


    /* Initialize Optimal Control Problem */
    OCP ocp( path.getTimePoints() );
    path.getTimePoints().print();
    ocp.minimizeLSQ( Q, h, path );


    /* Constraints */
    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, x == xstart );
    ocp.subjectTo( AT_START, y == ystart );
    ocp.subjectTo( AT_START, dx == dxstart );
    ocp.subjectTo( AT_START, dy == dystart );

    ocp.subjectTo( -10 <= dx <= 10 );
    ocp.subjectTo( -10 <= dy <= 10 );


    /* Solve Problem */
    OptimizationAlgorithm algorithm( ocp );

    cout << "SOLVING\n";
    algorithm.solve();

    VariablesGrid states, controls;
    algorithm.getDifferentialStates(states);
    algorithm.getControls(controls);
    states.print();
    DMatrix ret_values(1,2);
    ret_values(0,0) = states(1, 0);
    ret_values(0,1) = states(1, 1);

    return ret_values;
}
