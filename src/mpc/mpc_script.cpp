#include <acado_optimal_control.hpp>

#include "mpc_script.hpp"

BEGIN_NAMESPACE_ACADO

DMatrix optimize_path(VariablesGrid path, int xstart, int ystart,
                                          int dxstart, int dystart){

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
    ocp.minimizeLSQ( Q, h, path );


    /* Constraints */
    ocp.subjectTo( f );
    ocp.subjectTo( AT_START,  x == xstart  );
    ocp.subjectTo( AT_START,  y == ystart  );
    ocp.subjectTo( AT_START, dx == dxstart );
    ocp.subjectTo( AT_START, dy == dystart );

    ocp.subjectTo( -10 <= dx <= 10 );
    ocp.subjectTo( -10 <= dy <= 10 );


    /* Solve Problem */
    OptimizationAlgorithm algorithm( ocp );

    algorithm.solve();

    VariablesGrid states, controls;
    algorithm.getDifferentialStates(states);
    algorithm.getControls(controls);

    DMatrix ret_values(2,2);
    ret_values(0, 0) =   states(1, 0);
    ret_values(0, 1) =   states(1, 1);
    ret_values(1, 0) = controls(1, 0);
    ret_values(1, 1) = controls(1, 1);

    return ret_values;

}

CLOSE_NAMESPACE_ACADO

