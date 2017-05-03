#include <acado_optimal_control.hpp>

#include "mpc_script.hpp"



ACADO::DMatrix optimize_path(ACADO::VariablesGrid path, double xstart, double ystart,
                                          double dxstart, double dystart){

    USING_NAMESPACE_ACADO


    /* Introduce Variables */
    DifferentialState x, y;
    
    Control dx;
    Control dy;

    //IntermediateState speed;

    /* Differential Equation */
    DifferentialEquation f;

    f << dot(x) == dx;
    f << dot(y) == dy;

    //speed = sqrt(dx*dx + dy*dy);

    /* Least Squares Problem */
    Function h;

    h << x;
    h << y;

    DMatrix Q(2,2); Q.setIdentity();


    /*Function i;
    i << speed;
    DMatrix Qi(1,1); Qi.setIdentity();
    DVector Ri(1); Ri(0) = 1.0;*/

    /* Initialize Optimal Control Problem */
    OCP ocp( path.getTimePoints() );

    ocp.minimizeLSQ( Q, h, path );

    //ocp.minimizeLSQ( Qi, i, Ri );

    /* Constraints */
    ocp.subjectTo( f );
    ocp.subjectTo( AT_START,  x == xstart  );
    ocp.subjectTo( AT_START,  y == ystart  );
    ocp.subjectTo( AT_START, dx == dxstart );
    ocp.subjectTo( AT_START, dy == dystart );

    ocp.subjectTo( -35 <= dx <= 35 );
    ocp.subjectTo( -35 <= dy <= 35 );


    /* Solve Problem */
    OptimizationAlgorithm algorithm( ocp );

    //algorithm.set( INTEGRATOR_TYPE, INT_RK78 );
    //algorithm.set( DISCRETIZATION_TYPE, SINGLE_SHOOTING );

    algorithm.set(PRINT_COPYRIGHT, BT_FALSE);
    algorithm.set(PRINTLEVEL, NONE);

    algorithm.solve();

    VariablesGrid states, controls;
    algorithm.getDifferentialStates(states);
    algorithm.getControls(controls);

    DMatrix ret_values(path.getLastTime(), 4);

    for(int i = 0 ; i < path.getLastTime() ; i++){
        ret_values(i, 0) =   states(i+1, 0);
        ret_values(i, 1) =   states(i+1, 1);
        ret_values(i, 2) = controls(i+1, 0);
        ret_values(i, 3) = controls(i+1, 1);
    }

    return ret_values;

}

