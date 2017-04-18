#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado/variables_grid/matrix_variables_grid.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>

using namespace std;

void objectiveFunction(double *x, double *f, void *user_data){

    f[0] = x[0];

}

void referenceFunction(double *x, double *f, void *user_data){

    double x_state = x[0];

    if(x_state < 2){
        x_state = 5.0;
        cout << "Inne i IF\n";
    }

    f[0] = x_state;

}

//CFunction J(1, &objectiveFunction);
//CFunction R(1, &referenceFunction);


int main(){

    USING_NAMESPACE_ACADO

    /* Introduce Variables */
    DifferentialState x, y;
        
    Control dx;
    Control dy;
    Control k;

    //IntermediateState ref("",1,1);

    DifferentialEquation f;

    f << dot(x) == dx;
    f << dot(y) == dy;

    
    /* PATH */
    VariablesGrid pathGrid;
    pathGrid.read( "./../examples/my_examples/path.txt" );
    cout << "Path to follow: \n";    
    pathGrid.print();
    //pathGrid.getTimePoints().print();
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

    DMatrix Q(1,1); Q(0,0) = 100; //Q(1,1) = 1;
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
    
    GnuplotWindow windowStates;
    windowStates.addSubplot(x, "X" );
    windowStates.addSubplot(y, "Y" );
    //windowStates.addSubplot(ref, "REF");

    GnuplotWindow windowControl;
    windowControl.addSubplot( dx, "DX" );
    windowControl.addSubplot( dy, "DY" );
    windowControl.addSubplot( k, "K" );

    algorithm << windowStates;
    algorithm << windowControl;

    algorithm.solve();

    return 0;
}
