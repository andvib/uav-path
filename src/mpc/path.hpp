#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <acado_optimal_control.hpp>

using namespace std;

double** readPathFile(ifstream& file, int path_length);

int saveResults(double ** results, int length, double** path_data, int path_length);

ACADO::VariablesGrid generateHorizon(double** path_data, double timestep,
                                     int horizon_length, int path_length,
                                     double x_start,  double y_start);

int findClosestPoint(double x, double y, double h,
                     double phi, double theta, double psi,
                     double** path_data, int length);
