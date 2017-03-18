#include <math.h>
#include <stdexcept.h>
#include <acado_optimal_control.hpp>

#include "dynamics.hpp"
#include "x8_param.hpp"


/** Calculates the quaternion rotation matrix R in SO(3).
 */
Matrix rquat(Vector q){
    double tol = 1e-6;
    if( abs( sqrt(q(0)*q(0)+q(1)*q(1)+q(2)*q(2)+q(3)*q(3)) - 1) > tol){
        throw std::invalid_argument( "Norm of 'q' must be equal to 1" );
    }

    Matrix sym_mat(3,3) I_3x3(3,3), R(3,3);

    sym_mat(0,0) = 0;       sym_mat(0,1) = -q(3);   sym_mat(0,2) = q(2);
    sym_mat(1,0) = q(3);    sym_mat(1,1) = 0;       sym_mat(1,2) = -q(1);
    sym_mat(2,0) = -q(2);   sym_mat(2,1) = q(1);    sym_mat(2,2) = 0;

    I_3x3(0,0) = 1; I_3x3(0,1) = 0; I_3x3(0,2) = 0;
    I_3x3(1,0) = 0; I_3x3(1,1) = 1; I_3x3(1,2) = 0;
    I_3x3(2,0) = 0; I_3x3(2,1) = 0; I_3x3(2,2) = 1;

    R = I_3x3 + 2*q(0)*sym_mat + 2*sym_mat*sym_mat;

    return R;
}



/** Calculates the differential state of the UAV.
 */
Vector dynamics(Vector pos, Vector quat, Vector vel, Vector omega, Vector tau){

    Matrix M_rb(6,6), C_rb(6,6):

    //_________________________________________________________________
    M_rb(0,0) = mass;   M_rb(0,1) = 0;    M_rb(0,2) = 0;
    M_rb(1,0) = 0;      M_rb(1,1) = mass; M_rb(1,2) = 0;
    M_rb(2,0) = 0;      M_rb(2,1) = 0;    M_rb(2,2) = mass;

    M_rb(3,0) = 0;      M_rb(3,1) = 0;    M_rb(3,2) = 0;
    M_rb(4,0) = 0;      M_rb(4,1) = 0;    M_rb(4,2) = 0;
    M_rb(5,0) = 0;      M_rb(5,1) = 0;    M_rb(5,2) = 0;

    M_rb(0,3) = 0;      M_rb(0,4) = 0;    M_rb(0,5) = 0;
    M_rb(1,3) = 0;      M_rb(1,4) = 0;    M_rb(1,5) = 0;
    M_rb(2,3) = 0;      M_rb(2,4) = 0;    M_rb(2,5) = 0;

    M_rb(3,3) = J_x;    M_rb(3,4) = 0;    M_rb(3,5) = -J_xz;
    M_rb(4,3) = 0;      M_rb(4,4) = J_y;  M_rb(4,5) = 0;
    M_rb(5,3) = -J_xz;  M_rb(5,4) = 0;    M_rb(5,5) = J_z;
    //_________________________________________________________________

    //_________________________________________________________________
    C_rb(0,0) = 0;      C_rb(0,1) = 0;      C_rb(0,2) = 0;
    C_rb(1,0) = 0;      C_rb(1,1) = 0;      C_rb(1,2) = 0;
    C_rb(2,0) = 0;      C_rb(2,1) = 0;      C_rb(2,2) = 0;

    C_rb(3,0) = 0;      C_rb(3,1) = mass*w; C_rb(3,2) = -mass*v;
    C_rb(4,0) = -mass*w;C_rb(4,1) = 0;      C_rb(4,2) = mass*u;
    C_rb(5,0) = mass*v; C_rb(5,1) = -mass*u;C_rb(5,2) = 0;

    C_rb(0,3) = 0;      C_rb(0,4) = mass*w; C_rb(0,5) = -mass*v;
    C_rb(1,3) = -mass*w;C_rb(1,4) = 0;      C_rb(1,5) = mass*u;
    C_rb(2,3) = mass*v; C_rb(2,4) = -mass*u;C_rb(2,5) = 0;

    C_rb(3,3) = 0;              C_rb(3,4) = -J_xz*p+J_z*r;  C_rb(3,5) = -J_y*q;
    C_rb(4,3) = J_xz*p-J_z*r;   C_rb(4,4) = 0;              C_rb(4,5) = -J_xz*r+J_x*p;
    C_rb(5,3) = J_y*q;          C_rb(5,4) = J_xz*r-J_x*p;   C_rb(5,5) = 0;
    //_________________________________________________________________

    Vector ny_dot = M_rb.getInverse()*(tau-C_rb*ny);
    
    Matrix T(4,3);
    
    T(0,0) = -0.5*quat(1); T(0,1) = -0.5*quat(2); T(0,2) = -0.5*quat(3);
    T(1,0) = 0.5*quat(0);  T(1,1) = -0.5*quat(3); T(1,2) = 0.5*quat(2);
    T(2,0) = 0.5*quat(4);  T(2,1) = 0.5*quat(0);  T(2,2) = -0.5*quat(1);
    T(3,0) = -0.5*quat(2); T(3,1) = 0.5*quat(1);  T(3,2) = 0.5*quat(0);

    Matrix J1 = rquat(quat);

    Vector pos_dot  = J1*vel;
    Vector quat_dot = T*omega - (1/2)*(1 - quat.transpose()*quat)*quat;

}
