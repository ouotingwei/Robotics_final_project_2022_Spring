/*
NYCU DME 0811070 TingWei Ou
TODO:
    
*/
#include<iostream>
#include<Eigen/Dense>
#include<cmath>
#include<ros/ros.h>

//NAMESPACE
using namespace Eigen;
using namespace std;

class Kinematics{

    public:
    //FUNCTION
    void joint_variables_mode_output();
    void CartesianPoint_output();
    void output_check(double JOINT_VARIABLE_SOLUTION[6]);

    //VARIABLE
    char mode;
    double CartesianPoint[4]={0,0,0,0}; // [0] = n, [1] = o, [2] = a, [3] = p
    double joint_variables[6]={0,0,0,0,0,0}; // [0] = θ1, [1] = θ2, [2] = d3 , [3] = θ4, [4] = θ5, [5] = d6
    Eigen::Matrix<double, 4, 4> noap_input;
    //8 solution by calculation
    double JOINT_VARIABLE_SOLUTION_1[6] = {0,0,0,0,0,0};
    double JOINT_VARIABLE_SOLUTION_2[6] = {0,0,0,0,0,0};
    double JOINT_VARIABLE_SOLUTION_3[6] = {0,0,0,0,0,0};
    double JOINT_VARIABLE_SOLUTION_4[6] = {0,0,0,0,0,0};
    double JOINT_VARIABLE_SOLUTION_5[6] = {0,0,0,0,0,0};
    double JOINT_VARIABLE_SOLUTION_6[6] = {0,0,0,0,0,0};
    double JOINT_VARIABLE_SOLUTION_7[6] = {0,0,0,0,0,0};
    double JOINT_VARIABLE_SOLUTION_8[6] = {0,0,0,0,0,0};

    //WHILE FLAG
    bool while_flag = false;
    
    private:
    //VARIABLE
    double PI = 3.141592;
    double d2 = 6.375000;
    
};

//this function handles for computing joint variables input and output Cartesian Point & eular angle
//this function using local variable 'x' 'y' 'z' 'a' 'A' 'B' 'C'
//this function using local double matrix 'A1' 'A2' 'A3' 'A4' 'A5' 'A6' 'T6'
//this function will calculate & print Cartesian Point
void Kinematics::joint_variables_mode_output(){
    
    double x, y, z, A, B, C, a;
    Matrix<double, 4, 4> A1;
    Matrix<double, 4, 4> A2;
    Matrix<double, 4, 4> A3;
    Matrix<double, 4, 4> A4;
    Matrix<double, 4, 4> A5;
    Matrix<double, 4, 4> A6;
    Matrix<double, 4, 4> T6;

    A1 <<    cos(joint_variables[0]),  0, -1*sin(joint_variables[0]), 0,
             sin(joint_variables[0]),  0,    cos(joint_variables[0]), 0,
          0                         , -1,                          0, 0,
          0                         ,  0,                          0, 1;

    A2 <<    cos(joint_variables[1]),  0,    sin(joint_variables[1]), 0,
             sin(joint_variables[1]),  0, -1*cos(joint_variables[1]), 0,
          0                         ,  1,                          0, 16.1925,      // UNIT = cm
          0                         ,  0,                          0, 1;

    A3 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, joint_variables[2],
          0, 0, 0, 1;

    A4 <<    cos(joint_variables[3]),  0, -1*sin(joint_variables[3]), 0,
             sin(joint_variables[3]),  0,    cos(joint_variables[3]), 0,
          0                         , -1,                          0, 0,
          0                         ,  0,                          0, 1;

    A5 <<    cos(joint_variables[4]),  0,    sin(joint_variables[4]), 0,
             sin(joint_variables[4]),  0, -1*cos(joint_variables[4]), 0,
          0                         ,  1,                          0, 0,
          0                         ,  0,                          0, 1;

    A6 << cos(joint_variables[5]), -1*sin(joint_variables[5]), 0, 0,
          sin(joint_variables[5]),    cos(joint_variables[5]), 0, 0,
          0                      , 0                         , 1, 0,
          0                      , 0                         , 0, 1;
    
    T6 << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;
    

    T6 = (A1*(A2*(A3*(A4*(A5*A6)))));  // 1T6 MATRIX

    //euler angle z-y-z (x, y, z)
    x = T6(0,3);
    y = T6(1,3);
    z = T6(2,3);
    //euler angle z-y-z (ϕ, θ, ψ)
    a = atan2(T6(1, 2), T6(0, 2));
    A = a*180/PI;
    B = (atan2((T6(0, 2)*cos(a)) + (T6(1, 2)*sin(a)), T6(2,2))*180/PI);
    C = (atan2((-1*T6(0, 0)*sin(a) + (T6(1, 0)*cos(a))), (-1*T6(0, 1)*sin(a)) + (T6(1, 1)*cos(a)))*180/PI);

    //show solution
    cout<<"(n, o, a, p) = "<<endl;
    cout<<T6<<endl;
    cout<<" "<<endl;
    cout<<"Cartesian Point (x, y, z, ϕ, θ, ψ) = "<<endl;
    cout<<"     ( "<<x<<", "<<y<<", "<<z<<", "<<A<<", "<<B<<", "<<C<<" )"<<endl;
    cout<<"-----------------------------------"<<endl;
}

//this function handles inverse kinematics & calculate six JOINT VARIABLE
//this function using local variable 'double theta' & 'temp_a' 'temp_b'
//this function using global matrix 'noap_input'
//this finction will assign value to global array 'JOINT_VARIABLE_SOLUTION'
void Kinematics::CartesianPoint_output(){
    double theta_1_1, theta_1_2, 
           theta_2_1, theta_2_2, theta_2_3, theta_2_4,
           theta_4_1_1, theta_4_1_2, theta_4_2_1, theta_4_2_2, theta_4_3_1, theta_4_3_2, theta_4_4_1, theta_4_4_2,
           theta_5_1_1, theta_5_1_2, theta_5_2_1, theta_5_2_2, theta_5_3_1, theta_5_3_2, theta_5_4_1, theta_5_4_2,
           theta_6_1_1, theta_6_1_2, theta_6_2_1, theta_6_2_2, theta_6_3_1, theta_6_3_2, theta_6_4_1, theta_6_4_2,
           d_3_1, d_3_2;

    double temp_fr, temp_bk;
    double temp_a, temp_b;

    //dx = 03 ; dy = 13 ; dz = 23

    //θ1 -> two solution
    //θ1-1 (20)
    theta_1_1 = atan2(noap_input(1, 3), noap_input(0, 3)) - atan2(d2, sqrt(pow(noap_input(0, 3), 2) + pow(noap_input(1, 3), 2) - pow(d2, 2)));
        JOINT_VARIABLE_SOLUTION_5[0] = (theta_1_1*180/PI);
        JOINT_VARIABLE_SOLUTION_6[0] = (theta_1_1*180/PI);
        JOINT_VARIABLE_SOLUTION_7[0] = (theta_1_1*180/PI);
        JOINT_VARIABLE_SOLUTION_8[0] = (theta_1_1*180/PI);

    //θ1-2 (-74)
    theta_1_2 = (atan2(noap_input(1, 3), noap_input(0, 3)) - atan2(d2, -1*sqrt(pow(noap_input(0, 3), 2) + pow(noap_input(1, 3), 2) - pow(d2, 2))));
        JOINT_VARIABLE_SOLUTION_1[0] = (theta_1_2*180/PI);
        JOINT_VARIABLE_SOLUTION_2[0] = (theta_1_2*180/PI);
        JOINT_VARIABLE_SOLUTION_3[0] = (theta_1_2*180/PI);
        JOINT_VARIABLE_SOLUTION_4[0] = (theta_1_2*180/PI);

    //θ2 -> four solution
    //θ2-1 (20)
    theta_2_1 = atan2((cos(theta_1_1)*noap_input(0, 3)) + (sin(theta_1_1)*noap_input(1, 3)), noap_input(2, 3));
        JOINT_VARIABLE_SOLUTION_5[1] = (theta_2_1*180/PI);
        JOINT_VARIABLE_SOLUTION_6[1] = (theta_2_1*180/PI);

    //θ2-2 (160)
    theta_2_2 = atan2((cos(theta_1_1)*noap_input(0, 3)) + (sin(theta_1_1)*noap_input(1, 3)), -1*noap_input(2, 3));
        JOINT_VARIABLE_SOLUTION_3[1] = (theta_2_2*180/PI);
        JOINT_VARIABLE_SOLUTION_4[1] = (theta_2_2*180/PI);

    //θ2-3 (-20)
    theta_2_3 = atan2((cos(theta_1_2)*noap_input(0, 3)) + (sin(theta_1_2)*noap_input(1, 3)), noap_input(2, 3));
        JOINT_VARIABLE_SOLUTION_1[1] = (theta_2_3*180/PI);
        JOINT_VARIABLE_SOLUTION_2[1] = (theta_2_3*180/PI);

    //θ2-4 (-160)
    theta_2_4 = atan2((cos(theta_1_2)*noap_input(0, 3)) + (sin(theta_1_2)*noap_input(1, 3)), -1*noap_input(2, 3));
        JOINT_VARIABLE_SOLUTION_7[1] = (theta_2_4*180/PI);
        JOINT_VARIABLE_SOLUTION_8[1] = (theta_2_4*180/PI);

    //d3 -> two solution
    //d_3_1 (20)
    d_3_1 = noap_input(2, 3) / cos(theta_2_1);
        JOINT_VARIABLE_SOLUTION_1[2] = d_3_1;
        JOINT_VARIABLE_SOLUTION_2[2] = d_3_1;
        JOINT_VARIABLE_SOLUTION_5[2] = d_3_1;
        JOINT_VARIABLE_SOLUTION_6[2] = d_3_1;

    //d_3_2 (-20)
    d_3_2 = noap_input(2, 3) / cos(theta_2_2);
        JOINT_VARIABLE_SOLUTION_3[2] = d_3_2;
        JOINT_VARIABLE_SOLUTION_4[2] = d_3_2;
        JOINT_VARIABLE_SOLUTION_7[2] = d_3_2;
        JOINT_VARIABLE_SOLUTION_8[2] = d_3_2;

    //θ4 -> eight solution
    //θ4-1 (79)
    theta_4_1_1 = atan2((-1*sin(theta_1_2)*noap_input(0, 2)) + (cos(theta_1_2)*noap_input(1, 2)), (cos(theta_1_2)*cos(theta_2_3)*noap_input(0, 2)) + (sin(theta_1_2)*cos(theta_2_3)* noap_input(1, 2)) - (sin(theta_2_3)*noap_input(2, 2)));
        JOINT_VARIABLE_SOLUTION_1[3] = theta_4_1_1*180/PI;

    //θ4-1 - 180 (-100)
    theta_4_1_2 = ((theta_4_1_1*180/PI) - 180)*PI/180;
        JOINT_VARIABLE_SOLUTION_2[3] = theta_4_1_2*180/PI;

    //θ4-2(100)
    theta_4_2_1 = atan2((-1*sin(theta_1_2)*noap_input(0, 2)) + (cos(theta_1_2)*noap_input(1, 2)), (cos(theta_1_2)*cos(theta_2_2)*noap_input(0, 2)) + (sin(theta_1_2)*cos(theta_2_2)* noap_input(1, 2)) - (sin(theta_2_2)*noap_input(2, 2)));
        JOINT_VARIABLE_SOLUTION_3[3] = theta_4_2_1*180/PI;
    
    //θ4-2 - 180  (-79)
    theta_4_2_2 = ((theta_4_2_1*180/PI) - 180)*PI/180;
        JOINT_VARIABLE_SOLUTION_4[3] = theta_4_2_2*180/PI;
    
    //θ4-3 (20)
    theta_4_3_1 = atan2((-1*sin(theta_1_1)*noap_input(0, 2)) + (cos(theta_1_1)*noap_input(1, 2)), (cos(theta_1_1)*cos(theta_2_1)*noap_input(0, 2)) + (sin(theta_1_1)*cos(theta_2_1)* noap_input(1, 2)) - (sin(theta_2_1)*noap_input(2, 2)));
        JOINT_VARIABLE_SOLUTION_5[3] = theta_4_3_1*180/PI;

    //θ4-3 - 180 (-160)
    theta_4_3_2 = ((theta_4_3_1*180/PI) - 180)*PI/180;
        JOINT_VARIABLE_SOLUTION_6[3] = theta_4_3_2*180/PI;

    //θ4-4 (160)
    theta_4_4_1 = atan2((-1*sin(theta_1_1)*noap_input(0, 2)) + (cos(theta_1_1)*noap_input(1, 2)), (cos(theta_1_1)*cos(theta_2_4)*noap_input(0, 2)) + (sin(theta_1_1)*cos(theta_2_4)* noap_input(1, 2)) - (sin(theta_2_4)*noap_input(2, 2)));
        JOINT_VARIABLE_SOLUTION_7[3] = theta_4_4_1*180/PI;

    //θ4-4 - 180 (-20)
    theta_4_4_2 = ((theta_4_4_1*180/PI) - 180)*PI/180;
        JOINT_VARIABLE_SOLUTION_8[3] = theta_4_4_2*180/PI;

    //θ5 -> eight solution
    //θ5-1 
    temp_a = ((cos(theta_1_2)*cos(theta_2_3)*cos(theta_4_1_1) - sin(theta_1_2)*sin(theta_4_1_1))*noap_input(0 ,2)) + ((sin(theta_1_2)*cos(theta_2_3)*cos(theta_4_1_1) + cos(theta_1_2)*sin(theta_4_1_1))*noap_input(1, 2)) + ((-1*sin(theta_2_3)*cos(theta_4_1_1))*noap_input(2, 2));
    temp_b = (cos(theta_1_2)*sin(theta_2_3)*noap_input(0, 2)) + (sin(theta_1_2)*sin(theta_2_3)*noap_input(1, 2)) + (cos(theta_2_3)*noap_input(2, 2));
    theta_5_1_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_1[4] = theta_5_1_1*180/PI;

    //θ5-1-2
    temp_a = ((cos(theta_1_2)*cos(theta_2_3)*cos(theta_4_1_2) - sin(theta_1_2)*sin(theta_4_1_2))*noap_input(0 ,2)) + ((sin(theta_1_2)*cos(theta_2_3)*cos(theta_4_1_2) + cos(theta_1_2)*sin(theta_4_1_2))*noap_input(1, 2)) + ((-1*sin(theta_2_3)*cos(theta_4_1_2))*noap_input(2, 2));
    temp_b = (cos(theta_1_2)*sin(theta_2_3)*noap_input(0, 2)) + (sin(theta_1_2)*sin(theta_2_3)*noap_input(1, 2)) + (cos(theta_2_3)*noap_input(2, 2));
    theta_5_1_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_2[4] = theta_5_1_2*180/PI;

    //θ5-2-1
    temp_a = ((cos(theta_1_2)*cos(theta_2_2)*cos(theta_4_2_1) - sin(theta_1_2)*sin(theta_4_2_1))*noap_input(0 ,2)) + ((sin(theta_1_2)*cos(theta_2_2)*cos(theta_4_2_1) + cos(theta_1_2)*sin(theta_4_2_1))*noap_input(1, 2)) + ((-1*sin(theta_2_2)*cos(theta_4_2_1))*noap_input(2, 2));
    temp_b = (cos(theta_1_2)*sin(theta_2_2)*noap_input(0, 2)) + (sin(theta_1_2)*sin(theta_2_2)*noap_input(1, 2)) + (cos(theta_2_2)*noap_input(2, 2));
    theta_5_2_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_3[4] = theta_5_2_1*180/PI;

    //θ5-2-2
    temp_a = ((cos(theta_1_2)*cos(theta_2_2)*cos(theta_4_2_2) - sin(theta_1_2)*sin(theta_4_2_2))*noap_input(0 ,2)) + ((sin(theta_1_2)*cos(theta_2_2)*cos(theta_4_2_2) + cos(theta_1_2)*sin(theta_4_2_2))*noap_input(1, 2)) + ((-1*sin(theta_2_2)*cos(theta_4_2_2))*noap_input(2, 2));
    temp_b = (cos(theta_1_2)*sin(theta_2_2)*noap_input(0, 2)) + (sin(theta_1_2)*sin(theta_2_2)*noap_input(1, 2)) + (cos(theta_2_2)*noap_input(2, 2));
    theta_5_2_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_4[4] = theta_5_2_2*180/PI;

    //θ5-3-1
    temp_a = ((cos(theta_1_1)*cos(theta_2_1)*cos(theta_4_3_1) - sin(theta_1_1)*sin(theta_4_3_1))*noap_input(0 ,2)) + ((sin(theta_1_1)*cos(theta_2_1)*cos(theta_4_3_1) + cos(theta_1_1)*sin(theta_4_3_1))*noap_input(1, 2)) + ((-1*sin(theta_2_1)*cos(theta_4_3_1))*noap_input(2, 2));
    temp_b = (cos(theta_1_1)*sin(theta_2_1)*noap_input(0, 2)) + (sin(theta_1_1)*sin(theta_2_1)*noap_input(1, 2)) + (cos(theta_2_1)*noap_input(2, 2));
    theta_5_3_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_5[4] = theta_5_3_1*180/PI;

    //θ5-3-2
    temp_a = ((cos(theta_1_1)*cos(theta_2_1)*cos(theta_4_3_2) - sin(theta_1_1)*sin(theta_4_3_2))*noap_input(0 ,2)) + ((sin(theta_1_1)*cos(theta_2_1)*cos(theta_4_3_2) + cos(theta_1_1)*sin(theta_4_3_2))*noap_input(1, 2)) + ((-1*sin(theta_2_1)*cos(theta_4_3_2))*noap_input(2, 2));
    temp_b = (cos(theta_1_1)*sin(theta_2_1)*noap_input(0, 2)) + (sin(theta_1_1)*sin(theta_2_1)*noap_input(1, 2)) + (cos(theta_2_1)*noap_input(2, 2));
    theta_5_3_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_6[4] = theta_5_3_2*180/PI;


    //θ5-4-1
    temp_a = ((cos(theta_1_1)*cos(theta_2_4)*cos(theta_4_4_1) - sin(theta_1_1)*sin(theta_4_4_1))*noap_input(0 ,2)) + ((sin(theta_1_1)*cos(theta_2_4)*cos(theta_4_4_1) + cos(theta_1_1)*sin(theta_4_4_1))*noap_input(1, 2)) + ((-1*sin(theta_2_4)*cos(theta_4_4_1))*noap_input(2, 2));
    temp_b = (cos(theta_1_1)*sin(theta_2_4)*noap_input(0, 2)) + (sin(theta_1_1)*sin(theta_2_4)*noap_input(1, 2)) + (cos(theta_2_4)*noap_input(2, 2));
    theta_5_4_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_7[4] = theta_5_4_1*180/PI;

    //θ5-4-2
    temp_a = ((cos(theta_1_1)*cos(theta_2_4)*cos(theta_4_4_2) - sin(theta_1_1)*sin(theta_4_4_2))*noap_input(0 ,2)) + ((sin(theta_1_1)*cos(theta_2_4)*cos(theta_4_4_2) + cos(theta_1_1)*sin(theta_4_4_2))*noap_input(1, 2)) + ((-1*sin(theta_2_4)*cos(theta_4_4_2))*noap_input(2, 2));
    temp_b = (cos(theta_1_1)*sin(theta_2_4)*noap_input(0, 2)) + (sin(theta_1_1)*sin(theta_2_4)*noap_input(1, 2)) + (cos(theta_2_4)*noap_input(2, 2));
    theta_5_4_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_8[4] = theta_5_4_2*180/PI;

    //θ6 -> eight solution
    //θ6-1-1
    temp_fr = -1*((cos(theta_1_2)*cos(theta_2_3)*sin(theta_4_1_1)) + (sin(theta_1_2)*cos(theta_4_1_1)));
    temp_bk = -1*((sin(theta_1_2)*cos(theta_2_3)*sin(theta_4_1_1)) - (cos(theta_1_2)*cos(theta_4_1_1)));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_3)*sin(theta_4_1_1))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_3)*sin(theta_4_1_1))*noap_input(2, 1);
    theta_6_1_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_1[5] = theta_6_1_1*180/PI;

    //θ6-1-2
    temp_fr = (-1*(cos(theta_1_2)*cos(theta_2_3)*sin(theta_4_1_2))) - (sin(theta_1_2)*cos(theta_4_1_2));
    temp_bk = (-1*(sin(theta_1_2)*cos(theta_2_3)*sin(theta_4_1_2))) + (cos(theta_1_2)*cos(theta_4_1_2));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_3)*sin(theta_4_1_2))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_3)*sin(theta_4_1_2))*noap_input(2, 1);
    theta_6_1_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_2[5] = theta_6_1_2*180/PI;

    //θ6-2-1
    temp_fr = (-1*(cos(theta_1_2)*cos(theta_2_2)*sin(theta_4_2_1))) - (sin(theta_1_2)*cos(theta_4_2_1));
    temp_bk = (-1*(sin(theta_1_2)*cos(theta_2_2)*sin(theta_4_2_1))) + (cos(theta_1_2)*cos(theta_4_2_1));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_2)*sin(theta_4_2_1))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_2)*sin(theta_4_2_1))*noap_input(2, 1);
    theta_6_2_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_3[5] = theta_6_2_1*180/PI;

    //θ6-2-2
    temp_fr = (-1*(cos(theta_1_2)*cos(theta_2_2)*sin(theta_4_2_2))) - (sin(theta_1_2)*cos(theta_4_2_2));
    temp_bk = (-1*(sin(theta_1_2)*cos(theta_2_2)*sin(theta_4_2_2))) + (cos(theta_1_2)*cos(theta_4_2_2));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_2)*sin(theta_4_2_2))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_2)*sin(theta_4_2_2))*noap_input(2, 1);
    theta_6_2_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_4[5] = theta_6_2_2*180/PI;

    //θ6-3-1
    temp_fr = (-1*(cos(theta_1_1)*cos(theta_2_1)*sin(theta_4_3_1))) - (sin(theta_1_1)*cos(theta_4_3_1));
    temp_bk = (-1*(sin(theta_1_1)*cos(theta_2_1)*sin(theta_4_3_1))) + (cos(theta_1_1)*cos(theta_4_3_1));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_1)*sin(theta_4_3_1))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_1)*sin(theta_4_3_1))*noap_input(2, 1);
    theta_6_3_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_5[5] = theta_6_3_1*180/PI;

    //θ6-3-2
    temp_fr = (-1*(cos(theta_1_1)*cos(theta_2_1)*sin(theta_4_3_2))) - (sin(theta_1_1)*cos(theta_4_3_2));
    temp_bk = (-1*(sin(theta_1_1)*cos(theta_2_1)*sin(theta_4_3_2))) + (cos(theta_1_1)*cos(theta_4_3_2));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_1)*sin(theta_4_3_2))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_1)*sin(theta_4_3_2))*noap_input(2, 1);
    theta_6_3_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_6[5] = theta_6_3_2*180/PI;

    //θ6-4-1
    temp_fr = (-1*(cos(theta_1_1)*cos(theta_2_4)*sin(theta_4_4_1))) - (sin(theta_1_1)*cos(theta_4_4_1));
    temp_bk = (-1*(sin(theta_1_1)*cos(theta_2_4)*sin(theta_4_4_1))) + (cos(theta_1_1)*cos(theta_4_4_1));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_4)*sin(theta_4_4_1))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_4)*sin(theta_4_4_1))*noap_input(2, 1);
    theta_6_4_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_7[5] = theta_6_4_1*180/PI;

    //θ6-4-1
    temp_fr = (-1*(cos(theta_1_1)*cos(theta_2_4)*sin(theta_4_4_2))) - (sin(theta_1_1)*cos(theta_4_4_2));
    temp_bk = (-1*(sin(theta_1_1)*cos(theta_2_4)*sin(theta_4_4_2))) + (cos(theta_1_1)*cos(theta_4_4_2));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_4)*sin(theta_4_4_2))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_4)*sin(theta_4_4_2))*noap_input(2, 1);
    theta_6_4_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_8[5] = theta_6_4_2*180/PI; 

}

//this function handles for showing the answer calculated by inverse kinematics
//this function using global matrix & determine whether the angle limit is met
void Kinematics::output_check(double JOINT_VARIABLE_SOLUTION[6]){
    cout<<"<sol>Corresponding variables (θ1, θ2, d3, θ4, θ5, θ6): "<<endl;
    cout<<" (";
    for(int i = 0; i < 6; i++){
        cout<<"  "<<JOINT_VARIABLE_SOLUTION[i]<<" ";
    }
    cout<<" )"<<endl;

    if(JOINT_VARIABLE_SOLUTION[0] >= 160 || JOINT_VARIABLE_SOLUTION[0] <= -160){
        cout<<"[error] θ1 is out of range"<<endl;

    }

    if(JOINT_VARIABLE_SOLUTION[1] >= 125 || JOINT_VARIABLE_SOLUTION[1] <= -125){
        cout<<"[error] θ2 is out of range"<<endl;
    }

    if(JOINT_VARIABLE_SOLUTION[2] >= 30 || JOINT_VARIABLE_SOLUTION[2] <= -30){
        cout<<"[error] d3 is out of range"<<endl;
    }

    if(JOINT_VARIABLE_SOLUTION[3] >= 140 || JOINT_VARIABLE_SOLUTION[3] <= -140){
        cout<<"[error] θ4 is out of range"<<endl;
    }

    if(JOINT_VARIABLE_SOLUTION[4] >= 100 || JOINT_VARIABLE_SOLUTION[4] <= -100){
        cout<<"[error] θ5 is out of range"<<endl;
    }

    if(JOINT_VARIABLE_SOLUTION[5] >= 260 || JOINT_VARIABLE_SOLUTION[0] <= -260){
        cout<<"[error] θ6 is out of range"<<endl;
    }

    cout<<endl;
    cout<<endl;
        
}

class Motion_Planning{
    public:

    private:

};

int main(int argc, char **argv){

    ros::init(argc, argv,"path_planning");
	ros::NodeHandle n;

    Kinematics K;
    Motion_Planning M;

    return 0;
}