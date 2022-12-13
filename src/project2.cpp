/*
NYCU DME 0811070 TingWei Ou
TODO:
    
*/
#include<iostream>
#include<Eigen/Dense>
#include<cmath>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h>

//NAMESPACE
using namespace Eigen;
using namespace std;

class Path_Planning{

    public:
    //FUNCTION
    void joint_variables_mode_output();
    void CartesianPoint_output();
    void output_check(double JOINT_VARIABLE_SOLUTION[6]);
    void set();
    double column_mul(Matrix<double, 4, 1> A, Matrix<double, 4, 1> B);
    void cartesian_planning();
    void rotation_2_quaternion(Matrix<double, 4, 4> POS_ROTATION);

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

    Matrix<double, 4, 4> T6;

    float quaternion_x = 0;
    float quaternion_y = 0;
    float quaternion_z = 0;
    float quaternion_w = 1;


    //WHILE FLAG
    bool while_flag = false;

    //output_data


    private:

    double PI = 3.141592;
    double d2 = 6.375000;
    float sample_time = 0.002;
    float t_acc = 0.2;
    float t_trans = 0.5;
    Matrix<double, 4, 4> POS_A;
    Matrix<double, 4, 4> POS_B;
    Matrix<double, 4, 4> POS_C;
    
};

//setting the private matrix
void Path_Planning::set(){
    POS_A << 0, 0, -1, 10,
            -1, 0, 0, 20,
            0, 1, 0, 30,
            0, 0, 0, 1;

    POS_B << 1, 0, 0, 30,
            0, 1, 0, 20,
            0, 0, 1, 10,
            0, 0, 0, 1;

    POS_C << 0, -1, 0, -10,
            0, 0, 1, -20,
            -1, 0, 0, -30,
            0, 0, 0, 1;

}

//this function converts the rotation matrix into a quaternion representation
//input: 4x4 matrix
//output : change the global variable quaternion_ 
void Path_Planning::rotation_2_quaternion(Matrix<double, 4, 4> POS_ROTATION){

}


void Path_Planning::cartesian_planning(){
    

}

double Path_Planning::column_mul(Matrix<double, 4, 1> A, Matrix<double, 4, 1> B){
    return A(0,0)*B(0,0) + A(1, 0)*B(1, 0) + A(2, 0)*B(2, 0);
}

//this function handles for computing joint variables input and output Cartesian Point & eular angle
//this function using local variable 'x' 'y' 'z' 'a' 'A' 'B' 'C'
//this function using local double matrix 'A1' 'A2' 'A3' 'A4' 'A5' 'A6' 'T6'
//this function will calculate & print Cartesian Point
void Path_Planning::joint_variables_mode_output(){
    
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
void Path_Planning::CartesianPoint_output(){
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
void Path_Planning::output_check(double JOINT_VARIABLE_SOLUTION[6]){
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

int main(int argc, char **argv){

    ros::init(argc, argv,"path_planning");
	ros::NodeHandle n;
    ros::Publisher cartesian_visualization_pub = n.advertise<visualization_msgs::Marker>("cartesian_visualization_marker", 10);

    ros::Rate r(30);

    Path_Planning P;
    P.set();

    float f = 0.0;
    while(ros::ok()){
        //initialization
        visualization_msgs::Marker points, line_strip, line_list;
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = line_list.ns = "points_and_lines";
        points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;

        points.id = 0;
        line_strip.id = 1;
        line_list.id = 2;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;
        line_list.scale.x = 0.1;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;
        line_list.scale.x = 0.1;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;


        /*
        for(float t = 0, t < 1, t = t + 0.02){

            //z-axis
            points.pose.orientation.x = line_strip.pose.orientation.x = line_list.pose.orientation.x = P.quaternion_x;
            points.pose.orientation.y = line_strip.pose.orientation.y = line_list.pose.orientation.y = P.quaternion_y;
            points.pose.orientation.z = line_strip.pose.orientation.z = line_list.pose.orientation.z = P.quaternion_z;
            points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = P.quaternion_w;
            

        }
        */
        

        for (int i = 0; i < 100; i++){
            float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
            float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
            
            geometry_msgs::Point p;
            p.x = i - 50;
            p.y = y;
            p.z = z;

            points.points.push_back(p);
            line_strip.points.push_back(p);

            // The line list needs two points for each line
            line_list.points.push_back(p);
            p.z += 1.0;
            line_list.points.push_back(p);
            }

        cartesian_visualization_pub.publish(points);
        cartesian_visualization_pub.publish(line_strip);
        cartesian_visualization_pub.publish(line_list);

        r.sleep();

        f += 0.04;

    }

    return 0;
}