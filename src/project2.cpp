/*
NYCU DME 0811070 TingWei Ou
*/
#include<iostream>
#include<Eigen/Dense>
#include<cmath>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float32.h>

//NAMESPACE
using namespace Eigen;
using namespace std;

class Path_Planning{

    public:
    //FUNCTION
    void Kinematics();
    void Inverse_Kinematics(Matrix<double, 4, 4> noap_input);
    void set();
    double column_mul(Matrix<double, 3, 1> A, Matrix<double, 3, 1> B);
    void cartesian_position_planning(float t);
    void cartesian_velocity_planning(float t);
    void cartesian_acceleration_planning(float t);
    void rotation_2_quaternion(Matrix<double, 4, 4> POS_ROTATION);
    float boundary_Forward(float A, float B);
    float divide(float A, float B);
    double angle_normalization(double theta);
    void find_ok_pos();
    void joint_move_angle(float t);
    bool output_check(double JOINT_VARIABLE_SOLUTION[6]);

    //VARIABLE
    char mode;
    double CartesianPoint[4]={0,0,0,0}; // [0] = n, [1] = o, [2] = a, [3] = p
    double joint_variables[6]={0,0,0,0,0,0}; // [0] = θ1, [1] = θ2, [2] = d3 , [3] = θ4, [4] = θ5, [5] = d6
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

    Quaterniond quaternion;

    double POS_A_OK[6] = {-70.1668, -27.2045, 33.7313, -107.7851, 81.0769, 64.1944};
    double POS_B_OK[6] = {7.0042, 72.7549, 33.7313, 0, -72.7549, -7.0042};
    double POS_C_OK[6] = {109.8332, 27.2045, -33.7313, -22.0744, 64.5293, 9.8929};

    //FLAG
    bool while_flag = false;
    bool rotation_or_linear = false; //false = linear , true = linear

    //output_data
    float position_x, position_y, position_z, position_A, position_B, position_C;
    float velocity_x, velocity_y, velocity_z, velocity_A, velocity_B, velocity_C;
    float acceleration_x, acceleration_y, acceleration_z, acceleration_A, acceleration_B, acceleration_C;

    float A_X, A_Y, A_Z, A_A, A_B, A_C; // A = φ, B = θ, C = ψ
    float B_X, B_Y, B_Z, B_A, B_B, B_C;
    float C_X, C_Y, C_Z, C_A, C_B, C_C;
    float joint_a[6], joint_b[6], joint_c[6];

    float j_1_position, j_2_position, j_3_position, j_4_position, j_5_position, j_6_position;
    float j_1_velocity, j_2_velocity, j_3_velocity, j_4_velocity, j_5_velocity, j_6_velocity;
    float j_1_acceleration, j_2_acceleration, j_3_acceleration, j_4_acceleration, j_5_acceleration, j_6_acceleration;

    Matrix<double, 4, 4> position_T;
    Matrix<double, 4, 4> velocity_T;
    Matrix<double, 4, 4> acceleration_T;

    private:

    double PI = 3.141592;
    double d2 = 6.375000;
    float sample_time = 0.002;
    float t_acc = 0.2;
    float T = 0.5;
    float trans_T = 2*t_acc;
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

    Matrix3d rotation_matrix;
    rotation_matrix << POS_ROTATION(0, 0), POS_ROTATION(0, 1), POS_ROTATION(0, 2),
                       POS_ROTATION(1, 0), POS_ROTATION(1, 1), POS_ROTATION(1, 2),
                       POS_ROTATION(2, 0), POS_ROTATION(2, 1), POS_ROTATION(2, 2);

    quaternion = rotation_matrix;

    //cout<< quaternion.x() <<" "<< quaternion.y() <<" "<< quaternion.z() <<" "<< quaternion.w() << endl;
    //cout<< POS_ROTATION <<endl;

}

float Path_Planning::boundary_Forward(float A, float B){
    return ((B - A)*(0.5 - t_acc) / 0.5) + A;
}

float Path_Planning::divide(float A, float B){
    if(A == 0 && B == 0){
        return 0;
    }else if(A == 0 && B != 0 ){
        return 0;
    }else if(B == 0 && A != 0){
        return 999999999;
    }else{
        return A / B;
    }
}

void Path_Planning::cartesian_position_planning(float t){

    A_X = POS_A(0, 3);
    A_Y = POS_A(1, 3);
    A_Z = POS_A(2, 3);

    B_X = POS_B(0, 3);
    B_Y = POS_B(1, 3);
    B_Z = POS_B(2, 3);

    C_X = POS_C(0, 3);
    C_Y = POS_C(1, 3);
    C_Z = POS_C(2, 3);

    A_B = atan2(sqrt(pow(POS_A(2, 0), 2) + pow(POS_A(2, 1), 2)), POS_A(2, 2));
    A_A = atan2(divide(POS_A(1, 2), sin(A_B)), divide(POS_A(0, 2), sin(A_B)));
    A_C = atan2(divide(POS_A(2, 1), sin(A_B)), -1*divide(POS_A(2, 0), sin(A_B)));

    B_B = atan2(sqrt(pow(POS_B(2, 0), 2) + pow(POS_B(2, 1), 2)), POS_B(2, 2));
    B_A = atan2(divide(POS_B(1, 2), sin(B_B)), divide(POS_B(0, 2), sin(B_B)));
    B_C = atan2(divide(POS_B(2, 1), sin(B_B)), -1*divide(POS_B(2, 0), sin(B_B)));

    C_B = atan2(sqrt(pow(POS_C(2, 0), 2) + pow(POS_C(2, 1), 2)), POS_C(2, 2));
    C_A = atan2(divide(POS_C(1, 2), sin(C_B)), divide(POS_C(0, 2), sin(C_B)));
    C_C = atan2(divide(POS_C(2, 1), sin(C_B)) , -1*divide(POS_C(2, 0), sin(C_B)));


    if(t < 0.3){
        
        float h = t / T;

        position_x = (B_X - A_X)*h + A_X;
        position_y = (B_Y - A_Y)*h + A_Y;
        position_z = (B_Z - A_Z)*h + A_Z;
        position_A = (B_A - A_A)*h + A_A;
        position_B = (B_B - A_B)*h + A_B;
        position_C = (B_C - A_C)*h + A_C;

        //cout<<"B_C = "<<B_C <<" A_C = "<<A_C<<endl;

        //cout<< position_A << " " << position_B << " " << position_C<<endl;

        position_T << cos(position_A)*cos(position_B)*cos(position_C) - sin(position_A)*sin(position_C),    -1*cos(position_A)*cos(position_B)*sin(position_C) - sin(position_A)*cos(position_C),   cos(position_A)*sin(position_B),    position_x,
                        sin(position_A)*cos(position_B)*cos(position_C) + cos(position_A)*sin(position_C),    -1*sin(position_A)*cos(position_B)*sin(position_C) + cos(position_A)*cos(position_C),     sin(position_A)*sin(position_B),    position_y,
                        -1*sin(position_B)*cos(position_C),     sin(position_B)*sin(position_C),    cos(position_B),    position_z,
                        0, 0, 0, 1;

        rotation_2_quaternion(position_T);


    }else if(t >= 0.3 && t < 0.7){
        t = t - 0.5;

        float h = (t + t_acc) / (trans_T);

        position_x = ((((C_X - B_X)*t_acc / T) + (boundary_Forward(A_X, B_X) - B_X))*(2 - h)*pow(h, 2) - 2*((boundary_Forward(A_X, B_X) - B_X)))*h + B_X + (boundary_Forward(A_X, B_X) - B_X);
        position_y = ((((C_Y - B_Y)*t_acc / T) + (boundary_Forward(A_Y, B_Y) - B_Y))*(2 - h)*pow(h, 2) - 2*((boundary_Forward(A_Y, B_Y) - B_Y)))*h + B_Y + (boundary_Forward(A_Y, B_Y) - B_Y);
        position_z = ((((C_Z - B_Z)*t_acc / T) + (boundary_Forward(A_Z, B_Z) - B_Z))*(2 - h)*pow(h, 2) - 2*((boundary_Forward(A_Z, B_Z) - B_Z)))*h + B_Z + (boundary_Forward(A_Z, B_Z) - B_Z);
        position_A = ((((C_A - B_A)*t_acc / T) + (boundary_Forward(A_A, B_A) - B_A))*(2 - h)*pow(h, 2) - 2*((boundary_Forward(A_A, B_A) - B_A)))*h + B_A + (boundary_Forward(A_A, B_A) - B_A);
        position_B = ((((C_B - B_B)*t_acc / T) + (boundary_Forward(A_B, B_B) - B_B))*(2 - h)*pow(h, 2) - 2*((boundary_Forward(A_B, B_B) - B_B)))*h + B_B + (boundary_Forward(A_B, B_B) - B_B);
        position_C = ((((C_C - B_C)*t_acc / T) + (boundary_Forward(A_C, B_C) - B_C))*(2 - h)*pow(h, 2) - 2*((boundary_Forward(A_C, B_C) - B_C)))*h + B_C + (boundary_Forward(A_C, B_C) - B_C);

        //cout<<"B_A = "<<B_A <<" A_A = "<<A_A<<endl;

        //cout<< position_A << " " << position_B << " " << position_C<<endl;

        position_T << cos(position_A)*cos(position_B)*cos(position_C) - sin(position_A)*sin(position_C),    -1*cos(position_A)*cos(position_B)*sin(position_C) - sin(position_A)*cos(position_C),   cos(position_A)*sin(position_B),    position_x,
                        sin(position_A)*cos(position_B)*cos(position_C) + cos(position_A)*sin(position_C),    -1*sin(position_A)*cos(position_B)*sin(position_C) + cos(position_A)*cos(position_C),     sin(position_A)*sin(position_B),    position_y,
                        -1*sin(position_B)*cos(position_C),     sin(position_B)*sin(position_C),    cos(position_B),    position_z,
                        0, 0, 0, 1;

        rotation_2_quaternion(position_T);

        t = t + 0.5;

        //cout<< boundary_Backward(B_X, C_X) <<endl;

    }else{

        t = t - 0.5;

        float h = t / T;

        position_x = (C_X - B_X)*h + B_X;
        position_y = (C_Y - B_Y)*h + B_Y;
        position_z = (C_Z - B_Z)*h + B_Z;
        position_A = (C_A - B_A)*h + B_A;
        position_B = (C_B - B_B)*h + B_B;
        position_C = (C_C - B_C)*h + B_C;

        //cout<<"B_A = "<< B_A <<" A_A = "<<A_A<<endl;

        //cout<< position_A << " " << position_B << " " << position_C<<endl;

        position_T << cos(position_A)*cos(position_B)*cos(position_C) - sin(position_A)*sin(position_C),    -1*cos(position_A)*cos(position_B)*sin(position_C) - sin(position_A)*cos(position_C),   cos(position_A)*sin(position_B),    position_x,
                        sin(position_A)*cos(position_B)*cos(position_C) + cos(position_A)*sin(position_C),    -1*sin(position_A)*cos(position_B)*sin(position_C) + cos(position_A)*cos(position_C),     sin(position_A)*sin(position_B),    position_y,
                        -1*sin(position_B)*cos(position_C),     sin(position_B)*sin(position_C),    cos(position_B),    position_z,
                        0, 0, 0, 1;
                        
        rotation_2_quaternion(position_T);

        t = t + 0.5;

    }

}

void Path_Planning::cartesian_velocity_planning(float t){
    A_X = POS_A(0, 3);
    A_Y = POS_A(1, 3);
    A_Z = POS_A(2, 3);

    B_X = POS_B(0, 3);
    B_Y = POS_B(1, 3);
    B_Z = POS_B(2, 3);

    C_X = POS_C(0, 3);
    C_Y = POS_C(1, 3);
    C_Z = POS_C(2, 3);

    A_B = atan2(sqrt(pow(POS_A(2, 0), 2) + pow(POS_A(2, 1), 2)), POS_A(2, 2));
    A_A = atan2(divide(POS_A(1, 2), sin(A_B)), divide(POS_A(0, 2), sin(A_B)));
    A_C = atan2(divide(POS_A(2, 1), sin(A_B)), -1*divide(POS_A(2, 0), sin(A_B)));

    B_B = atan2(sqrt(pow(POS_B(2, 0), 2) + pow(POS_B(2, 1), 2)), POS_B(2, 2));
    B_A = atan2(divide(POS_B(1, 2), sin(B_B)), divide(POS_B(0, 2), sin(B_B)));
    B_C = atan2(divide(POS_B(2, 1), sin(B_B)), -1*divide(POS_B(2, 0), sin(B_B)));

    C_B = atan2(sqrt(pow(POS_C(2, 0), 2) + pow(POS_C(2, 1), 2)), POS_C(2, 2));
    C_A = atan2(divide(POS_C(1, 2), sin(C_B)), divide(POS_C(0, 2), sin(C_B)));
    C_C = atan2(divide(POS_C(2, 1), sin(C_B)) , -1*divide(POS_C(2, 0), sin(C_B)));

    if(t < 0.3){

        velocity_x = (B_X - A_X) / T;
        velocity_y = (B_Y - A_Y) / T;
        velocity_z = (B_Z - A_Z) / T;
        velocity_A = (B_A - A_A) / T;
        velocity_B = (B_B - A_B) / T;
        velocity_C = (B_C - A_C) / T;

        velocity_T << cos(velocity_A)*cos(velocity_B)*cos(velocity_C) - sin(velocity_A)*sin(velocity_C),    -1*cos(velocity_A)*cos(velocity_B)*sin(velocity_C) - sin(velocity_A)*cos(velocity_C),   cos(velocity_A)*sin(velocity_B),    velocity_x,
                        sin(velocity_A)*cos(velocity_B)*cos(velocity_C) + cos(velocity_A)*sin(velocity_C),    -1*sin(velocity_A)*cos(velocity_B)*sin(velocity_C) + cos(velocity_A)*cos(velocity_C),     sin(velocity_A)*sin(velocity_B),    velocity_y,
                        -1*sin(velocity_B)*cos(velocity_C),     sin(velocity_B)*sin(velocity_C),    cos(velocity_B),    velocity_z,
                        0, 0, 0, 1;

        rotation_2_quaternion(velocity_T);

    }else if(t >= 0.3 && t < 0.7){
        t = t - 0.5;

        float h = (t + t_acc) / (trans_T);

        velocity_x = ((((C_X - B_X)*t_acc / T) + (boundary_Forward(A_X, B_X) - B_X))*(1.5 - h)*2*pow(h, 2) - ((boundary_Forward(A_X, B_X) - B_X))) / t_acc;
        velocity_y = ((((C_Y - B_Y)*t_acc / T) + (boundary_Forward(A_Y, B_Y) - B_Y))*(1.5 - h)*2*pow(h, 2) - ((boundary_Forward(A_Y, B_Y) - B_Y))) / t_acc;
        velocity_z = ((((C_Z - B_Z)*t_acc / T) + (boundary_Forward(A_Z, B_Z) - B_Z))*(1.5 - h)*2*pow(h, 2) - ((boundary_Forward(A_Z, B_Z) - B_Z))) / t_acc;
        velocity_A = ((((C_A - B_A)*t_acc / T) + (boundary_Forward(A_A, B_A) - B_A))*(1.5 - h)*2*pow(h, 2) - ((boundary_Forward(A_A, B_A) - B_A))) / t_acc;
        velocity_B = ((((C_B - B_B)*t_acc / T) + (boundary_Forward(A_B, B_B) - B_B))*(1.5 - h)*2*pow(h, 2) - ((boundary_Forward(A_B, B_B) - B_B))) / t_acc;
        velocity_C = ((((C_C - B_C)*t_acc / T) + (boundary_Forward(A_C, B_C) - B_C))*(1.5 - h)*2*pow(h, 2) - ((boundary_Forward(A_C, B_C) - B_C))) / t_acc;

        velocity_T << cos(velocity_A)*cos(velocity_B)*cos(velocity_C) - sin(velocity_A)*sin(velocity_C),    -1*cos(velocity_A)*cos(velocity_B)*sin(velocity_C) - sin(velocity_A)*cos(velocity_C),   cos(velocity_A)*sin(velocity_B),    velocity_x,
                        sin(velocity_A)*cos(velocity_B)*cos(velocity_C) + cos(velocity_A)*sin(velocity_C),    -1*sin(velocity_A)*cos(velocity_B)*sin(velocity_C) + cos(velocity_A)*cos(velocity_C),     sin(velocity_A)*sin(velocity_B),    velocity_y,
                        -1*sin(velocity_B)*cos(velocity_C),     sin(velocity_B)*sin(velocity_C),    cos(velocity_B),    velocity_z,
                        0, 0, 0, 1;

        rotation_2_quaternion(velocity_T);

        t = t + 0.5;

    }else{

        t = t - 0.5;

        float h = t / T;

        velocity_x = (C_X - B_X) / T;
        velocity_y = (C_Y - B_Y) / T;
        velocity_z = (C_Z - B_Z) / T;
        velocity_A = (C_A - B_A) / T;
        velocity_B = (C_B - B_B) / T;
        velocity_C = (C_C - B_C) / T;

        velocity_T << cos(velocity_A)*cos(velocity_B)*cos(velocity_C) - sin(velocity_A)*sin(velocity_C),    -1*cos(velocity_A)*cos(velocity_B)*sin(velocity_C) - sin(velocity_A)*cos(velocity_C),   cos(velocity_A)*sin(velocity_B),    velocity_x,
                        sin(velocity_A)*cos(velocity_B)*cos(velocity_C) + cos(velocity_A)*sin(velocity_C),    -1*sin(velocity_A)*cos(velocity_B)*sin(velocity_C) + cos(velocity_A)*cos(velocity_C),     sin(velocity_A)*sin(velocity_B),    velocity_y,
                        -1*sin(velocity_B)*cos(velocity_C),     sin(velocity_B)*sin(velocity_C),    cos(velocity_B),    velocity_z,
                        0, 0, 0, 1;

        rotation_2_quaternion(velocity_T);

        t = t + 0.5;

    }
}

void Path_Planning::cartesian_acceleration_planning(float t){
    A_X = POS_A(0, 3);
    A_Y = POS_A(1, 3);
    A_Z = POS_A(2, 3);

    B_X = POS_B(0, 3);
    B_Y = POS_B(1, 3);
    B_Z = POS_B(2, 3);

    C_X = POS_C(0, 3);
    C_Y = POS_C(1, 3);
    C_Z = POS_C(2, 3);

    A_B = atan2(sqrt(pow(POS_A(2, 0), 2) + pow(POS_A(2, 1), 2)), POS_A(2, 2));
    A_A = atan2(divide(POS_A(1, 2), sin(A_B)), divide(POS_A(0, 2), sin(A_B)));
    A_C = atan2(divide(POS_A(2, 1), sin(A_B)), -1*divide(POS_A(2, 0), sin(A_B)));

    B_B = atan2(sqrt(pow(POS_B(2, 0), 2) + pow(POS_B(2, 1), 2)), POS_B(2, 2));
    B_A = atan2(divide(POS_B(1, 2), sin(B_B)), divide(POS_B(0, 2), sin(B_B)));
    B_C = atan2(divide(POS_B(2, 1), sin(B_B)), -1*divide(POS_B(2, 0), sin(B_B)));

    C_B = atan2(sqrt(pow(POS_C(2, 0), 2) + pow(POS_C(2, 1), 2)), POS_C(2, 2));
    C_A = atan2(divide(POS_C(1, 2), sin(C_B)), divide(POS_C(0, 2), sin(C_B)));
    C_C = atan2(divide(POS_C(2, 1), sin(C_B)) , -1*divide(POS_C(2, 0), sin(C_B)));

    if(t < 0.3){

        acceleration_x = acceleration_y = acceleration_z = acceleration_A = acceleration_B = acceleration_C = 0;

        acceleration_T << cos(acceleration_A)*cos(acceleration_B)*cos(acceleration_C) - sin(acceleration_A)*sin(acceleration_C),    -1*cos(acceleration_A)*cos(acceleration_B)*sin(acceleration_C) - sin(acceleration_A)*cos(acceleration_C),   cos(acceleration_A)*sin(acceleration_B),    acceleration_x,
                        sin(acceleration_A)*cos(acceleration_B)*cos(acceleration_C) + cos(acceleration_A)*sin(acceleration_C),    -1*sin(acceleration_A)*cos(acceleration_B)*sin(acceleration_C) + cos(acceleration_A)*cos(acceleration_C),     sin(acceleration_A)*sin(acceleration_B),    acceleration_y,
                        -1*sin(acceleration_B)*cos(acceleration_C),     sin(acceleration_B)*sin(acceleration_C),    cos(acceleration_B),    acceleration_z,
                        0, 0, 0, 1;

        rotation_2_quaternion(acceleration_T);

    }else if(t >= 0.3 && t < 0.7){
        t = t - 0.5;

        float h = (t + t_acc) / (trans_T);

        acceleration_x = ((((C_X - B_X)*t_acc / T) + (boundary_Forward(A_X, B_X) - B_X))*(1 - h))*3*h / pow(t_acc, 2);
        acceleration_y = ((((C_Y - B_Y)*t_acc / T) + (boundary_Forward(A_Y, B_Y) - B_Y))*(1 - h))*3*h / pow(t_acc, 2);
        acceleration_z = ((((C_Z - B_Z)*t_acc / T) + (boundary_Forward(A_Z, B_Z) - B_Z))*(1 - h))*3*h / pow(t_acc, 2);
        acceleration_A = ((((C_A - B_A)*t_acc / T) + (boundary_Forward(A_A, B_A) - B_A))*(1 - h))*3*h / pow(t_acc, 2);
        acceleration_B = ((((C_B - B_B)*t_acc / T) + (boundary_Forward(A_B, B_B) - B_B))*(1 - h))*3*h / pow(t_acc, 2);
        acceleration_C = ((((C_C - B_C)*t_acc / T) + (boundary_Forward(A_C, B_C) - B_C))*(1 - h))*3*h / pow(t_acc, 2);
        

        acceleration_T << cos(acceleration_A)*cos(acceleration_B)*cos(acceleration_C) - sin(acceleration_A)*sin(acceleration_C),    -1*cos(acceleration_A)*cos(acceleration_B)*sin(acceleration_C) - sin(acceleration_A)*cos(acceleration_C),   cos(acceleration_A)*sin(acceleration_B),    acceleration_x,
                        sin(acceleration_A)*cos(acceleration_B)*cos(acceleration_C) + cos(acceleration_A)*sin(acceleration_C),    -1*sin(acceleration_A)*cos(acceleration_B)*sin(acceleration_C) + cos(acceleration_A)*cos(acceleration_C),     sin(acceleration_A)*sin(acceleration_B),    acceleration_y,
                        -1*sin(acceleration_B)*cos(acceleration_C),     sin(acceleration_B)*sin(acceleration_C),    cos(acceleration_B),    acceleration_z,
                        0, 0, 0, 1;

        rotation_2_quaternion(acceleration_T);

        t = t + 0.5;

    }else{

        t = t - 0.5;

        acceleration_x = acceleration_y = acceleration_z = acceleration_A = acceleration_B = acceleration_C = 0;

        acceleration_T << cos(acceleration_A)*cos(acceleration_B)*cos(acceleration_C) - sin(acceleration_A)*sin(acceleration_C),    -1*cos(acceleration_A)*cos(acceleration_B)*sin(acceleration_C) - sin(acceleration_A)*cos(acceleration_C),   cos(acceleration_A)*sin(acceleration_B),    acceleration_x,
                        sin(acceleration_A)*cos(acceleration_B)*cos(acceleration_C) + cos(acceleration_A)*sin(acceleration_C),    -1*sin(acceleration_A)*cos(acceleration_B)*sin(acceleration_C) + cos(acceleration_A)*cos(acceleration_C),     sin(acceleration_A)*sin(acceleration_B),    acceleration_y,
                        -1*sin(acceleration_B)*cos(acceleration_C),     sin(acceleration_B)*sin(acceleration_C),    cos(acceleration_B),    acceleration_z,
                        0, 0, 0, 1;

        rotation_2_quaternion(acceleration_T);

        t = t + 0.5;

    }
}

double Path_Planning::column_mul(Matrix<double, 3, 1> A, Matrix<double, 3, 1> B){
    return A(0,0)*B(0,0) + A(1, 0)*B(1, 0) + A(2, 0)*B(2, 0);
}

//this function handles for computing joint variables input and output Cartesian Point & eular angle
//this function using local variable 'x' 'y' 'z' 'a' 'A' 'B' 'C'
//this function using local double matrix 'A1' 'A2' 'A3' 'A4' 'A5' 'A6' 'T6'
//this function will calculate & print Cartesian Point
void Path_Planning::Kinematics(){
    
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
    
}

double Path_Planning::angle_normalization(double theta){
    if(abs(theta) > 180 && theta < 0){
        
        return theta + 360;

    }else if(abs(theta) > 180 && theta > 0){
        
        return theta - 360;

    }else{
        return theta;
    }
}

//this function handles inverse kinematics & calculate six JOINT VARIABLE
//this function using local variable 'double theta' & 'temp_a' 'temp_b'
//this function using global matrix 'noap_input'
//this finction will assign value to global array 'JOINT_VARIABLE_SOLUTION'
void Path_Planning::Inverse_Kinematics(Matrix<double, 4, 4> noap_input){
     double theta_1_1 , theta_1_2, 
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
        JOINT_VARIABLE_SOLUTION_5[0] = angle_normalization(JOINT_VARIABLE_SOLUTION_5[0]);
        JOINT_VARIABLE_SOLUTION_6[0] = (theta_1_1*180/PI);
        JOINT_VARIABLE_SOLUTION_6[0] = angle_normalization(JOINT_VARIABLE_SOLUTION_6[0]);
        JOINT_VARIABLE_SOLUTION_7[0] = (theta_1_1*180/PI);
        JOINT_VARIABLE_SOLUTION_7[0] = angle_normalization(JOINT_VARIABLE_SOLUTION_7[0]);
        JOINT_VARIABLE_SOLUTION_8[0] = (theta_1_1*180/PI);
        JOINT_VARIABLE_SOLUTION_8[0] = angle_normalization(JOINT_VARIABLE_SOLUTION_8[0]);

    //θ1-2 (-74)
    theta_1_2 = (atan2(noap_input(1, 3), noap_input(0, 3)) - atan2(d2, -1*sqrt(pow(noap_input(0, 3), 2) + pow(noap_input(1, 3), 2) - pow(d2, 2))));
        JOINT_VARIABLE_SOLUTION_1[0] = (theta_1_2*180/PI);
        JOINT_VARIABLE_SOLUTION_1[0] = angle_normalization(JOINT_VARIABLE_SOLUTION_1[0]);
        JOINT_VARIABLE_SOLUTION_2[0] = (theta_1_2*180/PI);
        JOINT_VARIABLE_SOLUTION_2[0] = angle_normalization(JOINT_VARIABLE_SOLUTION_2[0]);
        JOINT_VARIABLE_SOLUTION_3[0] = (theta_1_2*180/PI);
        JOINT_VARIABLE_SOLUTION_3[0] = angle_normalization(JOINT_VARIABLE_SOLUTION_3[0]);
        JOINT_VARIABLE_SOLUTION_4[0] = (theta_1_2*180/PI);
        JOINT_VARIABLE_SOLUTION_4[0] = angle_normalization(JOINT_VARIABLE_SOLUTION_4[0]);

    //θ2 -> four solution
    //θ2-1 (20)
    theta_2_1 = atan2((cos(theta_1_1)*noap_input(0, 3)) + (sin(theta_1_1)*noap_input(1, 3)), noap_input(2, 3));
        JOINT_VARIABLE_SOLUTION_5[1] = (theta_2_1*180/PI);
        JOINT_VARIABLE_SOLUTION_5[1] = angle_normalization(JOINT_VARIABLE_SOLUTION_5[1]);
        JOINT_VARIABLE_SOLUTION_6[1] = (theta_2_1*180/PI);
        JOINT_VARIABLE_SOLUTION_6[1] = angle_normalization(JOINT_VARIABLE_SOLUTION_6[1]);

    //θ2-2 (160)
    theta_2_2 = atan2((cos(theta_1_1)*noap_input(0, 3)) + (sin(theta_1_1)*noap_input(1, 3)), -1*noap_input(2, 3));
        JOINT_VARIABLE_SOLUTION_3[1] = (theta_2_2*180/PI);
        JOINT_VARIABLE_SOLUTION_3[1] = angle_normalization(JOINT_VARIABLE_SOLUTION_3[1]);
        JOINT_VARIABLE_SOLUTION_4[1] = (theta_2_2*180/PI);
        JOINT_VARIABLE_SOLUTION_4[1] = angle_normalization(JOINT_VARIABLE_SOLUTION_4[1]);

    //θ2-3 (-20)
    theta_2_3 = atan2((cos(theta_1_2)*noap_input(0, 3)) + (sin(theta_1_2)*noap_input(1, 3)), noap_input(2, 3));
        JOINT_VARIABLE_SOLUTION_1[1] = (theta_2_3*180/PI);
        JOINT_VARIABLE_SOLUTION_1[1] = angle_normalization(JOINT_VARIABLE_SOLUTION_1[1]);
        JOINT_VARIABLE_SOLUTION_2[1] = (theta_2_3*180/PI);
        JOINT_VARIABLE_SOLUTION_2[1] = angle_normalization(JOINT_VARIABLE_SOLUTION_2[1]);

    //θ2-4 (-160)
    theta_2_4 = atan2((cos(theta_1_2)*noap_input(0, 3)) + (sin(theta_1_2)*noap_input(1, 3)), -1*noap_input(2, 3));
        JOINT_VARIABLE_SOLUTION_7[1] = (theta_2_4*180/PI);
        JOINT_VARIABLE_SOLUTION_7[1] = angle_normalization(JOINT_VARIABLE_SOLUTION_7[1]);
        JOINT_VARIABLE_SOLUTION_8[1] = (theta_2_4*180/PI);
        JOINT_VARIABLE_SOLUTION_8[1] = angle_normalization(JOINT_VARIABLE_SOLUTION_8[1]);

    //d3 -> two solution
    //d_3_1 (20)
    d_3_1 = noap_input(2, 3) / cos(theta_2_1);
        JOINT_VARIABLE_SOLUTION_1[2] = d_3_1;
        JOINT_VARIABLE_SOLUTION_1[2] = angle_normalization(JOINT_VARIABLE_SOLUTION_1[2]);
        JOINT_VARIABLE_SOLUTION_2[2] = d_3_1;
        JOINT_VARIABLE_SOLUTION_2[2] = angle_normalization(JOINT_VARIABLE_SOLUTION_2[2]);
        JOINT_VARIABLE_SOLUTION_5[2] = d_3_1;
        JOINT_VARIABLE_SOLUTION_5[2] = angle_normalization(JOINT_VARIABLE_SOLUTION_5[2]);
        JOINT_VARIABLE_SOLUTION_6[2] = d_3_1;
        JOINT_VARIABLE_SOLUTION_6[2] = angle_normalization(JOINT_VARIABLE_SOLUTION_6[2]);

    //d_3_2 (-20)
    d_3_2 = noap_input(2, 3) / cos(theta_2_2);
        JOINT_VARIABLE_SOLUTION_3[2] = d_3_2;
        JOINT_VARIABLE_SOLUTION_3[2] = angle_normalization(JOINT_VARIABLE_SOLUTION_3[2]);
        JOINT_VARIABLE_SOLUTION_4[2] = d_3_2;
        JOINT_VARIABLE_SOLUTION_4[2] = angle_normalization(JOINT_VARIABLE_SOLUTION_4[2]);
        JOINT_VARIABLE_SOLUTION_7[2] = d_3_2;
        JOINT_VARIABLE_SOLUTION_7[2] = angle_normalization(JOINT_VARIABLE_SOLUTION_7[2]);
        JOINT_VARIABLE_SOLUTION_8[2] = d_3_2;
        JOINT_VARIABLE_SOLUTION_8[2] = angle_normalization(JOINT_VARIABLE_SOLUTION_8[2]);

    //θ4 -> eight solution
    //θ4-1 (79)
    theta_4_1_1 = atan2((-1*sin(theta_1_2)*noap_input(0, 2)) + (cos(theta_1_2)*noap_input(1, 2)), (cos(theta_1_2)*cos(theta_2_3)*noap_input(0, 2)) + (sin(theta_1_2)*cos(theta_2_3)* noap_input(1, 2)) - (sin(theta_2_3)*noap_input(2, 2)));
        JOINT_VARIABLE_SOLUTION_1[3] = theta_4_1_1*180/PI;
        JOINT_VARIABLE_SOLUTION_1[3] = angle_normalization(JOINT_VARIABLE_SOLUTION_1[3]);

    //θ4-1 - 180 (-100)
    theta_4_1_2 = ((theta_4_1_1*180/PI) - 180)*PI/180;
        JOINT_VARIABLE_SOLUTION_2[3] = theta_4_1_2*180/PI;
        JOINT_VARIABLE_SOLUTION_2[3] = angle_normalization(JOINT_VARIABLE_SOLUTION_2[3]);

    //θ4-2(100)
    theta_4_2_1 = atan2((-1*sin(theta_1_2)*noap_input(0, 2)) + (cos(theta_1_2)*noap_input(1, 2)), (cos(theta_1_2)*cos(theta_2_2)*noap_input(0, 2)) + (sin(theta_1_2)*cos(theta_2_2)* noap_input(1, 2)) - (sin(theta_2_2)*noap_input(2, 2)));
        JOINT_VARIABLE_SOLUTION_3[3] = theta_4_2_1*180/PI;
        JOINT_VARIABLE_SOLUTION_3[3] = angle_normalization(JOINT_VARIABLE_SOLUTION_3[3]);
    
    //θ4-2 - 180  (-79)
    theta_4_2_2 = ((theta_4_2_1*180/PI) - 180)*PI/180;
        JOINT_VARIABLE_SOLUTION_4[3] = theta_4_2_2*180/PI;
        JOINT_VARIABLE_SOLUTION_4[3] = angle_normalization(JOINT_VARIABLE_SOLUTION_4[3]);
    
    //θ4-3 (20)
    theta_4_3_1 = atan2((-1*sin(theta_1_1)*noap_input(0, 2)) + (cos(theta_1_1)*noap_input(1, 2)), (cos(theta_1_1)*cos(theta_2_1)*noap_input(0, 2)) + (sin(theta_1_1)*cos(theta_2_1)* noap_input(1, 2)) - (sin(theta_2_1)*noap_input(2, 2)));
        JOINT_VARIABLE_SOLUTION_5[3] = theta_4_3_1*180/PI;
        JOINT_VARIABLE_SOLUTION_5[3] = angle_normalization(JOINT_VARIABLE_SOLUTION_5[3]);

    //θ4-3 - 180 (-160)
    theta_4_3_2 = ((theta_4_3_1*180/PI) - 180)*PI/180;
        JOINT_VARIABLE_SOLUTION_6[3] = theta_4_3_2*180/PI;
        JOINT_VARIABLE_SOLUTION_6[3] = angle_normalization(JOINT_VARIABLE_SOLUTION_6[3]);

    //θ4-4 (160)
    theta_4_4_1 = atan2((-1*sin(theta_1_1)*noap_input(0, 2)) + (cos(theta_1_1)*noap_input(1, 2)), (cos(theta_1_1)*cos(theta_2_4)*noap_input(0, 2)) + (sin(theta_1_1)*cos(theta_2_4)* noap_input(1, 2)) - (sin(theta_2_4)*noap_input(2, 2)));
        JOINT_VARIABLE_SOLUTION_7[3] = theta_4_4_1*180/PI;
        JOINT_VARIABLE_SOLUTION_7[3] = angle_normalization(JOINT_VARIABLE_SOLUTION_7[3]);

    //θ4-4 - 180 (-20)
    theta_4_4_2 = ((theta_4_4_1*180/PI) - 180)*PI/180;
        JOINT_VARIABLE_SOLUTION_8[3] = theta_4_4_2*180/PI;
        JOINT_VARIABLE_SOLUTION_8[3] = angle_normalization(JOINT_VARIABLE_SOLUTION_8[3]);

    //θ5 -> eight solution
    //θ5-1 
    temp_a = ((cos(theta_1_2)*cos(theta_2_3)*cos(theta_4_1_1) - sin(theta_1_2)*sin(theta_4_1_1))*noap_input(0 ,2)) + ((sin(theta_1_2)*cos(theta_2_3)*cos(theta_4_1_1) + cos(theta_1_2)*sin(theta_4_1_1))*noap_input(1, 2)) + ((-1*sin(theta_2_3)*cos(theta_4_1_1))*noap_input(2, 2));
    temp_b = (cos(theta_1_2)*sin(theta_2_3)*noap_input(0, 2)) + (sin(theta_1_2)*sin(theta_2_3)*noap_input(1, 2)) + (cos(theta_2_3)*noap_input(2, 2));
    theta_5_1_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_1[4] = theta_5_1_1*180/PI;
        JOINT_VARIABLE_SOLUTION_1[4] = angle_normalization(JOINT_VARIABLE_SOLUTION_1[4]);

    //θ5-1-2
    temp_a = ((cos(theta_1_2)*cos(theta_2_3)*cos(theta_4_1_2) - sin(theta_1_2)*sin(theta_4_1_2))*noap_input(0 ,2)) + ((sin(theta_1_2)*cos(theta_2_3)*cos(theta_4_1_2) + cos(theta_1_2)*sin(theta_4_1_2))*noap_input(1, 2)) + ((-1*sin(theta_2_3)*cos(theta_4_1_2))*noap_input(2, 2));
    temp_b = (cos(theta_1_2)*sin(theta_2_3)*noap_input(0, 2)) + (sin(theta_1_2)*sin(theta_2_3)*noap_input(1, 2)) + (cos(theta_2_3)*noap_input(2, 2));
    theta_5_1_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_2[4] = theta_5_1_2*180/PI;
        JOINT_VARIABLE_SOLUTION_2[4] = angle_normalization(JOINT_VARIABLE_SOLUTION_2[4]);

    //θ5-2-1
    temp_a = ((cos(theta_1_2)*cos(theta_2_2)*cos(theta_4_2_1) - sin(theta_1_2)*sin(theta_4_2_1))*noap_input(0 ,2)) + ((sin(theta_1_2)*cos(theta_2_2)*cos(theta_4_2_1) + cos(theta_1_2)*sin(theta_4_2_1))*noap_input(1, 2)) + ((-1*sin(theta_2_2)*cos(theta_4_2_1))*noap_input(2, 2));
    temp_b = (cos(theta_1_2)*sin(theta_2_2)*noap_input(0, 2)) + (sin(theta_1_2)*sin(theta_2_2)*noap_input(1, 2)) + (cos(theta_2_2)*noap_input(2, 2));
    theta_5_2_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_3[4] = theta_5_2_1*180/PI;
        JOINT_VARIABLE_SOLUTION_3[4] = angle_normalization(JOINT_VARIABLE_SOLUTION_3[4]);

    //θ5-2-2
    temp_a = ((cos(theta_1_2)*cos(theta_2_2)*cos(theta_4_2_2) - sin(theta_1_2)*sin(theta_4_2_2))*noap_input(0 ,2)) + ((sin(theta_1_2)*cos(theta_2_2)*cos(theta_4_2_2) + cos(theta_1_2)*sin(theta_4_2_2))*noap_input(1, 2)) + ((-1*sin(theta_2_2)*cos(theta_4_2_2))*noap_input(2, 2));
    temp_b = (cos(theta_1_2)*sin(theta_2_2)*noap_input(0, 2)) + (sin(theta_1_2)*sin(theta_2_2)*noap_input(1, 2)) + (cos(theta_2_2)*noap_input(2, 2));
    theta_5_2_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_4[4] = theta_5_2_2*180/PI;
        JOINT_VARIABLE_SOLUTION_4[4] = angle_normalization(JOINT_VARIABLE_SOLUTION_4[4]);

    //θ5-3-1
    temp_a = ((cos(theta_1_1)*cos(theta_2_1)*cos(theta_4_3_1) - sin(theta_1_1)*sin(theta_4_3_1))*noap_input(0 ,2)) + ((sin(theta_1_1)*cos(theta_2_1)*cos(theta_4_3_1) + cos(theta_1_1)*sin(theta_4_3_1))*noap_input(1, 2)) + ((-1*sin(theta_2_1)*cos(theta_4_3_1))*noap_input(2, 2));
    temp_b = (cos(theta_1_1)*sin(theta_2_1)*noap_input(0, 2)) + (sin(theta_1_1)*sin(theta_2_1)*noap_input(1, 2)) + (cos(theta_2_1)*noap_input(2, 2));
    theta_5_3_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_5[4] = theta_5_3_1*180/PI;
        JOINT_VARIABLE_SOLUTION_5[4] = angle_normalization(JOINT_VARIABLE_SOLUTION_5[4]);

    //θ5-3-2
    temp_a = ((cos(theta_1_1)*cos(theta_2_1)*cos(theta_4_3_2) - sin(theta_1_1)*sin(theta_4_3_2))*noap_input(0 ,2)) + ((sin(theta_1_1)*cos(theta_2_1)*cos(theta_4_3_2) + cos(theta_1_1)*sin(theta_4_3_2))*noap_input(1, 2)) + ((-1*sin(theta_2_1)*cos(theta_4_3_2))*noap_input(2, 2));
    temp_b = (cos(theta_1_1)*sin(theta_2_1)*noap_input(0, 2)) + (sin(theta_1_1)*sin(theta_2_1)*noap_input(1, 2)) + (cos(theta_2_1)*noap_input(2, 2));
    theta_5_3_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_6[4] = theta_5_3_2*180/PI;
        JOINT_VARIABLE_SOLUTION_6[4] = angle_normalization(JOINT_VARIABLE_SOLUTION_6[4]);


    //θ5-4-1
    temp_a = ((cos(theta_1_1)*cos(theta_2_4)*cos(theta_4_4_1) - sin(theta_1_1)*sin(theta_4_4_1))*noap_input(0 ,2)) + ((sin(theta_1_1)*cos(theta_2_4)*cos(theta_4_4_1) + cos(theta_1_1)*sin(theta_4_4_1))*noap_input(1, 2)) + ((-1*sin(theta_2_4)*cos(theta_4_4_1))*noap_input(2, 2));
    temp_b = (cos(theta_1_1)*sin(theta_2_4)*noap_input(0, 2)) + (sin(theta_1_1)*sin(theta_2_4)*noap_input(1, 2)) + (cos(theta_2_4)*noap_input(2, 2));
    theta_5_4_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_7[4] = theta_5_4_1*180/PI;
        JOINT_VARIABLE_SOLUTION_7[4] = angle_normalization(JOINT_VARIABLE_SOLUTION_7[4]);

    //θ5-4-2
    temp_a = ((cos(theta_1_1)*cos(theta_2_4)*cos(theta_4_4_2) - sin(theta_1_1)*sin(theta_4_4_2))*noap_input(0 ,2)) + ((sin(theta_1_1)*cos(theta_2_4)*cos(theta_4_4_2) + cos(theta_1_1)*sin(theta_4_4_2))*noap_input(1, 2)) + ((-1*sin(theta_2_4)*cos(theta_4_4_2))*noap_input(2, 2));
    temp_b = (cos(theta_1_1)*sin(theta_2_4)*noap_input(0, 2)) + (sin(theta_1_1)*sin(theta_2_4)*noap_input(1, 2)) + (cos(theta_2_4)*noap_input(2, 2));
    theta_5_4_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_8[4] = theta_5_4_2*180/PI;
        JOINT_VARIABLE_SOLUTION_8[4] = angle_normalization(JOINT_VARIABLE_SOLUTION_8[4]);

    //θ6 -> eight solution
    //θ6-1-1
    temp_fr = -1*((cos(theta_1_2)*cos(theta_2_3)*sin(theta_4_1_1)) + (sin(theta_1_2)*cos(theta_4_1_1)));
    temp_bk = -1*((sin(theta_1_2)*cos(theta_2_3)*sin(theta_4_1_1)) - (cos(theta_1_2)*cos(theta_4_1_1)));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_3)*sin(theta_4_1_1))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_3)*sin(theta_4_1_1))*noap_input(2, 1);
    theta_6_1_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_1[5] = theta_6_1_1*180/PI;
        JOINT_VARIABLE_SOLUTION_1[5] = angle_normalization(JOINT_VARIABLE_SOLUTION_1[5]);

    //θ6-1-2
    temp_fr = (-1*(cos(theta_1_2)*cos(theta_2_3)*sin(theta_4_1_2))) - (sin(theta_1_2)*cos(theta_4_1_2));
    temp_bk = (-1*(sin(theta_1_2)*cos(theta_2_3)*sin(theta_4_1_2))) + (cos(theta_1_2)*cos(theta_4_1_2));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_3)*sin(theta_4_1_2))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_3)*sin(theta_4_1_2))*noap_input(2, 1);
    theta_6_1_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_2[5] = theta_6_1_2*180/PI;
        JOINT_VARIABLE_SOLUTION_2[5] = angle_normalization(JOINT_VARIABLE_SOLUTION_2[5]);

    //θ6-2-1
    temp_fr = (-1*(cos(theta_1_2)*cos(theta_2_2)*sin(theta_4_2_1))) - (sin(theta_1_2)*cos(theta_4_2_1));
    temp_bk = (-1*(sin(theta_1_2)*cos(theta_2_2)*sin(theta_4_2_1))) + (cos(theta_1_2)*cos(theta_4_2_1));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_2)*sin(theta_4_2_1))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_2)*sin(theta_4_2_1))*noap_input(2, 1);
    theta_6_2_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_3[5] = theta_6_2_1*180/PI;
        JOINT_VARIABLE_SOLUTION_3[5] = angle_normalization(JOINT_VARIABLE_SOLUTION_3[5]);

    //θ6-2-2
    temp_fr = (-1*(cos(theta_1_2)*cos(theta_2_2)*sin(theta_4_2_2))) - (sin(theta_1_2)*cos(theta_4_2_2));
    temp_bk = (-1*(sin(theta_1_2)*cos(theta_2_2)*sin(theta_4_2_2))) + (cos(theta_1_2)*cos(theta_4_2_2));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_2)*sin(theta_4_2_2))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_2)*sin(theta_4_2_2))*noap_input(2, 1);
    theta_6_2_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_4[5] = theta_6_2_2*180/PI;
        JOINT_VARIABLE_SOLUTION_4[5] = angle_normalization(JOINT_VARIABLE_SOLUTION_4[5]);

    //θ6-3-1
    temp_fr = (-1*(cos(theta_1_1)*cos(theta_2_1)*sin(theta_4_3_1))) - (sin(theta_1_1)*cos(theta_4_3_1));
    temp_bk = (-1*(sin(theta_1_1)*cos(theta_2_1)*sin(theta_4_3_1))) + (cos(theta_1_1)*cos(theta_4_3_1));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_1)*sin(theta_4_3_1))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_1)*sin(theta_4_3_1))*noap_input(2, 1);
    theta_6_3_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_5[5] = theta_6_3_1*180/PI;
        JOINT_VARIABLE_SOLUTION_5[5] = angle_normalization(JOINT_VARIABLE_SOLUTION_5[5]);

    //θ6-3-2
    temp_fr = (-1*(cos(theta_1_1)*cos(theta_2_1)*sin(theta_4_3_2))) - (sin(theta_1_1)*cos(theta_4_3_2));
    temp_bk = (-1*(sin(theta_1_1)*cos(theta_2_1)*sin(theta_4_3_2))) + (cos(theta_1_1)*cos(theta_4_3_2));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_1)*sin(theta_4_3_2))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_1)*sin(theta_4_3_2))*noap_input(2, 1);
    theta_6_3_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_6[5] = theta_6_3_2*180/PI;
        JOINT_VARIABLE_SOLUTION_6[5] = angle_normalization(JOINT_VARIABLE_SOLUTION_6[5]);

    //θ6-4-1
    temp_fr = (-1*(cos(theta_1_1)*cos(theta_2_4)*sin(theta_4_4_1))) - (sin(theta_1_1)*cos(theta_4_4_1));
    temp_bk = (-1*(sin(theta_1_1)*cos(theta_2_4)*sin(theta_4_4_1))) + (cos(theta_1_1)*cos(theta_4_4_1));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_4)*sin(theta_4_4_1))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_4)*sin(theta_4_4_1))*noap_input(2, 1);
    theta_6_4_1 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_7[5] = theta_6_4_1*180/PI;
        JOINT_VARIABLE_SOLUTION_7[5] = angle_normalization(JOINT_VARIABLE_SOLUTION_7[5]);

    //θ6-4-1
    temp_fr = (-1*(cos(theta_1_1)*cos(theta_2_4)*sin(theta_4_4_2))) - (sin(theta_1_1)*cos(theta_4_4_2));
    temp_bk = (-1*(sin(theta_1_1)*cos(theta_2_4)*sin(theta_4_4_2))) + (cos(theta_1_1)*cos(theta_4_4_2));
    temp_a = (temp_fr)*noap_input(0, 0) + (temp_bk)*noap_input(1, 0) + (sin(theta_2_4)*sin(theta_4_4_2))*noap_input(2, 0);
    temp_b = (temp_fr)*noap_input(0, 1) + (temp_bk)*noap_input(1, 1) + (sin(theta_2_4)*sin(theta_4_4_2))*noap_input(2, 1);
    theta_6_4_2 = atan2(temp_a, temp_b);
        JOINT_VARIABLE_SOLUTION_8[5] = theta_6_4_2*180/PI; 
        JOINT_VARIABLE_SOLUTION_8[5] = angle_normalization(JOINT_VARIABLE_SOLUTION_8[5]);

}

//this function handles for showing the answer calculated by inverse kinematics
//this function using global matrix & determine whether the angle limit is met
bool Path_Planning::output_check(double JOINT_VARIABLE_SOLUTION[6]){

    return false;

    if(JOINT_VARIABLE_SOLUTION[0] >= 160 || JOINT_VARIABLE_SOLUTION[0] <= -160){
        return true;

    }

    if(JOINT_VARIABLE_SOLUTION[1] >= 125 || JOINT_VARIABLE_SOLUTION[1] <= -125){
        return true;

    }

    if(JOINT_VARIABLE_SOLUTION[2] >= 30 || JOINT_VARIABLE_SOLUTION[2] <= -30){
        return true;

    }

    if(JOINT_VARIABLE_SOLUTION[3] >= 140 || JOINT_VARIABLE_SOLUTION[3] <= -140){
        return true;

    }

    if(JOINT_VARIABLE_SOLUTION[4] >= 100 || JOINT_VARIABLE_SOLUTION[4] <= -100){
        return true;

    }

    if(JOINT_VARIABLE_SOLUTION[5] >= 260 || JOINT_VARIABLE_SOLUTION[0] <= -260){
        return true;
        
    }
        
}

void Path_Planning::find_ok_pos(){

    Inverse_Kinematics(POS_A);

    if(output_check(JOINT_VARIABLE_SOLUTION_1) == true){
        for(int i = 0; i < 6; i++){
            POS_A_OK[i] = JOINT_VARIABLE_SOLUTION_1[i];
        }

        cout<<"1"<<endl;
    }else if(output_check(JOINT_VARIABLE_SOLUTION_2) == false){
        for(int i = 0; i < 6; i++){
            POS_A_OK[i] = JOINT_VARIABLE_SOLUTION_2[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_3) == false){
        for(int i = 0; i < 6; i++){
            POS_A_OK[i] = JOINT_VARIABLE_SOLUTION_3[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_4) == false){
        for(int i = 0; i < 6; i++){
            POS_A_OK[i] = JOINT_VARIABLE_SOLUTION_4[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_5) == false){
        for(int i = 0; i < 6; i++){
            POS_A_OK[i] = JOINT_VARIABLE_SOLUTION_5[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_6) == false){
        for(int i = 0; i < 6; i++){
            POS_A_OK[i] = JOINT_VARIABLE_SOLUTION_6[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_7) == false){
        for(int i = 0; i < 6; i++){
            POS_A_OK[i] = JOINT_VARIABLE_SOLUTION_7[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_8) == false){
        for(int i = 0; i < 6; i++){
            POS_A_OK[i] = JOINT_VARIABLE_SOLUTION_8[i];
        }
    }

    

    Inverse_Kinematics(POS_B);

    if(output_check(JOINT_VARIABLE_SOLUTION_1) == false){
        for(int i = 0; i < 6; i++){
            POS_B_OK[i] = JOINT_VARIABLE_SOLUTION_1[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_2) == false){
        for(int i = 0; i < 6; i++){
            POS_B_OK[i] = JOINT_VARIABLE_SOLUTION_2[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_3) == false){
        for(int i = 0; i < 6; i++){
            POS_B_OK[i] = JOINT_VARIABLE_SOLUTION_3[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_4) == false){
        for(int i = 0; i < 6; i++){
            POS_B_OK[i] = JOINT_VARIABLE_SOLUTION_4[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_5) == false){
        for(int i = 0; i < 6; i++){
            POS_B_OK[i] = JOINT_VARIABLE_SOLUTION_5[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_6) == false){
        for(int i = 0; i < 6; i++){
            POS_B_OK[i] = JOINT_VARIABLE_SOLUTION_6[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_7) == false){
        for(int i = 0; i < 6; i++){
            POS_B_OK[i] = JOINT_VARIABLE_SOLUTION_7[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_8) == false){
        for(int i = 0; i < 6; i++){
            POS_B_OK[i] = JOINT_VARIABLE_SOLUTION_8[i];
        }
    }

    Inverse_Kinematics(POS_C);

    if(output_check(JOINT_VARIABLE_SOLUTION_1) == false){
        for(int i = 0; i < 6; i++){
            POS_C_OK[i] = JOINT_VARIABLE_SOLUTION_1[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_2) == false){
        for(int i = 0; i < 6; i++){
            POS_C_OK[i] = JOINT_VARIABLE_SOLUTION_2[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_3) == false){
        for(int i = 0; i < 6; i++){
            POS_C_OK[i] = JOINT_VARIABLE_SOLUTION_3[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_4) == false){
        for(int i = 0; i < 6; i++){
            POS_C_OK[i] = JOINT_VARIABLE_SOLUTION_4[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_5) == false){
        for(int i = 0; i < 6; i++){
            POS_C_OK[i] = JOINT_VARIABLE_SOLUTION_5[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_6) == false){
        for(int i = 0; i < 6; i++){
            POS_C_OK[i] = JOINT_VARIABLE_SOLUTION_6[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_7) == false){
        for(int i = 0; i < 6; i++){
            POS_C_OK[i] = JOINT_VARIABLE_SOLUTION_7[i];
        }
    }else if(output_check(JOINT_VARIABLE_SOLUTION_8) == false){
        for(int i = 0; i < 6; i++){
            POS_C_OK[i] = JOINT_VARIABLE_SOLUTION_8[i];
        }
    }

}

void Path_Planning::joint_move_angle(float t){  

    //find_ok_pos();
    

    if(t < 0.3){
        
        float h = t / T;

        j_1_position = (POS_B_OK[0] - POS_A_OK[0])*h + POS_A_OK[0];
        j_2_position = (POS_B_OK[1] - POS_A_OK[1])*h + POS_A_OK[1];
        j_3_position = (POS_B_OK[2] - POS_A_OK[2])*h + POS_A_OK[2];
        j_4_position = (POS_B_OK[3] - POS_A_OK[3])*h + POS_A_OK[3];
        j_5_position = (POS_B_OK[4] - POS_A_OK[4])*h + POS_A_OK[4];
        j_6_position = (POS_B_OK[5] - POS_A_OK[5])*h + POS_A_OK[5];


    }else if(t >= 0.3 && t < 0.7){
        t = t - 0.5;

        float h = (t + t_acc) / (trans_T);

        j_1_position = ((((POS_C_OK[0] - POS_B_OK[0])*t_acc / T) + (boundary_Forward(POS_A_OK[0], POS_B_OK[0]) - POS_B_OK[0]))*(2 - h)*pow(h, 2) - 2*((boundary_Forward(POS_A_OK[0], POS_B_OK[0]) - POS_B_OK[0])))*h + POS_B_OK[0] + (boundary_Forward(POS_A_OK[0], POS_B_OK[0]) - POS_B_OK[0]);
        j_2_position = ((((POS_C_OK[1] - POS_B_OK[1])*t_acc / T) + (boundary_Forward(POS_A_OK[1], POS_B_OK[1]) - POS_B_OK[1]))*(2 - h)*pow(h, 2) - 2*((boundary_Forward(POS_A_OK[1], POS_B_OK[1]) - POS_B_OK[1])))*h + POS_B_OK[1] + (boundary_Forward(POS_A_OK[1], POS_B_OK[1]) - POS_B_OK[1]);
        j_3_position = ((((POS_C_OK[2] - POS_B_OK[2])*t_acc / T) + (boundary_Forward(POS_A_OK[2], POS_B_OK[2]) - POS_B_OK[2]))*(2 - h)*pow(h, 2) - 2*((boundary_Forward(POS_A_OK[2], POS_B_OK[2]) - POS_B_OK[2])))*h + POS_B_OK[2] + (boundary_Forward(POS_A_OK[2], POS_B_OK[2]) - POS_B_OK[2]);
        j_4_position = ((((POS_C_OK[3] - POS_B_OK[3])*t_acc / T) + (boundary_Forward(POS_A_OK[3], POS_B_OK[3]) - POS_B_OK[3]))*(2 - h)*pow(h, 2) - 2*((boundary_Forward(POS_A_OK[3], POS_B_OK[3]) - POS_B_OK[3])))*h + POS_B_OK[3] + (boundary_Forward(POS_A_OK[3], POS_B_OK[3]) - POS_B_OK[3]);
        j_5_position = ((((POS_C_OK[4] - POS_B_OK[4])*t_acc / T) + (boundary_Forward(POS_A_OK[4], POS_B_OK[4]) - POS_B_OK[4]))*(2 - h)*pow(h, 2) - 2*((boundary_Forward(POS_A_OK[4], POS_B_OK[4]) - POS_B_OK[4])))*h + POS_B_OK[4] + (boundary_Forward(POS_A_OK[4], POS_B_OK[4]) - POS_B_OK[4]);
        j_6_position = ((((POS_C_OK[5] - POS_B_OK[5])*t_acc / T) + (boundary_Forward(POS_A_OK[5], POS_B_OK[5]) - POS_B_OK[5]))*(2 - h)*pow(h, 2) - 2*((boundary_Forward(POS_A_OK[5], POS_B_OK[5]) - POS_B_OK[5])))*h + POS_B_OK[5] + (boundary_Forward(POS_A_OK[5], POS_B_OK[5]) - POS_B_OK[5]);

        t = t + 0.5;

    }else{

        t = t - 0.5;

        float h = t / T;

        j_1_position = (POS_C_OK[0] - POS_B_OK[0])*h + POS_B_OK[0];
        j_2_position = (POS_C_OK[1] - POS_B_OK[1])*h + POS_B_OK[1];
        j_3_position = (POS_C_OK[2] - POS_B_OK[2])*h + POS_B_OK[2];
        j_4_position = (POS_C_OK[3] - POS_B_OK[3])*h + POS_B_OK[3];
        j_5_position = (POS_C_OK[4] - POS_B_OK[4])*h + POS_B_OK[4];
        j_6_position = (POS_C_OK[5] - POS_B_OK[5])*h + POS_B_OK[5];

        t = t + 0.5;

    }

}

int main(int argc, char **argv){

    char format;
    int joint;

    ros::init(argc, argv,"path_planning");
	ros::NodeHandle n;
    ros::Publisher cartesian_visualization_pub = n.advertise<visualization_msgs::Marker>("cartesian_visualization_marker", 100);
    ros::Publisher cartesian_visualization_pub_pose = n.advertise<geometry_msgs::PoseStamped>("pose", 100);
    ros::Publisher joint_pub = n.advertise<std_msgs::Float32>("joint", 100);

    ros::Publisher cartesian_position_pub = n.advertise<geometry_msgs::Point>("position", 100);
    ros::Publisher cartesian_velocity_pub = n.advertise<geometry_msgs::Point>("velocity", 100);
    ros::Publisher cartesian_acceleration_pub = n.advertise<geometry_msgs::Point>("acceleration", 100);

    Path_Planning P;

    P.set();

    //initialization
    visualization_msgs::Marker points, line_strip, line_list, line_arrow;
    points.header.frame_id = line_strip.header.frame_id = line_arrow.header.frame_id = "my_frame";
    points.header.stamp = line_strip.header.stamp = line_arrow.header.stamp = ros::Time::now();
    points.ns = line_strip.ns  = "points_and_lines";
    line_arrow.ns = "points_arrow";

    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
    line_arrow.id = 3;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_arrow.type = visualization_msgs::Marker::ARROW;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.5;
    points.scale.y = 0.5;

    line_arrow.scale.x = 5;
    line_arrow.scale.y = 1;
    line_arrow.scale.z = 1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Arrow list is red
    line_arrow.color.r = 1.0;
    line_arrow.color.a = 1.0;
    
    //line_arrow.lifetime = ros::Duration();

    ros::Rate r(10);

    cout<< "[ * ]Please choose a output format "<< endl;
    cout<< " a = cartesian move position  /  b = cartesian move velocity      /  c = cartesian move acceleration "<< endl;
    cout<< " d = joint move angle         /  e = joint move angular velocity  /  f = joint move acceleration "<<endl;
    cout<<"g = cartesian move rviz visualization  /  h = joint move rviz visualization "<< endl;
    cin >> format;
    
    if(format == 'd' || format == 'e' || format == 'f'){
        cout<<"which joint : ";
        cin>> joint ;

    }
    
    for(float t = 0; t <= 1; t = t + 0.02){

        //z-axis
        /*
        points.pose.orientation.x = line_strip.pose.orientation.x = line_list.pose.orientation.x = P.quaternion.x();
        points.pose.orientation.y = line_strip.pose.orientation.y = line_list.pose.orientation.y = P.quaternion.y();
        points.pose.orientation.z = line_strip.pose.orientation.z = line_list.pose.orientation.z = P.quaternion.z();
        */

        geometry_msgs::Point point;
        geometry_msgs::Pose pose;
        std_msgs::Float32 j;

        if(format == 'a'){

            P.cartesian_position_planning(t);

            pose.position.x = P.position_x;
            pose.position.y = P.position_y;
            pose.position.z = P.position_z;
            pose.orientation.x = P.quaternion.x();
            pose.orientation.y = P.quaternion.y();
            pose.orientation.z = P.quaternion.z();
            pose.orientation.w = P.quaternion.w();

            cartesian_position_pub.publish(pose);

        }else if(format == 'b'){

            P.cartesian_velocity_planning(t);

            pose.position.x = P.velocity_x;
            pose.position.y = P.velocity_y;
            pose.position.z = P.velocity_z;
            pose.orientation.x = P.quaternion.x();
            pose.orientation.y = P.quaternion.y();
            pose.orientation.z = P.quaternion.z();
            pose.orientation.w = P.quaternion.w();

            cartesian_velocity_pub.publish(pose);

        }else if(format == 'c'){

            P.cartesian_acceleration_planning(t);

            pose.position.x = P.acceleration_x;
            pose.position.y = P.acceleration_y;
            pose.position.z = P.acceleration_z;
            pose.orientation.x = P.quaternion.x();
            pose.orientation.y = P.quaternion.y();
            pose.orientation.z = P.quaternion.z();
            pose.orientation.w = P.quaternion.w();

            cartesian_acceleration_pub.publish(pose);

        }else if(format == 'd'){

            P.joint_move_angle(t);

            if(joint == 1){
                j.data = P.j_1_position;
            }else if(joint == 2){
                j.data = P.j_2_position;
            }else if(joint == 3){
                j.data = P.j_3_position;
            }else if(joint == 4){
                j.data = P.j_4_position;
            }else if(joint == 5){
                j.data = P.j_5_position;
            }else if(joint == 6){
                j.data = P.j_6_position;
            }

            joint_pub.publish(j);

        }else if(format == 'g'){

            P.cartesian_position_planning(t);

            point.x = P.position_x;
            point.y = P.position_y;
            point.z = P.position_z;
            points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1;

            line_arrow.pose.position.x = P.position_x;
            line_arrow.pose.position.y = P.position_y;
            line_arrow.pose.position.z = P.position_z;
            line_arrow.pose.orientation.x = P.quaternion.x();
            line_arrow.pose.orientation.y = P.quaternion.y();
            line_arrow.pose.orientation.z = P.quaternion.z();
            line_arrow.pose.orientation.w = P.quaternion.w();

            //cout<<P.quaternion.x() << " "<<P.quaternion.y() << " "<<P.quaternion.z() << " " << P.quaternion.w()<<endl;  

            points.points.push_back(point);
            line_strip.points.push_back(point);

            cartesian_visualization_pub.publish(points);
            cartesian_visualization_pub.publish(line_strip);
            cartesian_visualization_pub.publish(line_arrow);

        }

        r.sleep();


    }
    
    return 0;
}