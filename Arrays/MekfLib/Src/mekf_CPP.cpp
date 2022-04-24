#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <cmath>
#include "../Inc/mekf_CPP.h"
#include "C:/Users/Dell/CPP_libraries/eigen-3.4.0/Eigen/Dense"

//Disclaimer: I'm aware that this code is poorly optimized and does things in a very roundabout way.
//            I just hope it works. I don't have the energy to refactor.

MEKF::MEKF(double dt,double sigma_gyro, double sigma_acc, double sigma_gps, double P_init_orien, double P_init_pos) {
    //double
    this->T = dt;

    //vectors
    this->Va_k_1 = Eigen::Vector3d::Zero(); //3x1
    this->Va_k = Eigen::Vector3d::Zero(); //3x1
    this->ra_k_1 = Eigen::Vector3d::Zero(); //3x1
    this->ra_k = Eigen::Vector3d::Zero(); //3x1
    this->ga << 0.0, 
                0.0,
                -9.81; //3x1

    this->gyro = Eigen::Vector3d::Zero(); //3x1
    this->acc = Eigen::Vector3d::Zero(); //3x1
    this->gps = Eigen::Vector3d::Zero(); //3x1

    this->correction_term = Eigen::MatrixXd::Zero(9, 1); //9x1 zeros

    //static matrices
    this->Cab_k_1 = initRotation(0.2, 0.3, 0.5); //3x3
    this->Cab_k = Eigen::MatrixXd::Identity(3, 3); //3x3

    //dynamic matrices
    this->A = Eigen::MatrixXd::Zero(9, 9); //9x9 zeros

    double cov_gyro = sigma_gyro * sigma_gyro;
    double cov_acc = sigma_acc * sigma_acc;
    double cov_gps = sigma_gps * sigma_gps;
    this->Q.resize(6, 6);
    this->Q <<  cov_gyro, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, cov_gyro, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, cov_gyro, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, cov_acc, 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, cov_acc, 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.0, cov_acc; //6x6 
    this->R = (Eigen::MatrixXd::Identity(3, 3).array() * cov_gps).matrix(); //3x3

    this->P_k = Eigen::MatrixXd::Identity(9, 9); //9x9
    this->P_k_1.resize(9, 9);
    this->P_k_1 << P_init_orien, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //could be done with diag matrices? 
                   0.0, P_init_orien, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                   0.0, 0.0, P_init_orien, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, P_init_pos, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, P_init_pos, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, P_init_pos, 0.0, 0.0, 0.0, 
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, P_init_pos, 0.0, 0.0, 
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, P_init_pos, 0.0, 
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, P_init_pos; //9x9
    this->L = Eigen::MatrixXd::Zero(9, 6); //9x6 
    this->S1 = Eigen::MatrixXd::Identity(9, 9); //9x9
    this->S2 = Eigen::MatrixXd::Identity(3, 3); //3x3

    this->K_k = Eigen::MatrixXd::Zero(9, 3); //9x3 
    this->M_k = Eigen::MatrixXd::Identity(3, 3); //3x3
    this->C_k.resize(3, 9);
    this->C_k << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0; //3x9 

}

void MEKF::kf_predict(double gyro_input, double acc_input){
    

}

void MEKF::kf_correct() {
    std::cout << "cpp correct" << std::endl;
    return;
}

void MEKF::kf_update() {
    std::cout << "cpp update" << std::endl;
    this->Cab_k_1 = this->Cab_k;
    this->Va_k_1 = this->Va_k;
    this->ra_k_1 = this->ra_k;
    this->P_k_1 = this->P_k;
    return;
}

//Helper methods 

//creates a rotation matrix from Euler angles 
Eigen::Matrix3d MEKF::initRotation(double roll,double yaw, double pitch) {

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();
    return rotationMatrix;
}

//applies the cross operator on a 3d vector
Eigen::MatrixXd MEKF::crossOperator(Eigen::Vector3d c) {
    Eigen::MatrixXd cross;
    cross << 0.0, -c(2), c(1),
             c(2), 0.0, -c(0),
             -c(1), c(0), 0.0;
    return cross;
}