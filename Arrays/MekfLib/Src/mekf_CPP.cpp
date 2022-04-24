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
    this->Cab_k_1 = initRotation(0.2, 0.3, 0.5); //3x3
    this->Va_k_1 = Eigen::Vector3d::Zero(); //3x1
    this->ra_k_1 = Eigen::Vector3d::Zero(); //3x1
    this->ga << 0, 
                0,
                -9.81; //3x1

    this->A = Eigen::MatrixXd::Zero(9, 9); //9x9 zeros
    this->T = dt;

    double cov_gyro = pow(sigma_gyro, 2);
    double cov_acc = pow(sigma_acc, 2);
    double cov_gps = pow(sigma_gps, 2);
    this->Q.resize(6, 6);
    this->Q <<  cov_gyro, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, cov_gyro, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, cov_gyro, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, cov_acc, 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, cov_acc, 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.0, cov_acc; //6x6 
    this->R = (Eigen::MatrixXd::Identity(3, 3).array() * cov_gps).matrix(); //3x3
    std::cout << "R : " << this->R << std::endl; 

    this->gyro = Eigen::Vector3d::Zero(); //3x1
    this->acc = Eigen::Vector3d::Zero(); //3x1
    this->gps = Eigen::Vector3d::Zero(); //3x1

    this->Cab_k = Eigen::MatrixXd::Identity(3, 3); //3x3
    this->Va_k = Eigen::Vector3d::Zero(); //3x1
    this->ra_k = Eigen::Vector3d::Zero(); //3x1

    this->P_k = Eigen::MatrixXd::Identity(9, 9); //9x9
    this->P_k_1.resize(9, 9);
    this->P_k_1 << P_init_orien, 0, 0, 0, 0, 0, 0, 0, 0, //could be done with diag matrices? 
                   0, P_init_orien, 0, 0, 0, 0, 0, 0, 0, 
                   0, 0, P_init_orien, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, P_init_pos, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, P_init_pos, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, P_init_pos, 0, 0, 0, 
                   0, 0, 0, 0, 0, 0, P_init_pos, 0, 0, 
                   0, 0, 0, 0, 0, 0, 0, P_init_pos, 0, 
                   0, 0, 0, 0, 0, 0, 0, 0, P_init_pos; //9x9
    this->L = Eigen::MatrixXd::Zero(9, 6); //9x6 
    this->S1 = Eigen::MatrixXd::Identity(9, 9); //9x9
    this->S2 = Eigen::MatrixXd::Identity(3, 3); //3x3

    this->correction_term = Eigen::MatrixXd::Zero(9, 1); //9x1 zeros
    this->K_k = Eigen::MatrixXd::Zero(9, 3); //9x3 
    this->M_k = Eigen::MatrixXd::Identity(3, 3); //3x3
    this->C_k.resize(3, 9);
    this->C_k << 0, 0, 0, 0, 0, 0, 1, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 1, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 1; //3x9 

    std::cout << this->R << std::endl;
}

void MEKF::kf_predict(){
    Eigen::Matrix3d r;
    r << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
    std::cout << "cpp predict" << std::endl;
    std::cout << r << std::endl;
    std::cout << r(1, 2) << std::endl;
}

void MEKF::kf_correct() {
    std::cout << "cpp correct" << std::endl;
    return;
}

void MEKF::kf_update() {
    std::cout << "cpp update" << std::endl;
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