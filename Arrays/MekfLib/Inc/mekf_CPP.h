#ifndef MEKF_CPP_H_
#define MEKF_CPP_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "C:/Users/Dell/CPP_libraries/eigen-3.4.0/Eigen/Dense"

class MEKF {

    public:
        MEKF (double dt,double sigma_gyro, double sigma_acc, double sigma_gps, double P_init_orien, double P_init_pos);

        //variables
        int a = 0;
        Eigen::Matrix3d Cab_k_1;
        Eigen::Vector3d Va_k_1;
        Eigen::Vector3d ra_k_1;
        Eigen::Vector3d ga; //3x1

        Eigen::MatrixXd A; //9x9 zeros
        double T = 0.1;

        Eigen::MatrixXd Q; //6x6 
        Eigen::MatrixXd R; //3x3

        Eigen::Vector3d gyro; //3x1
        Eigen::Vector3d acc; //3x1
        Eigen::Vector3d gps; //3x1

        Eigen::Matrix3d Cab_k;
        Eigen::Matrix3d Va_k;
        Eigen::Matrix3d ra_k;

        Eigen::MatrixXd P_k; //9x9
        Eigen::MatrixXd P_k_1;
        Eigen::MatrixXd L; //9x6 
        Eigen::MatrixXd S1;  
        Eigen::MatrixXd S2;

        Eigen::VectorXd correction_term; //9x1 zeros
        Eigen::MatrixXd K_k; //9x3 
        Eigen::MatrixXd M_k; //3x3
        Eigen::MatrixXd C_k; //3x9 

        //predict step
        void kf_predict();

        //correct step
        void kf_correct();

        // t -> t+1
        void kf_update();

        //Helpers 
        Eigen::Matrix3d initRotation(double roll, double yaw, double pitch);
};
#endif 