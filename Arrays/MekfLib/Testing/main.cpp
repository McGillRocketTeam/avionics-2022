
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include "../Inc/mekf_CPP.h"
#include <cmath>

void printVector(Eigen::Vector3d v) {
	std::cout << v(0) << v(1) << v(2) << std::endl;
}

void printMatrix3(Eigen::Matrix3d m) {
	std::cout << m(0, 0) << m(0, 1) << m(0, 2) << std::endl;
	std::cout << m(1, 0) << m(1, 1) << m(1, 2) << std::endl;
	std::cout << m(2, 0) << m(2, 1) << m(2, 2) << std::endl;
}

void runMEKF() {// used for testing only
	int const N = 100;
	std::cout << "cpp main" << std::endl;
	Eigen::Vector3d gyro_input;
	Eigen::Vector3d acc_input;

	MEKF mekf(0.01, 0.1, 0.1, 2, 0.1, 0.5);

	Eigen::Vector3d gyro_meas[N];
	Eigen::Vector3d acc_meas[N];
	Eigen::Vector3d gps_meas[N];

	std::cout << "loop 1" << std::endl;
	//acc and gyro data gen
	for (int i = 0; i < N; i++) {
		
		gyro_input << sin(2*120*i*3.1415),
					  sin(2*90*i*3.1415), 
					  sin(2*70*i*3.1415);
		
		acc_input << 7.0 * sin(2*40*i*3.1415),
					 2.0 * sin(2*80*i*3.1415),
					 3.0 * sin(2*50*i*3.1415);

		mekf.kf_predict(gyro_input, acc_input);
		mekf.kf_update();
		Eigen::Vector3d gps_noise;
		gps_noise << sin(10*i),
					 sin(10*i),
					 sin(10*i);

		gps_meas[i] = mekf.ra_k + gps_noise;
		gyro_meas[i] = gyro_input + gps_noise;
		acc_meas[i] = acc_input + gps_noise;
	}
	
	Eigen::Vector3d pos_pred[N];
	Eigen::Vector3d orien_pred[N];

	std::cout << "loop 2" << std::endl;
	//data consumption
	for (int i = 0; i < N; i++) {
		mekf.kf_predict(gyro_meas[i], acc_meas[i]);
		mekf.kf_update();
		std::cout << "predict worked" << std::endl;
		mekf.kf_correct(gps_meas[i]); 
		mekf.kf_update();
		std::cout << "correct worked" << std::endl;
		pos_pred[i] = mekf.ra_k;
		orien_pred[i] = mekf.dcmToEuler(mekf.Cab_k);
		printVector(mekf.ra_k);
		printMatrix3(mekf.Cab_k);
	}
	printVector(mekf.ra_k);
}

int main(void) {
	runMEKF();

	return(0);
}