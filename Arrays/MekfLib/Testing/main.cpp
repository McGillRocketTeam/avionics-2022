
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "../Inc/mekf_CPP.h"

// used for testing only
int main(void) {

    
	std::cout << "cpp main" << std::endl;

	MEKF mekf(0.01, 0.1, 0.1, 2, 0.1, 0.5);
	for (int i = 0; i < 100; i++) {
		std::cout << "ra_k" << std::endl;
		std::cout << mekf.ra_k << std::endl;
		Eigen::Vector3d gyro_input;
		gyro_input << 0.0,
					  0.0, 
					  0.0;
		Eigen::Vector3d acc_input;
		acc_input << 1.0,
					 0.0,
					 0.0;
		mekf.kf_predict(gyro_input, acc_input);
		mekf.kf_update();
	}
	std::cout << "end main" << std::endl;

	return(0);
}