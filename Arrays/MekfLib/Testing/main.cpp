
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
		mekf.kf_predict(1.0, 1.0);
	}
	std::cout << "end main" << std::endl;

	return(0);
}