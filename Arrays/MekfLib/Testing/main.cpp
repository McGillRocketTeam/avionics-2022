
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "../Inc/mekf_CPP.h"

// used for testing only
int main(void) {

    
	std::cout << "cpp main" << std::endl;

	MEKF mekf(0.01, 0.1, 0.1, 2, 0.1, 0.5);
	std::cout << mekf.Cab_k << std::endl;
	mekf.kf_predict();
	std::cout << "end main" << std::endl;

	return(0);
}