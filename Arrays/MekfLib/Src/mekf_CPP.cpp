#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "../Inc/mekf_CPP.h"
#include "C:/Users/Dell/CPP_libraries/eigen-3.4.0/Eigen/Dense"

MEKF::MEKF() { //user defined default constructor
    this->a = 3;
}

void MEKF::kf_predict(){
    Eigen::Matrix3d r;
    r << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    std::cout << "cpp predict" << std::endl;
    std::cout << r << std::endl;
}

void MEKF::kf_correct() {
    std::cout << "cpp correct" << std::endl;
}

void MEKF::kf_update() {
    std::cout << "cpp update" << std::endl;
}