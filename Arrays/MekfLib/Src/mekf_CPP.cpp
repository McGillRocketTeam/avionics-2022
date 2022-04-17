#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "../Inc/mekf_CPP.h"

MEKF::MEKF() { //user defined default constructor
    this->a = 3;
}

void MEKF::kf_predict(){
    std::cout << "cpp predict" << std::endl;
}

void MEKF::kf_correct() {
    std::cout << "cpp correct" << std::endl;
}

void MEKF::kf_update() {
    std::cout << "cpp update" << std::endl;
}