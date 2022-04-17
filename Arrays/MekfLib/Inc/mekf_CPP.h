#ifndef MEKF_CPP_H_
#define MEKF_CPP_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class MEKF {

    public:
        MEKF ();

        int a = 0;
        //predict step
        void kf_predict();

        //correct step
        void kf_correct();

        // t -> t+1
        void kf_update();
};
#endif 