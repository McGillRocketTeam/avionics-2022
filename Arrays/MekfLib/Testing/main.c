#include "../Inc/wrapper.h"
#include <stdio.h>

int main(int argc, char* argv[]) {
        struct MEKF* c = newMEKF(0.01, 0.1, 0.1, 0.5, 0.1, 0.5);
        	
        double gyro_input[3] = { 0.0, 0.0, 0.0 }; 
        double acc_input[3] = {1.0, 0.0, 0.0};

        for(int i=0; i<100; i++) {
            MEKF_predict(c, gyro_input, acc_input);
            MEKF_update(c);
            double *p;
            p = MEKF_getPosition(c);
            printf("%d\n", p[0]);
            printf("%d\t", p[1]);
            printf("%d\t", p[2]);
        }

        deleteMEKF(c);
}