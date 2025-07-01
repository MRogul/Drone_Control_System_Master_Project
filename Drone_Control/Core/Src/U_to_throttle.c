//U_to_throttle.c

#include <U_to_throttle.h>
#include "arm_math.h"

#define kf 0.0001
#define n_max 29250*0.8
#define w_max 2*3.14*n_max/60
#define num_DOF 3


float32_t a[4]={1,2,2,2};
float32_t b[4]={1,0,3,1};
float32_t c[4];

arm_matrix_instance_f32 A;
arm_matrix_instance_f32 B;
arm_matrix_instance_f32 C;



void GetThrottle(float u_vector[num_DOF]){

}

void test(){
	arm_mat_init_f32(&A, 2, 2, a);
	arm_mat_init_f32(&B, 2, 2, b);
	arm_mat_init_f32(&C, 2, 2, c);


	arm_mat_mult_f32(&A, &B, &C);
}
