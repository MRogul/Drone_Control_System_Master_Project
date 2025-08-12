//U_to_throttle.c

#include <U_to_throttle.h>
#include "arm_math.h"
#include <math.h>

#define g 9.81
#define m 2.5
#define L 0.17

#define n_max 29250*0.8
#define w_max 2*M_PI*n_max/60
#define Ix 0.02
#define Iy 0.02
#define Iz 0.04

#define kf 0.0001
#define b 0.0004
#define km 0.0015


///3DOF///
float32_t M_date[12]={
		L/sqrt(2),-L/sqrt(2),-L/sqrt(2),L/sqrt(2),
		L/sqrt(2),L/sqrt(2),-L/sqrt(2),-L/sqrt(2),
		km, -km, km, -km
};
////2DOF////
/*
float32_t M_date[8]={
		L/sqrt(2),-L/sqrt(2),-L/sqrt(2),L/sqrt(2),
		L/sqrt(2),L/sqrt(2),-L/sqrt(2),-L/sqrt(2),
};
*/
float32_t MT_date[12];
float32_t MMt_date[9];
float32_t MMt_inv_date[9];
float32_t M_pseudo_date[12];
float32_t T_date[4];

arm_matrix_instance_f32 Mixer;
arm_matrix_instance_f32 MixerT;
arm_matrix_instance_f32 MMt;
arm_matrix_instance_f32 MMt_inv;
arm_matrix_instance_f32 M_pseudo;
arm_matrix_instance_f32 U_vec;
arm_matrix_instance_f32 T_vec;


void GetThrottle(float *u_vector, float *speed){
	/*
	float U_date[sizeof(u_vector)];
	for (int i=0; i<sizeof(u_vector);i++){
		U_date[i]=u_vector[i];
	}
	*/
	arm_mat_init_f32(&U_vec, 3, 1, u_vector);
	arm_mat_init_f32(&T_vec, 4, 1, T_date);

	arm_mat_init_f32(&Mixer, 3, 4, M_date);
	arm_mat_init_f32(&MixerT, 4, 3, MT_date);
	arm_mat_init_f32(&MMt, 3, 3, MMt_date);
	arm_mat_init_f32(&MMt_inv, 3, 3, MMt_inv_date);
	arm_mat_init_f32(&M_pseudo, 4, 3, M_pseudo_date);

	//Transponowanie macierzy M-> M^T
	arm_mat_trans_f32(&Mixer, &MixerT);

	//Mnożenie macierzy M*M^T
	arm_mat_mult_f32(&Mixer, &MixerT, &MMt);

	//Odwracanie macierzy (M*M^T)^-1
	arm_mat_inverse_f32(&MMt, &MMt_inv);

	//Pseudoodwrotność (ostatnie mnożenie) M^T*(M*M^T)^-1
	arm_mat_mult_f32(&MixerT, &MMt_inv, &M_pseudo);

	//Wyliczanie wektora T -> T=M_peudo*u
	arm_mat_mult_f32(&M_pseudo, &U_vec, &T_vec);

	//Przeliczenie na throttle

	speed[0]=(sqrtf(T_date[0]/kf)*2000/w_max)+48;
	speed[1]=(sqrtf(T_date[1]/kf)*2000/w_max)+48;
	speed[2]=(sqrtf(T_date[2]/kf)*2000/w_max)+48;
	speed[3]=(sqrtf(T_date[3]/kf)*2000/w_max)+48;


}
