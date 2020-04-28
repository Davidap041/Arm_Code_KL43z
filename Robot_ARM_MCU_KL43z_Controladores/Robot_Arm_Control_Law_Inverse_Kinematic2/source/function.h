#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_
#include <math.h>
#include <mpu_6050.h>

// Parâmetros para estimação de posição do Rho
#define a0 1
#define a1 -3.635
#define a2 5.683
#define a3 -4.721
#define a4 2.068
#define a5 -0.3792

#define b0 0
#define b1 0.01931

// Variáveis da Estimação de posição do Elo 1 Ângulo Rho
// valores Passados de Entrada e Saída atualidos na frequência de 100Hz
float y_estimado[5], u_estimado[2];
float ang_motor;

// Parâmetros para as matrizes de Referência
#define pi 3.1415926
#define pi_2 1.570796

typedef struct {
	float ref_time;

	float x;
	float y;
	float z;

	float rho;
	float theta1;
	float theta2;
	float theta3;

} Reference_data;


void References_Circle(Reference_data *reference) {
	reference->ref_time = reference->ref_time + Ts;
	float x, y, z;
	if (reference->ref_time < 5 * pi_2) {
		x = 2 * cos(reference->ref_time);
		y = 8;
		z = 2 * sin(reference->ref_time);
		z = 25 + z;

	} else {
//		x = 8 + 3 * cos(5 * pi_2);
//		y = 3;
//		z = 12 + 3 * sin(5 * pi_2);
		reference->ref_time = pi_2;
	}
	reference->x = x;
	reference->y = y;
	reference->z = z;
}

void Inverse_Kinematic(Reference_data *reference) {
#define L3 8
#define L2 13
#define L1 13
#define L1L1 169
#define L2L2  169

	double r, phi, beta, qsi, qsi_p1, qsi_p2, qsi_p3, qsi_p4, rho, theta1,
			theta2, theta2_p1, theta2_p2, theta2_p3, theta2_p4, theta3, beta_1,
			beta_2;

	float x, y, z;
	x = reference->x;
	y = reference->y;
	z = reference->z;

//	r(i) = sqrt(x(i)^2 + z_(i)^2);
	r = sqrtf(x * x + z * z);

//	phi(i) = -atan2(y(i),r(i));
	phi = -atan2f(y, r);

//	beta(i) = atan2(y(i)-l3*sin(phi(i)),r(i) - l3*cos(phi(i)));
	beta_1 = y - L3 * sin(phi);
	beta_2 = r - L3 * cos(phi);

	beta = atan2f(beta_1, beta_2);

//	qsi(i) = acos(((r(i) - l3*cos(phi(i)))^2 + (y(i) - l3*sin(phi(i)))^2 + l1^2 - l2^2)
//				/(2*l1*sqrt((r(i) - l3*cos((phi(i))))^2 + (y(i) - l3*sin(phi(i)))^2)));
	qsi_p1 = ((r - L3 * cos(phi))) * (r - L3 * cos(phi));
	qsi_p2 = (y - L3 * sin(phi)) * (y - L3 * sin(phi)) + L1L1 - L2L2;

	qsi_p3 = (r - L3 * cos(phi)) * (r - L3 * cos(phi));
	qsi_p4 = (y - L3 * sin(phi)) * (y - L3 * sin(phi));
	qsi_p3 = 2 * L1 * sqrtf(qsi_p3 + qsi_p4);

	qsi = (qsi_p1 + qsi_p2) / (qsi_p3);

	qsi = acosf(qsi);

//	rho(i) = atan2(x(i),z_(i));
	rho = atan2f(x, z);

//	theta1(i) = (beta(i) + qsi(i)) - pi/2;
	theta1 = (beta + qsi) - pi_2;

//	theta2(i) = -acos(((r(i) - l3*cos(phi(i)))^2 +
//			(y(i) - l3*sin(phi(i)))^2 - l1^2 - l2^2)/(2*l1*l2));
	theta2_p1 = (r - L3 * cos(phi)) * (r - L3 * cos(phi));
	theta2_p2 = (y - L3 * sin(phi)) * (y - L3 * sin(phi));
	theta2_p3 = L1L1 + L2L2;
	theta2_p4 = 2 * L1 * L2;
	theta2 = -acosf((theta2_p1 + theta2_p2 - theta2_p3) / theta2_p4);

//	theta3(i) = (phi(i) - theta1(i) - pi/2 - theta2(i));
	theta3 = phi - theta1 - pi_2 - theta2;

//	Posicionamento dos Eixos
	rho = -rho;
	theta1 = -theta1;
	theta2 = -theta2;
	theta3 = -theta3;

	reference->rho = rho;
	if (theta1 > 0 && theta1 < 5) {
		reference->theta1 = theta1;
	}
	if (theta2 > 0 && theta2 < 5) {
		reference->theta2 = theta2;
	}
	reference->theta3 = theta3;

}

static inline float GetAccPitch(const acc_data_t *Data, int eixo) {

	float acc_x = (Data->ax - Data->M_ax) * g * ACC_RESOLUTION;
	float acc_y = (Data->ay - Data->M_ay) * g * ACC_RESOLUTION;
	float acc_z = (Data->az - Data->M_az) * g * ACC_RESOLUTION;
	float ang_pitch;
	if (eixo == 1)
		ang_pitch = atan2f(acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z));
	if (eixo == 2)
		ang_pitch = atan2f(acc_y, sqrtf(acc_x * acc_x + acc_z * acc_z));
	if (eixo == 3)
		ang_pitch = atan2f(acc_z, sqrtf(acc_x * acc_x + acc_y * acc_y));
	return ang_pitch;
}

float GetAccRoll(const acc_data_t *Data, int eixo) {

	float Gyro_x = (Data->ax) * g * ACC_RESOLUTION;
	float Gyro_y = (Data->ay) * g * ACC_RESOLUTION;
	float Gyro_z = (Data->az) * g * ACC_RESOLUTION;
	float ang_Roll;
	if (eixo == 1)
		ang_Roll = atan2f(Gyro_y, Gyro_z);
	if (eixo == 2)
		ang_Roll = atan2f(Gyro_x, Gyro_z);
	if (eixo == 3)
		ang_Roll = atan2f(Gyro_x, Gyro_y);
	return ang_Roll;
}
float estimar_posicao_rho(float entrada) {
//    0.01931 z^-1
//-------------------------------------------------------------------
//1 - 3.635 z^-1 + 5.683 z^-2 - 4.721 z^-3 + 2.068 z^-4 - 0.3792 z^-5

//	Ordem do Polinômio de Estimação
	int k = 6;

	y_estimado[k - 5] = y_estimado[k - 4];
	y_estimado[k - 4] = y_estimado[k - 3];
	y_estimado[k - 3] = y_estimado[k - 2];
	y_estimado[k - 2] = y_estimado[k - 1];
	y_estimado[k - 1] = y_estimado[k];

	u_estimado[k - 1] = u_estimado[k];
	u_estimado[k] = entrada;

	y_estimado[k] = b0 * u_estimado[k] + b1 * u_estimado[k - 1]
			- a1 * y_estimado[k - 1] - a2 * y_estimado[k - 2]
			- a3 * y_estimado[k - 3] - a4 * y_estimado[k - 4]
			- a5 * y_estimado[k - 5];

	return y_estimado[k];
}

#endif /* FUNCTIONS_H_ */
