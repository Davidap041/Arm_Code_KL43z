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
