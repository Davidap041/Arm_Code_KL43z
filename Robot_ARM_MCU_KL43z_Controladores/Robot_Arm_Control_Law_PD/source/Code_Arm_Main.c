#include <mpu_6050.h>
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL43Z4.h"
#include "fsl_debug_console.h"
#include "fsl_pit.h"
#include "fsl_gpio.h"
#include "function.h"
#include "perifericos.h"
#include "Kalman.h"
#include "Control_Law.h"
//#include "arm_math.h"

// Variáveis Globais
// Definição do endereçamento dos 4 sensores
acc_data_t acc_1 = { .address = 0x68, .I2C = I2C0_BASE, };
acc_data_t acc_2 = { .address = 0x68, .I2C = I2C0_BASE, };
acc_data_t acc_3 = { .address = 0x68, .I2C = I2C0_BASE, };
acc_data_t acc_4 = { .address = 0x68, .I2C = I2C0_BASE, };

// Variáveis para Print
int valor_1;
int valor_2;
int valor_3;
int valor_4;

//Variáveis de Testes e DEBUGs
uint32_t teste_motor1 = 4500;
uint32_t teste_motor2 = 3600;
uint32_t teste_motor3 = 6000;
uint32_t teste_motor4 = 6000;

// Variáveis relacionadas ao filtro de kalman
// Instancia dos Filtros
Kalman_data kalman_0 = { .Q_angle = 0.1f, .Q_bias = 0.01f, .R_measure = 0.03f,
		.angle = 0, .bias = 0 };
Kalman_data kalman_1 = { .Q_angle = 0.5f, .Q_bias = 0.01f, .R_measure = 0.03f,
		.angle = 0, .bias = 0 };
Kalman_data kalman_2 = { .Q_angle = 0.5f, .Q_bias = 0.01f, .R_measure = 0.03f,
		.angle = 0, .bias = 0 };
Kalman_data kalman_3 = { .Q_angle = 0.5f, .Q_bias = 0.01f, .R_measure = 0.03f,
		.angle = 0, .bias = 0 };

// Variáveis relacionadas aos ângulos dos elos e controlador PD
Angle rho = { .identification = 0, .kp_pd = 277.7778, .kv_pd = 36.1111, .m =
		0.2940, .b = 1.02549, .k = 0.00044956, .upper_limit = 1.5,
		.lower_limit = -1.5, .means = -0.12567 };
Angle theta1 = { .identification = 1, .kp_pd = 277.7778, .kv_pd = 36.1111, .m =
		0.2390, .b = 0.98597, .k = 0.001232, .upper_limit = 1.5, .lower_limit =
		-1.5, .means = -1.306469 };
Angle theta2 = { .identification = 2, .kp_pd = 277.7778, .kv_pd = 36.1111, .m =
		0.1440, .b = 0.951269, .k = 0.009040, .upper_limit = 1.5, .lower_limit =
		-1.5, .means = 0.4190549 };
Angle theta3 = { .identification = 3, .kp_pd = 277.7778, .kv_pd = 36.1111, .m =
		0.072, .b = 1.209418, .k = 0.006143, .upper_limit = 1.5, .lower_limit =
		-1.5, .means = 0.199037 };

// Variáveis para o cálculo dos Ângulos dos Quatro Sensores
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double Kal_Angle; // Calculated angle using a Kalman filter
double Gyro_Rate;
double pitch;

int interruption_Flag = 0;
int interruption_Cont = 0;
int interruption_Printf = 0;

float angulos_atualizados[4];
// Ângulos de controle PD
float referencia = 0.5;
int contador_degrau = 0;
bool direction = true;

void inicializar_sensores(acc_data_t *acc_n) {
	bool acc_init = ACC_Init(acc_n);
	while (!acc_init) {
//		printf("\nsensor nao respondeu.. ");
		inicializar_I2Cs();
		acc_init = ACC_Init(acc_n);
	}
}
void atualizar_leitura_angulo(int sensor) {
	if (sensor == 1) { // Motor da Base
		selecionar_sensor(1);
//		inicializar_sensores(&acc_1);
		ACC_ReadRaw(&acc_1); // atualizar valores do sensor
		gyroZ = acc_1.gz * 1.3323e-04;
		float ang_1 = motor_angulo_1();
		ang_motor = estimar_posicao_rho(ang_1);
		Gyro_Rate = -gyroZ;
		Kal_Angle = getAngle(&kalman_0, ang_motor, Gyro_Rate, Ts); // Calculate the angle using a Kalman filter
		angulos_atualizados[0] = Kal_Angle;
	}
	if (sensor == 2) { // Motor do eixo
		selecionar_sensor(2);
//		inicializar_sensores(&acc_2);

		ACC_ReadRaw(&acc_2); // atualizar valores do sensor
		accX = acc_2.ax * g * ACC_RESOLUTION;
		accZ = acc_2.az * g * ACC_RESOLUTION;
		gyroY = acc_2.gy * 1.3323e-04;

		pitch = atan2f(-accX, accZ);
		Gyro_Rate = gyroY;
		// Calculate the angle using a Kalman filter
		Kal_Angle = getAngle(&kalman_1, pitch, Gyro_Rate, Ts);
		angulos_atualizados[1] = Kal_Angle;
	}
	if (sensor == 3) {
		selecionar_sensor(3);
//		inicializar_sensores(&acc_3);

		ACC_ReadRaw(&acc_3); // atualizar valores do sensor
		accX = acc_3.ax * g * ACC_RESOLUTION;
		accY = acc_3.ay * g * ACC_RESOLUTION;
		gyroZ = acc_3.gz * 1.3323e-04;

		pitch = atan2f(accX, accY);
		Gyro_Rate = gyroZ; // Convert to deg/s
		// Calculate the angle using a Kalman filter
		Kal_Angle = getAngle(&kalman_2, pitch, Gyro_Rate, Ts);
		angulos_atualizados[2] = Kal_Angle;
	}
	if (sensor == 4) {
		selecionar_sensor(4);
//		inicializar_sensores(&acc_4);

		ACC_ReadRaw(&acc_4); // atualizar valores do sensor

		accX = acc_4.ax * g * ACC_RESOLUTION;
		accY = acc_4.ay * g * ACC_RESOLUTION;
		gyroZ = acc_4.gz * 1.3323e-04;

		pitch = atan2f(accX, accY);
		Gyro_Rate = gyroZ; // Convert to deg/s
		Kal_Angle = -getAngle(&kalman_3, pitch, Gyro_Rate, Ts); // Calculate the angle using a Kalman filter
		angulos_atualizados[3] = Kal_Angle;
	}
}
void Pd_Control_Law() {
//	Atualizar Ângulos
	for (int j = 1; j < 5; j++) {
		atualizar_leitura_angulo(j);
	}

	rho.Th = angulos_atualizados[0];
	theta1.Th = angulos_atualizados[1];
	theta2.Th = angulos_atualizados[2];
	theta3.Th = angulos_atualizados[3];

	// Calcula Referência com o atraso
	rho.dTh_ref = (rho.Th_ref - rho.Th_ref_anterior) / Ts;
	theta1.dTh_ref = (theta1.Th_ref - theta1.Th_ref_anterior) / Ts;
	theta2.dTh_ref = (theta2.Th_ref - theta2.Th_ref_anterior) / Ts;
	theta3.dTh_ref = (theta3.Th_ref - theta3.Th_ref_anterior) / Ts;

	// Guarda os Valores de Referência
	rho.Th_ref_anterior = rho.Th_ref;
	theta1.Th_ref_anterior = theta1.Th_ref;
	theta2.Th_ref_anterior = theta2.Th_ref;
	theta3.Th_ref_anterior = theta3.Th_ref;

	// Calculo dos Diferenciais Th
	rho.dTh = (rho.Th - rho.Th_anterior) / Ts;
	theta1.dTh = (theta1.Th - theta1.Th_anterior) / Ts;
	theta2.dTh = (theta2.Th - theta2.Th_anterior) / Ts;
	theta3.dTh = (theta3.Th - theta3.Th_anterior) / Ts;
	//	Guarada os valores dos Ângulos
	rho.Th_anterior = rho.Th;
	theta1.Th_anterior = theta1.Th;
	theta2.Th_anterior = theta2.Th;
	theta3.Th_anterior = theta3.Th;

	// Cálculo do Sinal de Controle u[k]
//	Control_Signal(&rho);
//	Control_Signal(&theta1);
//	Control_Signal(&theta2);
	Control_Signal(&theta3);

// Atulizar os Motores com os valores de u[k] atualizados
//	atualizar_motor_1(rho.u);
//	atualizar_motor_2(theta1.u);
//	atualizar_motor_3(theta2.u);
	atualizar_motor_4(theta3.u);

}
void PD_References() {
//	theta3.Th_ref_anterior = theta3.Th_ref;
//	if (referencia > 1.5) {
//		referencia = -1.5;
//	} else {
//		referencia = referencia + 0.001;
//	}
//	theta3.Th_ref = referencia;
	if (contador_degrau == 100) {
		if (direction == true) {
			referencia = 0.5;
			direction = false;
		} else {
			referencia = 0.5;
			direction = true;
		}
		contador_degrau = 0;
	} else {
		contador_degrau++;
	}
	theta3.Th_ref = referencia;

	int valor_da = 1304.45 * referencia + 2048;
	atualizar_dac(valor_da);
}
void Interruption_Printf() {
	int print = 10000 * theta3.u;
	int print2 = 10000 * referencia;
	printf("\r\nValor do angulo 4: %d, valor da referencia: %d.e-4", print,
			print2);
}

void PIT_DriverIRQHandler() {
	if (PIT_GetStatusFlags(PIT, kPIT_Chnl_0)) {
		PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
		interruption_Flag = 1;
		if (interruption_Cont == 10) {
			interruption_Cont = 0;
			Interruption_Printf();
		} else {
			interruption_Cont++;
		}
	}
}

void Interruption_Program() {
	GPIO_PortToggle(BOARD_INTERRUPTION_GPIO, 1U << BOARD_INTERRUPTION_PIN);
	PD_References();
	Pd_Control_Law();
}
int main(void) {
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
	printf("\r\nStarting Kl");

//	inicializar_PWMs();
	atualizar_motores(4500, 3600, 6000, 6000);
	delay(10000);

	inicializar_I2Cs();
	printf("\r\nComunicacao I2C inicializada ");

	selecionar_sensor(1);
	inicializar_sensores(&acc_1);
	printf("\r\nComunicacao inicializada : Sensor 1 ");
	selecionar_sensor(2);
	inicializar_sensores(&acc_2);
	printf("\r\nComunicacao inicializada : Sensor 2 ");
	selecionar_sensor(3);
	inicializar_sensores(&acc_3);
	printf("\r\nComunicacao inicializada : Sensor 3 ");
	selecionar_sensor(4);
	inicializar_sensores(&acc_4);
	printf("\r\nComunicacao inicializada : Sensor 4 ");

	printf("\r\nTodos os Sensores okay");
	inicializar_dac(3.14);
	configurar_interrupcao_Hz(100);
// Ficou em 3.855 Hz lendo os quatro sensores

	while (1) {
		if (interruption_Flag == 1) {
			Interruption_Program();
			interruption_Flag = 0;
		}

//		atualizar_motores(teste_motor1, teste_motor2, teste_motor3,
//				teste_motor4);
	}
	return 0;
}
