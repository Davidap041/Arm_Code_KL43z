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
Kalman_data kalman_1 = { .Q_angle = 0.001f, .Q_bias = 0.0001f, .R_measure =
		0.03f, .angle = 0, .bias = 0 };
Kalman_data kalman_2 = { .Q_angle = 0.001f, .Q_bias = 0.0001f, .R_measure =
		0.03f, .angle = 0, .bias = 0 };
Kalman_data kalman_3 = { .Q_angle = 0.0001f, .Q_bias = 0.01f,
		.R_measure = 0.03f, .angle = 0, .bias = 0 };

// Variáveis relacionadas aos ângulos dos elos e controlador PD
Angle rho = { .identification = 0, .kp_pd = 2.7, .kv_pd = 0.3, .m = 0.2940, .b =
		1.02549, .k = 0.00044956, .upper_limit = rho_angle_max, .lower_limit =
rho_angle_min, .means = 0, .Th_ref = 0 };

Angle theta1 = { .identification = 1, .kp_pd = 2.7, .kv_pd = 0.3, .m = 0.2390,
		.b = 0.98597, .k = 0.001232, .upper_limit = theta1_angle_max,
		.lower_limit = theta1_angle_min, .means = 1.3065, .Th_ref = 0 };

Angle theta2 = { .identification = 2, .kp_pd = 2.7, .kv_pd = 0.3, .m = 0.1440,
		.b = 0.951269, .k = 0.009040, .upper_limit = theta2_angle_min,
		.lower_limit = theta2_angle_max, .means = 1.9899, .Th_ref = 0 };

Angle theta3 =
		{ .identification = 3, .kp_pd = 5, .kv_pd = 0.6, .m = 0.072, .b =
				1.209418, .k = 0.006143, .upper_limit =
		theta3_angle_max, .lower_limit = theta3_angle_min, .means = 1.3718,
				.Th_ref = 0 };
//Angle theta3 = { .identification = 3, .kp_pd = 277.7778, .kv_pd = 36.1111, .m =
//		0.072, .b = 1.209418, .k = 0.006143, .upper_limit = theta4_max,
//		.lower_limit = theta4_min, .means = 0.199037, .Th_ref = 0.5 };

// Variáveis para o cálculo dos Ângulos dos Quatro Sensores
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double Kal_Angle; // Calculated angle using a Kalman filter
double Gyro_Rate;
double pitch;
float ang_2 = 0;

int interruption_Flag = 0;
int interruption_Cont = 0;
int interruption_Printf = 0;
uint16_t contador_degrau = 0;
bool direction;
bool inicializar_programa = true;

// Variáveis para instancias as referências
Reference_data Circle = { .ref_time = pi_2 }, Triangle, Straight, Saw;

float angulos_atualizados[4];
// Ângulos de controle PD
float referencia = 0.5;

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
		Gyro_Rate = -gyroZ;
		rho.angulo_motor = motor_angulo_1();

		rho.estimativa_motor = estimar_posicao_rho(rho.angulo_motor);

		Kal_Angle = getAngle(&kalman_0, rho.estimativa_motor, Gyro_Rate, Ts); // Calculate the angle using a Kalman filter
		angulos_atualizados[0] = Kal_Angle;

		rho.Th = angulos_atualizados[0] - pi_2;

	}
	if (sensor == 2) { // Theta1
		selecionar_sensor(2);
//		inicializar_sensores(&acc_2);
		ACC_ReadRaw(&acc_2); // atualizar valores do sensor
		accX = acc_2.ax * g * ACC_RESOLUTION;
		accZ = acc_2.az * g * ACC_RESOLUTION;
		gyroY = acc_2.gy * 1.3323e-04;

		pitch = atan2f(-accX, accZ);
		Gyro_Rate = gyroY;
		// Calculate the angle using a Kalman filter
//		Kal_Angle = getAngle(&kalman_1, pitch, Gyro_Rate, Ts); // Antes
		Kal_Angle = -getAngle(&kalman_1, pitch, Gyro_Rate, Ts);
		angulos_atualizados[1] = Kal_Angle;

		float dif_ruido = (angulos_atualizados[1] - theta1.Th) * 1000;
		theta1.dif_ruido = abs(dif_ruido);

		theta1.Th = angulos_atualizados[1];
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
//		Antes
//		Kal_Angle = getAngle(&kalman_2, pitch, Gyro_Rate, Ts);
//		angulos_atualizados[2] = Kal_Angle; Antes

//		Depois
		Kal_Angle = getAngle(&kalman_2, pitch, Gyro_Rate, Ts);
		angulos_atualizados[2] = Kal_Angle + pi_2;
//		Theta2' = theta2 -theta1
		angulos_atualizados[2] = angulos_atualizados[2]
				- angulos_atualizados[1];
		float dif_ruido = (angulos_atualizados[2] - theta2.Th) * 1000;
		theta2.dif_ruido = abs(dif_ruido);
//		if (theta2.dif_ruido < 10)
		theta2.Th = angulos_atualizados[2];
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
// Antes
//		Kal_Angle = -getAngle(&kalman_3, pitch, Gyro_Rate, Ts); // Calculate the angle using a Kalman filter
//		angulos_atualizados[3] = Kal_Angle;
// Depois
		Kal_Angle = getAngle(&kalman_3, pitch, Gyro_Rate, Ts); // Calculate the angle using a Kalman filter
		angulos_atualizados[3] = Kal_Angle + pi_2;
//		Theta3' = Theta3 -Theta2-Theta1
		angulos_atualizados[3] = angulos_atualizados[3] - angulos_atualizados[2]
				- angulos_atualizados[1];

		theta3.Th = angulos_atualizados[3];
	}
}
void Pd_Control_Law(Reference_data *reference) {
//	Atualizar Valores de Referencia
	rho.Th_ref = reference->rho;
	theta1.Th_ref = reference->theta1;
	theta2.Th_ref = reference->theta2;
	theta3.Th_ref = reference->theta3;

// Calcula Referência com o atraso
	rho.dTh_ref = (rho.Th_ref - rho.Th_ref_anterior);
	theta1.dTh_ref = (theta1.Th_ref - theta1.Th_ref_anterior);
	theta2.dTh_ref = (theta2.Th_ref - theta2.Th_ref_anterior);
	theta3.dTh_ref = (theta3.Th_ref - theta3.Th_ref_anterior);

// Guarda os Valores de Referência
	rho.Th_ref_anterior = rho.Th_ref;
	theta1.Th_ref_anterior = theta1.Th_ref;
	theta2.Th_ref_anterior = theta2.Th_ref;
	theta3.Th_ref_anterior = theta3.Th_ref;
// Calculo dos Diferenciais Th
	rho.dTh = (rho.Th - rho.Th_anterior);
	theta1.dTh = (theta1.Th - theta1.Th_anterior);
	theta2.dTh = (theta2.Th - theta2.Th_anterior);
	theta3.dTh = (theta3.Th - theta3.Th_anterior);

//	Guarada os valores dos Ângulos
	rho.Th_anterior = rho.Th;
	theta1.Th_anterior = theta1.Th;
	theta2.Th_anterior = theta2.Th;
	theta3.Th_anterior = theta3.Th;

// Cálculo do Sinal de Controle u[k]
	Control_Signal(&rho);
	Control_Signal(&theta1);
	Control_Signal(&theta2);
	Control_Signal(&theta3);

}
void Open_Loop_Control_Law(Reference_data *reference) {

	rho.Th_ref = reference->rho;
	theta1.Th_ref = reference->theta1;
	theta2.Th_ref = reference->theta2;
	theta3.Th_ref = reference->theta3;

	rho.u = rho.Th_ref;
	theta1.u = theta1.Th_ref;
	theta2.u = theta2.Th_ref;
	theta3.u = theta3.Th_ref;
}
void References_Saw(Reference_data *reference) {
	if (contador_degrau >= 500) {
		if (direction == true) {
			referencia = 1.5;
			direction = false;
		} else {
			referencia = 2.5;
			direction = true;
		}
		contador_degrau = 0;
	} else {
		contador_degrau++;
	}
	reference->rho = referencia;
	reference->theta1 = referencia;
	reference->theta2 = referencia;
	reference->theta3 = referencia;
//
//	rho.Th_ref = referencia; // 1 a -1 -> .kp_pd = 6.945, .kv_pd = 0.9025
//	theta1.Th_ref = referencia; // 0.2 a 0.9 -> .kp_pd = 6.945, .kv_pd = 0.9025
//	theta2.Th_ref = referencia; // 1.5 a 2.5 -> .kp_pd = 6.945, .kv_pd = 0.9025
//	theta3.Th_ref = referencia; // 1 a 1.6 -> .kp_pd = 6.945, .kv_pd = 0.9025

}

void Refresh_DAC() {
	int valor_da = 1304.45 * theta3.Th + 2048;
	atualizar_dac(valor_da);
}
void Interruption_Printf() {
//	int a = 1000 * rho.Th;
//	int b = 1000 * theta1.Th;
//	int c = 1000 * theta2.Th;
//	int d = 1000 * theta3.Th;
//	printf("\r\nrho:%d theta1:%d theta2:%d theta3:%d  ", a, b, c, d);
//
//	int a_1 = 1000 * Circle.x;
//	int b_1 = 1000 * Circle.y;
//	int c_1 = 1000 * Circle.z;
//	int d_1 = 1000 * Circle.ref_time;
//	printf("\r\nx:%d y:%d z:%d time:%d -> ", a_1, b_1, c_1, d_1);
//
	int a_2 = 1000 * Circle.rho;
	int b_2 = 1000 * rho.Th;
	int c_2 = 1000 * Circle.theta1;
	int d_2 = 1000 * theta1.Th;
	printf("\r\nrho:%d rho[Th]:%d theta1:%d theta1[Th]:%d", a_2, b_2, c_2, d_2);

	int a_3 = 1000 * Circle.theta2;
	int b_3 = 1000 * theta2.Th;
	int c_3 = 1000 * Circle.theta3;
	int d_3 = 1000 * theta3.Th;

	printf(
			" Theta2:%d[10^-3] theta2[Th]:%d[10^-3] theta3:%d[10^-3] theta3[Th]:%d[10^-3]",
			a_3, b_3, c_3, d_3);

//	int a_0 = 1000 * rho.Th_ref;
//	int b_0 = 1000 * rho.u;
//	int c_0 = 1000 * rho.Th;
//	int d_0 = 1000 * rho.estimativa_motor;
//	int e_0 = 1000 * rho.angulo_motor;
//	int f_0 = rho.dif_ruido;
//
//	printf(
//			"\n\r rho_ref:%d[10^-3] rho[u]:%d[10^-3] rho[Th]:%d[10^-3]  dif.rho:%d ang_estimado:%d ang_motor : %d",
//			a_0, b_0, c_0, f_0, d_0, e_0);
////
//	int a_1 = 1000 * theta1.Th_ref;
//	int b_1 = 1000 * theta1.u;
//	int c_1 = 1000 * theta1.Th;
//	int f_1 = theta1.dif_ruido;
//
//	printf(
//			"\r\nth1_ref:%d[10^-3] th1[u]:%d[10^-3] th1[Th]:%d[10^-3]  dif.th1:%d...",
//			a_1, b_1, c_1, f_1);
//
//	int a_2 = 1000 * theta3.Th_ref;
//	int b_2 = 1000 * theta3.u;
//	int c_2 = 1000 * theta3.Th;
//
//	int f_2 = theta3.dif_ruido;
//
//	printf(
//			"\r\nth3_ref:%d[10^-3] th3[u]:%d[10^-3] th3[Th]:%d[10^-3]  dif.th3:%d",
//			a_2, b_2, c_2, f_2);

//	int a_3 = 1000 * rho.u;
//	int b_3 = 1000 * theta1.u;
//	int c_3 = 1000 * theta2.u;
//	int d_3 = 1000 * theta3.u;
//
//	printf(
//			"->> rho[u]:%d[10^-3] theta1[u]:%d[10^-3] theta2[u]:%d[10^-3] theta3[u]:%d[10^-3]",
//			a_3, b_3, c_3, d_3);
//	int a_2 = 1000 * rho.u;
//	int b_2 = 1000 * theta1.u;
//	int c_2 = 1000 * theta2.u;
//	int d_2 = 1000 * theta3.u;
//
//	printf(
//			"\n\r rho:%d[10^-3] TPM0:%d[10^-3] ang_motor:%d[10^-3] motor_ang:%d[10^-3]",
//			a_2, b_2, c_2, d_2);

}

void PIT_DriverIRQHandler() {
	if (PIT_GetStatusFlags(PIT, kPIT_Chnl_0)) {
		PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
		interruption_Flag = 1;
		if (interruption_Cont == 1) {
			interruption_Cont = 0;
			Interruption_Printf();
		} else {
			interruption_Cont++;
		}
	}
}
void Varredura_Sensores() {
// Atualização dos Sensores
	for (int j = 1; j < 5; j++) {
		atualizar_leitura_angulo(j);
	}
}
void Varredura_Motores() {
// Atulizar os Motores com os valores de u[k] atualizados
	atualizar_motor_1(rho.u);
	atualizar_motor_2(theta1.u);
	atualizar_motor_3(theta2.u);
	atualizar_motor_4(theta3.u);
//	if (inicializar_programa == true) {
//		delay(1000000);
//		inicializar_programa = false;
//	}
}
void Interruption_Program() {
	GPIO_PortToggle(BOARD_INTERRUPTION_GPIO, 1U << BOARD_INTERRUPTION_PIN);
	Varredura_Sensores(); //  2.96ms
	References_Circle(&Circle); // 3.29ms
//	References_Saw(&Saw);
	Inverse_Kinematic(&Circle); // 5.92 - 6.20ms
	Pd_Control_Law(&Circle);
//	Open_Loop_Control_Law(&Circle); // 5.92 - 6.20ms
	Varredura_Motores(); //5.92 - 6.20ms
	GPIO_PortToggle(BOARD_INTERRUPTION_GPIO, 1U << BOARD_INTERRUPTION_PIN);

//	PD_References();
//	Pd_Control_Law();
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
//	atualizar_motores(4500, 4800, 3200, 6000);
//	delay(10000);

//	delay(10000000);

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
	float interruptin_freq = 1 / Ts;
	configurar_interrupcao_Hz(interruptin_freq);
	printf("\r\n Interrupção okay");
// Ficou em 3.855 Hz lendo os quatro sensores

	while (1) {
		if (interruption_Flag == 1) {
			Interruption_Program();
			interruption_Flag = 0;
		}
		Refresh_DAC();

//		atualizar_motores(teste_motor1, teste_motor2, teste_motor3,
//				teste_motor4);
	}
	return 0;
}
