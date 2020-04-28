/*
 * perifericos.h
 *
 *  Created on: 14 de fev de 2020
 *      Author: davia
 */

#ifndef PERIFERICOS_H_
#define PERIFERICOS_H_

#include "fsl_dac.h"
#include "fsl_pit.h"
#include "fsl_tpm.h"
#include "fsl_i2c.h"

#define motor1_max 7300
#define motor1_min 1500
#define theta1_max 1.19693
#define theta1_min -1.257

#define motor2_max 4800
#define motor2_min 2000
#define theta2_max -0.562
#define theta2_min -1.972

#define motor3_max 5400
#define motor3_min 1100
#define theta3_max -1.544
#define theta3_min 1.558

#define motor4_max 5500
#define motor4_min 1200
#define theta4_max 1.06
#define theta4_min -2.92

#define motor1_a ((motor1_max - motor1_min) / (theta1_max - theta1_min))
#define motor1_b ((theta1_max*motor1_min - theta1_min*motor1_max)/(theta1_max - theta1_min))

#define ang_1_a ((theta1_max - theta1_min) / (motor1_max - motor1_min))
#define ang_1_b ((motor1_max*theta1_min - motor1_min*theta1_max)/(motor1_max - motor1_min))

#define motor2_a ((motor2_max - motor2_min) / (theta2_max - theta2_min))
#define motor2_b ((theta2_max*motor2_min - theta2_min*motor2_max)/(theta2_max - theta2_min))

#define motor3_a ((motor3_max - motor3_min) / (theta3_max - theta3_min))
#define motor3_b ((theta3_max*motor3_min - theta3_min*motor3_max)/(theta3_max - theta3_min))

#define motor4_a ((motor4_max - motor4_min) / (theta4_max - theta4_min))
#define motor4_b ((theta4_max*motor4_min - theta4_min*motor4_max)/(theta4_max - theta4_min))

void inicializar_dac(float tensao_max) {
	static dac_config_t DAC_CONFIG;
	DAC_CONFIG.enableLowPowerMode = false;
	DAC_CONFIG.referenceVoltageSource = kDAC_ReferenceVoltageSourceVref2; //Ver qua eh
	DAC_Init(DAC0, &DAC_CONFIG);
	DAC_Enable(DAC0, true);
	DAC_SetBufferReadPointer(DAC0, 0U);
	// Valor de tensão entre 0 e 3,3 V
	uint16_t dacValue = (tensao_max * 4096.0f) / 3.31f;
	DAC_SetBufferValue(DAC0, 0U, dacValue);
}
void atualizar_dac(uint16_t dacValue) {
	if (dacValue < 0)
		dacValue = 0;
	if (dacValue > 4096)
		dacValue = 4096;
	// Valor de 12 bits entre 0 e 4096
	DAC_SetBufferValue(DAC0, 0U, dacValue);
}
void configurar_interrupcao_Hz(float Hz) {
	uint32_t usec = 1000000.0 / Hz;
	//	TPM_SetTimerPeriod(TPM1, USEC_TO_COUNT(usec, CLOCK_GetBusClkFreq()));
	//	TPM_EnableInterrupts(TPM1, kTPM_Chnl0InterruptEnable);
	//	TPM_StartTimer(TPM1, kTPM_SystemClock);

	/* Structure of initialize PIT */
	pit_config_t pitConfig;

	/* Board pin, clock, debug console init */
	//BOARD_InitHardware();
	PIT_GetDefaultConfig(&pitConfig);

	/* Init pit module */
	PIT_Init(PIT, &pitConfig);

	/* Set timer period for channel 0 */
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0,
			USEC_TO_COUNT(usec, CLOCK_GetBusClkFreq()));

	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

	/* Enable at the NVIC */
	EnableIRQ(PIT_IRQn);

	/* Start channel 0 */
	PIT_StartTimer(PIT, kPIT_Chnl_0);

}
//void inicializar_PWMs() {
//	tpm_config_t MeuPWM = { .enableDebugMode = false, .enableDoze = false,
//			.enableReloadOnTrigger = false, .enableStartOnTrigger = false,
//			.enableStopOnOverflow = false, .prescale = kTPM_Prescale_Divide_8,
//			.useGlobalTimeBase =
//			false, };
//
//	//Inicializa TPM (Timer / PWM Module)
//	TPM_Init(TPM0, &MeuPWM);
//	//Configuração dos dois canais para o TPM2 (Vetor)
//	tpm_chnl_pwm_signal_param_t MeuCanalPWM1[4] = { { .chnlNumber = 0,
//			.dutyCyclePercent = 0, .level = kTPM_HighTrue }, { .chnlNumber = 1,
//			.dutyCyclePercent = 0, .level = kTPM_HighTrue }, { .chnlNumber = 2,
//			.dutyCyclePercent = 0, .level = kTPM_HighTrue }, { .chnlNumber = 3,
//			.dutyCyclePercent = 0, .level = kTPM_HighTrue } };
//
//	//Incializadao do PWM. Modulo TPM1, 2 Canais. Verificar que argumento 2 pode ser um vetor de estutura de canais (declarado acima)
//	TPM_SetupPwm(TPM0, MeuCanalPWM1, 4, kTPM_EdgeAlignedPwm, 50,
//			CLOCK_GetPeriphClkFreq());
////	aNTIGO = 24 mhZ MCGPCLK = 48mhz;
//// SE PRECISAR MEXER NA FREQUÊNCIA MUDAR A DIVISÃO DO PRESCALLER
//	//Incializadao do PWM. Modulo TPM0, 1 Canal.
//
//	TPM_StartTimer(TPM0, kTPM_SystemClock);
//
//}

void atualizar_motores(uint32_t teste_motor1, uint32_t teste_motor2,
		uint32_t teste_motor3, uint32_t teste_motor4) {
	TPM0_PERIPHERAL->CONTROLS[0].CnV = teste_motor1;
	TPM0_PERIPHERAL->CONTROLS[1].CnV = teste_motor2;
	TPM0_PERIPHERAL->CONTROLS[2].CnV = teste_motor3;
	TPM0_PERIPHERAL->CONTROLS[3].CnV = teste_motor3;
}
void inicializar_I2Cs() {

// Configuração Inicial de i2c0
	i2c_master_config_t I2C0_Config = { .baudRate_Bps = 400000, .enableMaster =
	true, .enableStopHold = false, };
	I2C_MasterInit(I2C0, &I2C0_Config, CLOCK_GetBusClkFreq());
// Configuração Inicial de i2c1
	i2c_master_config_t I2C1_Config = { .baudRate_Bps = 400000, .enableMaster =
	true, .enableStopHold = false, };
	I2C_MasterInit(I2C1, &I2C1_Config, CLOCK_GetBusClkFreq());

}
void delay(unsigned long n) {
	n = n * 10;
	while (n > 0)
		n--;
}
void atualizar_motor_1(float sinal_controle) {
	float valor_pwm = motor1_a * sinal_controle + motor1_b;
	TPM0_PERIPHERAL->CONTROLS[0].CnV = valor_pwm;
}

float motor_angulo_1() {
	float valor_pwm = TPM0_PERIPHERAL->CONTROLS[0].CnV;
	float valor_angulo = ang_1_a * valor_pwm + ang_1_b;
	return valor_angulo;
}

void atualizar_motor_2(float sinal_controle) {
	float valor_pwm = motor2_a * sinal_controle + motor2_b;
	TPM0_PERIPHERAL->CONTROLS[1].CnV = valor_pwm;
}
void atualizar_motor_3(float sinal_controle) {
	float valor_pwm = motor3_a * sinal_controle + motor3_b;
	TPM0_PERIPHERAL->CONTROLS[2].CnV = valor_pwm;
}
void atualizar_motor_4(float sinal_controle) {
	float valor_pwm = motor4_a * sinal_controle + motor4_b;
	TPM0_PERIPHERAL->CONTROLS[3].CnV = valor_pwm;
}
void selecionar_sensor(int sensor) {
	GPIO_PortSet(BOARD_Sensor_1_GPIO, 1U << BOARD_Sensor_1_PIN);
	GPIO_PortSet(BOARD_Sensor_2_GPIO, 1U << BOARD_Sensor_2_PIN);
	GPIO_PortSet(BOARD_Sensor_3_GPIO, 1U << BOARD_Sensor_3_PIN);
	GPIO_PortSet(BOARD_Sensor_4_GPIO, 1U << BOARD_Sensor_4_PIN);

	delay(10000);
	if (sensor == 1) {
		GPIO_PortClear(BOARD_Sensor_1_GPIO, 1U << BOARD_Sensor_1_PIN);
		GPIO_PortSet(BOARD_Sensor_2_GPIO, 1U << BOARD_Sensor_2_PIN);
		GPIO_PortSet(BOARD_Sensor_3_GPIO, 1U << BOARD_Sensor_3_PIN);
		GPIO_PortSet(BOARD_Sensor_4_GPIO, 1U << BOARD_Sensor_4_PIN);
	}
	if (sensor == 2) {
		GPIO_PortSet(BOARD_Sensor_1_GPIO, 1U << BOARD_Sensor_1_PIN);
		GPIO_PortClear(BOARD_Sensor_2_GPIO, 1U << BOARD_Sensor_2_PIN);
		GPIO_PortSet(BOARD_Sensor_3_GPIO, 1U << BOARD_Sensor_3_PIN);
		GPIO_PortSet(BOARD_Sensor_4_GPIO, 1U << BOARD_Sensor_4_PIN);
	}
	if (sensor == 3) {
		GPIO_PortSet(BOARD_Sensor_1_GPIO, 1U << BOARD_Sensor_1_PIN);
		GPIO_PortSet(BOARD_Sensor_2_GPIO, 1U << BOARD_Sensor_2_PIN);
		GPIO_PortClear(BOARD_Sensor_3_GPIO, 1U << BOARD_Sensor_3_PIN);
		GPIO_PortSet(BOARD_Sensor_4_GPIO, 1U << BOARD_Sensor_4_PIN);
	}
	if (sensor == 4) {
		GPIO_PortSet(BOARD_Sensor_1_GPIO, 1U << BOARD_Sensor_1_PIN);
		GPIO_PortSet(BOARD_Sensor_2_GPIO, 1U << BOARD_Sensor_2_PIN);
		GPIO_PortSet(BOARD_Sensor_3_GPIO, 1U << BOARD_Sensor_3_PIN);
		GPIO_PortClear(BOARD_Sensor_4_GPIO, 1U << BOARD_Sensor_4_PIN);
	}
	delay(10000);
}
#endif /* PERIFERICOS_H_ */
