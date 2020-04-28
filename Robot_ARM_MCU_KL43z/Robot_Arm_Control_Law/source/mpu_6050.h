#ifndef MPU_6050_H_
#define MPU_6050_H_

#include "fsl_i2c.h"
#define ACC_REG_WHOAMI 0x75
#define g 9.8
#define ACC_RESOLUTION (1.0f/16384.0f)
# define Ts 0.01
int cont = 0;
// Conversão de ângulo y para cima, x para a direita e z saindo da tela
// Giro pela regra da mão direita
typedef struct {
	uint8_t address;
	int I2C;

	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t temp;
	int16_t gx;
	int16_t gy;
	int16_t gz;

	int32_t AcelCUMx;
	int32_t AcelCUMy;
	int32_t AcelCUMz;

	int32_t GyroCUMx;
	int32_t GyroCUMy;
	int32_t GyroCUMz;

	int32_t posi_inicial_x;
	int32_t posi_inicial_y;
	int32_t posi_inicial_z;
	int32_t Acum_posi_inicial_x;
	int32_t Acum_posi_inicial_y;
	int32_t Acum_posi_inicial_z;

	// Média dos Valores durante a calibração do Acelerômetro
	int16_t M_ax;
	int16_t M_ay;
	int16_t M_az;

	// Média dos Valores durante a calibração do Giroscópio
	int16_t M_gx;
	int16_t M_gy;
	int16_t M_gz;
	// Valores dos Ângulos anteriores para cálculo da Integração
	float angGx_anterior;
	float angGy_anterior;
	float angGz_anterior;

	// Valores das posições anteriores
	float posi_aux_x;
	float posi_aux_y;
	float posi_aux_z;

	// Valores das Velocidades Ângulares do Giroscópio
	float v_ang_x;
	float v_ang_y;
	float v_ang_z;
	// Valores dos Ângulos atuais do Giroscópio
	float angGx_atual;
	float angGy_atual;
	float angGz_atual;
	// Valores dos Ângulos atuais do Acelerômetro
	float ang_pitch_x;
	float ang_pitch_y;
	float ang_pitch_z;

	float ang_roll_x;
	float ang_roll_y;
	float ang_roll_z;

	// Valores dos Ângulos atuais do Giroscópio
	float ang_fusao;

} acc_data_t;

static inline void ACC_Read(acc_data_t *acc, uint8_t reg, uint8_t *data,
		uint8_t size) {
// Selecionar a localização do escravo e do port de acordo com o numero do sensor
	i2c_master_transfer_t xfer = { .data = data, .dataSize = size, .direction =
			kI2C_Read, .slaveAddress = acc->address, .subaddress = reg,
			.subaddressSize = 1 };
	if (acc->I2C == I2C0_BASE)
		I2C_MasterTransferBlocking(I2C0, &xfer);
	else
		I2C_MasterTransferBlocking(I2C1, &xfer);

}

static inline void ACC_Write(acc_data_t *acc, uint8_t reg, uint8_t *data,
		uint8_t size) {
	i2c_master_transfer_t xfer = { .data = data, .dataSize = size, .direction =
			kI2C_Write, .slaveAddress = 0x68, .subaddress = reg,
			.subaddressSize = 1 };
	if (acc->I2C == I2C0_BASE)
		I2C_MasterTransferBlocking(I2C0, &xfer);
	else
		I2C_MasterTransferBlocking(I2C1, &xfer);
}

static inline void ACC_ReadRaw(acc_data_t *acc) {
	uint8_t leitura[14] = { 0 };
	uint8_t ready = 0;
	while ((ready && 1) != 1) {
		ACC_Read(acc, 0x3A, &ready, 1);
	}

	ACC_Read(acc, 0x3B, leitura, 14);
	acc->ax = (leitura[0] << 8) | (leitura[1]);
	acc->ay = (leitura[2] << 8) | (leitura[3]);
	acc->az = (leitura[4] << 8) | (leitura[5]);
	acc->temp = (leitura[6] << 8) | (leitura[7]);
	acc->gx = (leitura[8] << 8) | (leitura[9]);
	acc->gy = (leitura[10] << 8) | (leitura[11]);
	acc->gz = (leitura[12] << 8) | (leitura[13]);
	/*
	 if(cont<100){
	 data->OffsetX = ( data->OffsetX + (data->x));
	 data->OffsetY = ( data->OffsetY + (data->y));
	 data->OffsetZ = ( data->OffsetZ + (data->z));
	 cont++;
	 }
	 else{
	 data->OffsetX= (data->OffsetX)/cont;
	 data->OffsetY= (data->OffsetY)/cont;
	 data->OffsetZ= (data->OffsetZ)/cont;
	 }
	 */
}

static inline bool ACC_Init(acc_data_t *acc) {
	uint8_t data = 0; // iniciou com 0x40
	ACC_Read(acc, ACC_REG_WHOAMI, &data, 1u);
	if (data != acc->address)
		return false;
	//Acelerometro mede a resolução +-2g
	data = 0b00000000;
	ACC_Write(acc, 0x1C, &data, 1);

	//pequeno filtro digital
	data = 0b00000001;
	ACC_Write(acc, 0x1A, &data, 1);

	//liga interrupção dado pronto
	data = 0b00000001;
	ACC_Write(acc, 0x38, &data, 1);

	//DESLIGA GYRO
	//data = 0b00000111;
	data = 0b00000000;
	ACC_Write(acc, 0x6C, &data, 1);

	//configura Gyro em  ± 250 °/s
	data = 0b00000000;
	ACC_Write(acc, 0x1B, &data, 1);

	//Divisor de clock - amostragem
	//OR = 1Khz / (1+data)
	data = 24;
	ACC_Write(acc, 0x19, &data, 1);
	//

	//ACORDA
	data = 0b00000000;
	ACC_Write(acc, 0x6B, &data, 1);

	return true;
}
//void calibrar_sensor(acc_data_t *acc) {
//	int i = 0;
//	int Tcal = 500;
//	//int Tcal = 3; // para debugar
//	//Faz a leitura de todos os dados de giroscópio e acelerômetro
//	ACC_ReadRaw(acc);
//	// Cálculos do Acelerômetro
//	//Calcula as acelerações em relação ao eixo escolhido
//	float aceleracao_x = (acc->ax) * g * ACC_RESOLUTION;
//	float aceleracao_y = (acc->ay) * g * ACC_RESOLUTION;
//	float aceleracao_z = (acc->az) * g * ACC_RESOLUTION;
//	//Calcula velocidade em relação ao eixo escolhido
//	float velocidade_x = 0;
//	float velocidade_aux_x = 0;
//	float velocidade_y = 0;
//	float velocidade_aux_y = 0;
//	float velocidade_z = 0;
//	float velocidade_aux_z = 0;
//
//	velocidade_x = velocidade_aux_x + Ts * aceleracao_x;
//	velocidade_aux_x = velocidade_x;
//	velocidade_y = velocidade_aux_y + Ts * aceleracao_y;
//	velocidade_aux_y = velocidade_y;
//	velocidade_z = velocidade_aux_z + Ts * aceleracao_z;
//	velocidade_aux_z = velocidade_z;
//
//	//	calcula as posições em relação ao eixo escolhido
//	float posi_x = 0;
//	float posi_aux_x = 0;
//	float posi_y = 0;
//	float posi_aux_y = 0;
//	float posi_z = 0;
//	float posi_aux_z = 0;
//
//	posi_x = posi_aux_x + Ts * velocidade_x;
//	posi_aux_x = posi_x;
//	posi_y = posi_aux_y + Ts * velocidade_y;
//	posi_aux_y = posi_y;
//	posi_z = posi_aux_z + Ts * velocidade_z;
//	posi_aux_z = posi_z;
//
//	while (i < Tcal + 1) {
//		//inicia rotina de calibração
//		ACC_ReadRaw(acc);
//
//		if (i < Tcal) {
//			//acumula as variáveis do acelerômetro
//			//LSB Sensitivy = 16384 LSB/g
//			acc->AcelCUMx = acc->AcelCUMx + (acc->ax);
//			acc->AcelCUMy = acc->AcelCUMy + (acc->ay);
//			acc->AcelCUMz = acc->AcelCUMz + (acc->az - 16384);
//
//			// Acumula as posições do acelerômetro
//			acc->Acum_posi_inicial_x = posi_x + acc->Acum_posi_inicial_x;
//			acc->Acum_posi_inicial_y = posi_y + acc->Acum_posi_inicial_y;
//			acc->Acum_posi_inicial_z = posi_z + acc->Acum_posi_inicial_z;
//			//acumula as variáveis do giroscópio
//			acc->GyroCUMx = acc->GyroCUMx + acc->gx;
//			acc->GyroCUMy = acc->GyroCUMy + acc->gy;
//			acc->GyroCUMz = acc->GyroCUMz + acc->gz;
//			i++;
//		} else {
//			//calcula a media dos valores de aceleração acelerômetro
//			acc->M_ax = acc->AcelCUMx / i;
//			acc->M_ay = acc->AcelCUMy / i;
//			acc->M_az = acc->AcelCUMz / i;
//			// Calcula a média dos valores de posição do acelerômetro
//			acc->posi_inicial_x = acc->Acum_posi_inicial_x / i;
//			acc->posi_inicial_y = acc->Acum_posi_inicial_y / i;
//			acc->posi_inicial_z = acc->Acum_posi_inicial_z / i;
//			// Calcula a média dos valores do Giroscópio
//			acc->M_gx = acc->GyroCUMx / i;
//			acc->M_gy = acc->GyroCUMy / i;
//			acc->M_gz = acc->GyroCUMz / i;
//			i++;
//		}
//	}
//}
#endif /* MPU_6050_H_ */
