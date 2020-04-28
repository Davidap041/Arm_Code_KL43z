/*
 * Control_Law.h
 *
 *  Created on: 5 de mar de 2020
 *      Author: davia
 */

#ifndef CONTROL_LAW_H_
#define CONTROL_LAW_H_

typedef struct {
//	Variáveis de Controle PD
	int identification;

	float kp_pd;
	float kv_pd;
	float m;
	float b;
	float k;
	float u;
	float means;

	float erro_atual;
	float erro_derivadas;

	float pd_pp1;
	float pd_pp2;

	float Th; // Ângulo do servo atual
	float Th_anterior; // Ângulo do servo da amostra passada

	float dTh; 	// Diferencial do Ângulo do Servo
	float dTh_ref;
	float Th_ref; // Referências do Ângulo do Servo;
	float Th_ref_anterior; // Referências do Ângulo do Servo;
//	Variáveis do Saturador
	float upper_limit;
	float lower_limit;

} Angle;

void Control_Signal(Angle *elo) {
	elo->erro_atual = (elo->Th_ref - elo->Th);
	elo->erro_derivadas = (elo->dTh_ref - elo->dTh);

	elo->pd_pp1 = (elo->k * elo->Th) + (elo->b * elo->dTh);
	elo->pd_pp2 = elo->m
			* (elo->kp_pd * elo->erro_atual + elo->kv_pd * elo->erro_derivadas);

	elo->u = elo->pd_pp1 + elo->pd_pp2 + elo->means;

//	elo->u = elo->u / 10;
	// Implementação dos saturadores saturadores
//	if (elo->u > elo->upper_limit) {
//		elo->u = elo->upper_limit;
//	}
//	if (elo->u < elo->lower_limit) {
//		elo->u = elo->lower_limit;
//	}

//	printf("/r/nControl Signal: %f", elo->u);
}

#endif /* CONTROL_LAW_H_ */
