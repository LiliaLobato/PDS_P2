/*
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    Practica_2_PDS.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "MK64F12.h"
#include <stdint.h>
static const uint32_t pc6_sw2 = 0x40; //6
#define FALSE 0
#define ADC_0 0
#define ADC_1 1

uint16_t ADC_result(uint8_t ADC);
void delay(uint16_t delay);
uint32_t sw2_pressed();
uint32_t sw3_pressed();

void main(void) {
	volatile uint16_t AudIn;
	uint32_t Salida;
	uint8_t SalidaL;
	uint8_t SalidaH;

	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; //habilitar reloj a ADC
	SIM->SCGC5 |= 0xA00; //habilitar reloj a botones
	SIM->SCGC2 |= 0x1000; //habilitar reloj DAC

	// Configuracion del clock del ADC1
	SIM->SCGC3 |= SIM_SCGC3_ADC1_MASK;

	PORTC->PCR[6] = 0x100 | 0x03; //habilitar resistencia pull up botón SW2
	//	PORTA->PCR[4] = 0x100 | 0x03; //habilitar resistencia pull up botón

	GPIOC->PDDR &= ~0x40; //habilitar botón como entrada SW2
	//	GPIOA->PDDR &= ~0x10; //habilitar botón como entrada

	//	Inicialización del ADC0
	ADC0->CFG1 = ADC_CFG1_ADIV(
			0)|ADC_CFG1_ADLSMP_MASK |ADC_CFG1_MODE(1)|ADC_CFG1_ADICLK(0); //ADC_CFG1_ADIV(en 0) para no promediar ADC_CFG1_MODE(en 1) para 12 bits
	ADC0->CFG2 = 0;
	ADC0->SC2 = 0;
	ADC0->SC3 = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3);

	//	Inicialización del DAC0
	SIM->SCGC2 = 0x1000;
	DAC0->C0 = 0xC0;
	DAC0->DAT[0].DATL = 0;
	DAC0->DAT[0].DATH = 0;

	for (;;) {
		AudIn = ADC_result(ADC_0);
		Salida = AudIn;

		SalidaL = (uint16_t) Salida & 0xFF;
		SalidaH = (uint16_t) ((Salida >> 8) & 0x0F);

		DAC0->DAT[0].DATL = SalidaL;
		DAC0->DAT[0].DATH = SalidaH;

	}

}


uint16_t ADC_result(uint8_t ADC) {
	uint16_t adc_result;
	ADC0->SC1[0] = ADC_SC1_ADCH(17);//PTE24 tarjeta, ADC0SE17 mapeado en 10001
	while ((ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0)
		;
	adc_result = ADC0->R[0];

	return (adc_result);
}
