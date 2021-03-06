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
#include <GPIO.h>
#include <PushButton.h>
#include <RGB.h>

#define ADC_0 0
#define DELAY_ARG 600000
#define SICONV 0
#define NOCONV 1

uint16_t ADC_result(uint8_t ADC);
uint16_t conv(uint8_t convolucion_flag, uint16_t AudIn);

void main(void) {
	/**Variable to capture the input value*/
	uint32_t input_value_SW3 = 0;
	uint32_t input_value_SW2 = 0;

	//Valor de para el cambio de amplitud
	volatile float amplitud = 1;

	//Valor para cambio de convolución y no convolución
	uint8_t convolucion_flag = NOCONV;

	//Configuración de PushButtons
	PushButton_sw3_config();
	PushButton_sw2_config();

	//Variable para guardar la lectura del ADC
	volatile uint16_t AudIn;
	uint32_t Salida;
	uint8_t SalidaL;
	uint8_t SalidaH;

	//Configuración de clock;
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
	SIM->SCGC2 = 0x1000;

	//Configuración de RGB
	RGB_green_config();

	//Configuración ADC
	ADC0->CFG1 = ADC_CFG1_ADIV(0) | ADC_CFG1_ADLSMP_MASK |
	ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(0);
	ADC0->CFG2 = 0;
	ADC0->SC2 = 0;
	ADC0->SC3 = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3);

	//Configuración DAC
	DAC0->C0 = 0xC0;
	DAC0->DAT[0].DATL = 0;
	DAC0->DAT[0].DATH = 0;

	for (;;) {

		//Entrada del ADC
		AudIn = ADC_result(ADC_0);

		//Lectura de estado de push button
		input_value_SW3 = PushButton_read(PB_SW3);
		input_value_SW2 = PushButton_read(PB_SW2);

		//Validación de cual botón se tiene presionado ----------------------------------------------
		if (FALSE == input_value_SW3) {		//SW3, aumenta volumen
			delay(DELAY_ARG);				//Debouncer
			input_value_SW2 = PushButton_read(PB_SW2); //Valida si hay mas de un botón presionado
			if (FALSE == input_value_SW2) { //SW2
				delay(DELAY_ARG);			//Debouncer
				//si ambos sw están presionados, revisa el estado de la convolucion
				if (NOCONV == convolucion_flag) {
					convolucion_flag = SICONV;
				} else {
					convolucion_flag = NOCONV;
				}
			} else { //Si solo un sw está presionado
				if (amplitud == 0) {
					amplitud = 0.1;
				} else {
					amplitud = amplitud - 0.1;
				}
			}
		}

		//Validación de cual botón se tiene presionado ----------------------------------------------
		if (FALSE == input_value_SW2) { 	//SW2, baja volumen
			delay(DELAY_ARG);				//Debouncer
			input_value_SW3 = PushButton_read(PB_SW3);//Valida si hay mas de un botón presionado
			if (FALSE == input_value_SW3) {	//SW3
				delay(DELAY_ARG);			//Debouncer
				//si ambos sw están presionados, revisa el estado de la convolucion
				if (NOCONV == convolucion_flag) {
					convolucion_flag = SICONV;
				} else {
					convolucion_flag = NOCONV;
				}
			} else { //Si solo un sw está presionado
				amplitud = amplitud + 0.1;
			}
		}

		//Salida al DAC
		Salida = conv(convolucion_flag, AudIn) * (amplitud);
		SalidaL = (uint16_t) Salida & 0xFF;
		SalidaH = (uint16_t) ((Salida >> 8) & 0x0F);
		DAC0->DAT[0].DATL = SalidaL;
		DAC0->DAT[0].DATH = SalidaH;

	}

}

uint16_t ADC_result(uint8_t ADC) {
	uint16_t adc_result;
	ADC0->SC1[0] = ADC_SC1_ADCH(17); //PTE24 tarjeta, ADC0SE17 mapeado en 10001
	while ((ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0)
		;
	adc_result = ADC0->R[0];

	return (adc_result);
}

uint16_t conv(uint8_t convolucion_flag, uint16_t AudIn) {
	long double convolucion = 0;
	float resultArray[7] = { };
	uint32_t j = 0;
	float h[7] = { 0.07840464525404556, 0.17707825519483075,
			0.22014353249171387, 0.2759015644497544, 0.22014353249171387,
			0.17707825519483075, 0.07840464525404556 };

	if (SICONV == convolucion_flag) {	//SE REALIZA CONVOLUCIÓN
		//Llenado de arreglo
		for (uint8_t i = 6; i >= 1; i--) {
			resultArray[i] = resultArray[i - 1];
		}
		resultArray[0] = AudIn;

		//Operación de la convolución
		for (j = 0; j < 7; j++) {
			convolucion = (resultArray[j] * h[7 - j]);
			AudIn = (AudIn + (uint16_t) convolucion);
		}

		RGB_green_on();
		return AudIn;
	} else {	//SE REGRESA EL VALOR TAL CUAL ENTRA
		RGB_green_off();
		return AudIn;
	}
}

