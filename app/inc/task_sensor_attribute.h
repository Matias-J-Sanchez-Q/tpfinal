/*
 * Copyright (c) 2026 Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @author : Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>
 */

#ifndef TASK_SENSOR_ATTRIBUTE_H_
#define TASK_SENSOR_ATTRIBUTE_H_

/********************** CPP guard ********************************************/
#ifdef __cplusplus
extern "C" {
#endif

/********************** inclusions *******************************************/
#include "task_system_attribute.h"

/********************** macros ***********************************************/

/********************** typedef **********************************************/

/* Eventos que excitan el Task Sensor */
typedef enum task_sensor_ev
{
	EV_BTN_UP,
	EV_BTN_DOWN
} task_sensor_ev_t;

/* Estados del Task Sensor */
typedef enum task_sensor_st
{
	ST_BTN_UP,
	ST_BTN_FALLING,
	ST_BTN_DOWN,
	ST_BTN_RISING
} task_sensor_st_t;

/*
 * Identificadores de sensores logicos.
 *
 * Mapeo a pines fisicos (pines D0..D15 del .ioc):
 *   ID_BTN_B  -> BTN_B (GPIO_Input pull-up, e.g. PA1  / CN8 pin 2)
 *   ID_BTN_C  -> BTN_C (GPIO_Input pull-up, e.g. PA4  / CN8 pin 5)
 *   ID_BTN_D  -> BTN_D (GPIO_Input pull-up, pin elegido en el .ioc)
 *
 * Todos activos en LOW (BTN_x_PRESSED = GPIO_PIN_RESET).
 */
typedef enum task_sensor_id
{
	ID_BTN_B,
	ID_BTN_C,
	ID_BTN_D
} task_sensor_id_t;

/* Registro de configuracion (ROM) - uno por boton fisico */
typedef struct
{
	task_sensor_id_t	identifier;
	GPIO_TypeDef *		gpio_port;
	uint16_t			pin;
	GPIO_PinState		pressed;       /* Nivel logico considerado "presionado" */
	uint32_t			tick_max;      /* Ticks de antirrebote                 */
	task_system_ev_t	signal_down;   /* Evento enviado al presionar           */
	task_system_ev_t	signal_up;     /* Evento enviado al soltar              */
} task_sensor_cfg_t;

/* Datos dinamicos (RAM) - uno por boton fisico */
typedef struct
{
	uint32_t			tick;
	task_sensor_st_t	state;
	task_sensor_ev_t	event;
} task_sensor_dta_t;

/********************** external data declaration ****************************/
extern task_sensor_dta_t task_sensor_dta_list[];

/********************** external functions declaration ***********************/

/********************** End of CPP guard *************************************/
#ifdef __cplusplus
}
#endif

#endif /* TASK_SENSOR_ATTRIBUTE_H_ */

/********************** end of file ******************************************/
