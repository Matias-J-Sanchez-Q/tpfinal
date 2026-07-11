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

/********************** inclusions *******************************************/
/* Project includes */
#include "main.h"

/* Demo includes */
#include "logger.h"
#include "dwt.h"

/* Application & Tasks includes */
#include "board.h"
#include "app.h"
#include "task_sensor_attribute.h"
#include "task_system_attribute.h"
#include "task_system_interface.h"

/********************** macros and definitions *******************************/

/*
 * Debounce timing (period = 1 ms/tick).
 * DEL_BTN_MAX = 50 -> ventana de antirrebote de 50 ms.
 */
#define DEL_BTN_MIN     0ul
#define DEL_BTN_MED     25ul
#define DEL_BTN_MAX     50ul

#define SENSOR_CFG_QTY  (sizeof(task_sensor_cfg_list) / sizeof(task_sensor_cfg_t))
#define SENSOR_DTA_QTY  SENSOR_CFG_QTY

/********************** internal data declaration ****************************/

/*
 * Mapeo de sensores logicos a pines fisicos (NUCLEO_F103RC).
 *
 * Los simbolos BTN_B_PORT/PIN/PRESSED y BTN_C_PORT/PIN/PRESSED ya estan
 * declarados en board.h.  El tercer boton (ID_BTN_D) reutiliza BTN_C_PORT
 * como alias hasta que se agregue BTN_D en board.h y el .ioc.
 *
 * Senales de task_system_attribute.h (ya declaradas en el proyecto):
 *   signal_down : EV_SYS_BUTTON | EV_SYS_CAMERA | EV_SYS_SENSOR_COIL
 *   signal_up   : EV_SYS_IDLE
 *
 *  ID             Port          Pin          Pressed        tick_max     signal_down          signal_up
 */
const task_sensor_cfg_t task_sensor_cfg_list[] =
{
	{ID_BTN_B, BTN_B_PORT, BTN_B_PIN, BTN_B_PRESSED, DEL_BTN_MAX, EV_SYS_BUTTON,      EV_SYS_IDLE},
	{ID_BTN_C, BTN_C_PORT, BTN_C_PIN, BTN_C_PRESSED, DEL_BTN_MAX, EV_SYS_CAMERA,      EV_SYS_IDLE},
	{ID_BTN_D, BTN_D_PORT, BTN_D_PIN, BTN_D_PRESSED, DEL_BTN_MAX, EV_SYS_SENSOR_COIL, EV_SYS_IDLE},
};

task_sensor_dta_t task_sensor_dta_list[SENSOR_DTA_QTY];

/********************** internal functions declaration ***********************/
void task_sensor_statechart(uint32_t index);

/********************** internal data definition *****************************/
const char *p_task_sensor    = "Task Sensor (Sensor Statechart)";
const char *p_task_sensor_   = "Non-Blocking Code";
const char *p_task_sensor__  = "(Update by Time Code, period = 1mS)";

/********************** external data declaration ****************************/

/********************** external functions definition ************************/

/* --------------------------------------------------------------------------
 * task_sensor_init
 * Inicializa cada sensor en ST_BTN_UP. Llamada una sola vez al arrancar.
 * -------------------------------------------------------------------------- */
void task_sensor_init(void *parameters)
{
	uint32_t index;
	task_sensor_dta_t *p_task_sensor_dta;
	task_sensor_st_t   state;
	task_sensor_ev_t   event;

	LOGGER_INFO(" ");
	LOGGER_INFO("  %s is running - Tick [mS] = %lu",
	            GET_NAME(task_sensor_init), HAL_GetTick());
	LOGGER_INFO("   %s is a %s", GET_NAME(task_sensor), p_task_sensor);
	LOGGER_INFO("   %s is a %s", GET_NAME(task_sensor), p_task_sensor_);
	LOGGER_INFO("   %s is a %s", GET_NAME(task_sensor), p_task_sensor__);

	for (index = 0; SENSOR_DTA_QTY > index; index++)
	{
		p_task_sensor_dta = &task_sensor_dta_list[index];

		state = ST_BTN_UP;
		p_task_sensor_dta->state = state;

		event = EV_BTN_UP;
		p_task_sensor_dta->event = event;

		p_task_sensor_dta->tick = DEL_BTN_MIN;

		LOGGER_INFO(" ");
		LOGGER_INFO("   %s = %lu   %s = %lu   %s = %lu",
		            GET_NAME(index), index,
		            GET_NAME(state), (uint32_t)state,
		            GET_NAME(event), (uint32_t)event);
	}
}

/* --------------------------------------------------------------------------
 * task_sensor_update
 * Llamada cada 1 ms por el scheduler. Ejecuta el statechart de cada sensor.
 * -------------------------------------------------------------------------- */
void task_sensor_update(void *parameters)
{
	uint32_t index;

	for (index = 0; SENSOR_DTA_QTY > index; index++)
	{
		task_sensor_statechart(index);
	}
}

/* --------------------------------------------------------------------------
 * task_sensor_statechart
 *
 * FSM de antirrebote de 4 estados (ver task_sensor.jpg):
 *
 *  ST_BTN_UP
 *    EV_BTN_DOWN  -> tick = DEL_BTN_MAX  -> ST_BTN_FALLING
 *
 *  ST_BTN_FALLING
 *    [tick > 0]             -> tick--              (interno)
 *    EV_BTN_UP  [tick==0]   -> ST_BTN_UP           (rebote, ignorado)
 *    EV_BTN_DOWN [tick==0]  -> raise signal_down
 *                           -> ST_BTN_DOWN          (pulsacion confirmada)
 *
 *  ST_BTN_DOWN
 *    EV_BTN_UP  -> tick = DEL_BTN_MAX  -> ST_BTN_RISING
 *
 *  ST_BTN_RISING
 *    [tick > 0]             -> tick--              (interno)
 *    EV_BTN_DOWN [tick==0]  -> ST_BTN_DOWN         (rebote, ignorado)
 *    EV_BTN_UP  [tick==0]   -> raise signal_up
 *                           -> ST_BTN_UP            (soltar confirmado)
 * -------------------------------------------------------------------------- */
void task_sensor_statechart(uint32_t index)
{
	const task_sensor_cfg_t *p_task_sensor_cfg;
	task_sensor_dta_t       *p_task_sensor_dta;

	p_task_sensor_cfg = &task_sensor_cfg_list[index];
	p_task_sensor_dta = &task_sensor_dta_list[index];

	/* Lectura del GPIO */
	if (p_task_sensor_cfg->pressed ==
	    HAL_GPIO_ReadPin(p_task_sensor_cfg->gpio_port, p_task_sensor_cfg->pin))
	{
		p_task_sensor_dta->event = EV_BTN_DOWN;
	}
	else
	{
		p_task_sensor_dta->event = EV_BTN_UP;
	}

	/* Maquina de estados */
	switch (p_task_sensor_dta->state)
	{
		case ST_BTN_UP:

			if (EV_BTN_DOWN == p_task_sensor_dta->event)
			{
				p_task_sensor_dta->tick  = DEL_BTN_MAX;
				p_task_sensor_dta->state = ST_BTN_FALLING;
				LOGGER_INFO("  [BTN %lu] ST_BTN_UP -> ST_BTN_FALLING (boton presionado)", index);
			}
			break;

		case ST_BTN_FALLING:

			if (DEL_BTN_MIN < p_task_sensor_dta->tick)
			{
				/* Interno: decrementar contador de antirrebote */
				p_task_sensor_dta->tick--;
			}
			else if (EV_BTN_UP == p_task_sensor_dta->event)
			{
				/* Rebote/glitch: volver a UP sin emitir evento */
				p_task_sensor_dta->state = ST_BTN_UP;
				LOGGER_INFO("  [BTN %lu] ST_BTN_FALLING -> ST_BTN_UP (rebote ignorado)", index);
			}
			else
			{
				/* EV_BTN_DOWN && tick==0: pulsacion confirmada */
				put_event_task_system(p_task_sensor_cfg->signal_down);
				p_task_sensor_dta->state = ST_BTN_DOWN;
				LOGGER_INFO("  [BTN %lu] ST_BTN_FALLING -> ST_BTN_DOWN (confirmada)", index);
			}
			break;

		case ST_BTN_DOWN:

			if (EV_BTN_UP == p_task_sensor_dta->event)
			{
				p_task_sensor_dta->tick  = DEL_BTN_MAX;
				p_task_sensor_dta->state = ST_BTN_RISING;
				LOGGER_INFO("  [BTN %lu] ST_BTN_DOWN -> ST_BTN_RISING (boton soltado)", index);
			}
			break;

		case ST_BTN_RISING:

			if (DEL_BTN_MIN < p_task_sensor_dta->tick)
			{
				/* Interno: decrementar contador de antirrebote */
				p_task_sensor_dta->tick--;
			}
			else if (EV_BTN_DOWN == p_task_sensor_dta->event)
			{
				/* Rebote/glitch: volver a DOWN sin emitir evento */
				p_task_sensor_dta->state = ST_BTN_DOWN;
				LOGGER_INFO("  [BTN %lu] ST_BTN_RISING -> ST_BTN_DOWN (rebote ignorado)", index);
			}
			else
			{
				/* EV_BTN_UP && tick==0: soltar confirmado */
				put_event_task_system(p_task_sensor_cfg->signal_up);
				p_task_sensor_dta->state = ST_BTN_UP;
				LOGGER_INFO("  [BTN %lu] ST_BTN_RISING -> ST_BTN_UP (confirmado)", index);
			}
			break;

		default:
			p_task_sensor_dta->tick  = DEL_BTN_MIN;
			p_task_sensor_dta->state = ST_BTN_UP;
			p_task_sensor_dta->event = EV_BTN_UP;
			break;
	}
}

/********************** end of file ******************************************/
