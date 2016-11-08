/*
 * Copyright (c) 2015, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup launchpad-button-sensor
 * @{
 *
 * \file
 * Driver for LaunchPad buttons
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/sensors.h"
#include "eve003/switch-sensor.h"
#include "sys/timer.h"
#include "lpm.h"

#include "ti-lib.h"

#include <stdint.h>

/*---------------------------------------------------------------------------*/
#define SWITCH_GPIO_CFG         (IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO | \
                                 IOC_IOPULL_UP    | IOC_SLEW_DISABLE  | \
                                 IOC_HYST_DISABLE | IOC_BOTH_EDGES    | \
                                 IOC_INT_ENABLE   | IOC_IOMODE_NORMAL | \
                                 IOC_NO_WAKE_UP   | IOC_INPUT_ENABLE)

//=============================================================================
//                                                                   
//=============================================================================
void init_switch_io (void)
{
  ti_lib_rom_ioc_pin_type_gpio_input(BOARD_IOID_SW_S1);
  ti_lib_rom_ioc_port_configure_set(BOARD_IOID_SW_S1, IOC_PORT_GPIO, SWITCH_GPIO_CFG);

  ti_lib_rom_ioc_pin_type_gpio_input(BOARD_IOID_SW_S2);
  ti_lib_rom_ioc_port_configure_set(BOARD_IOID_SW_S2, IOC_PORT_GPIO, SWITCH_GPIO_CFG);

  ti_lib_rom_ioc_pin_type_gpio_input(BOARD_IOID_SW_S3);
  ti_lib_rom_ioc_port_configure_set(BOARD_IOID_SW_S3, IOC_PORT_GPIO, SWITCH_GPIO_CFG);
}

//------------------------------------------------------------------------------
static int get_switch_stat (void)
{
  int stat;
  
  stat = 0;
  if (ti_lib_gpio_read_dio(BOARD_IOID_SW_S1) == 0)
    stat |= 1;
  if (ti_lib_gpio_read_dio(BOARD_IOID_SW_S2) == 0)
    stat |= 2;
  if (ti_lib_gpio_read_dio(BOARD_IOID_SW_S3) == 0)
    stat |= 4; 

  return stat;
}


//=============================================================================
// SWITCH SENSOR                                                               
//=============================================================================
/*---------------------------------------------------------------------------*/
static int
config_switch(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
    init_switch_io();
    break;

  case SENSORS_ACTIVE:
    init_switch_io();
    break;
  default:
    break;
  }

  return 1;
}

/*---------------------------------------------------------------------------*/
static int
value_switch(int type)
{
  if (type == GET_SWITCH_STATUS)
    return get_switch_stat();

  return 0;
}

/*---------------------------------------------------------------------------*/
static int
status_switch(int type)
{
  return get_switch_stat();
}


/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(switch_sensor,SWITCH_SENSOR,value_switch,config_switch,status_switch);








/*---------------------------------------------------------------------------*/
/** @} */
