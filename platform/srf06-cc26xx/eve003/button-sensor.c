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
#include "eve003/button-sensor.h"
#include "gpio-interrupt.h"
#include "sys/timer.h"
#include "lpm.h"

#include "ti-lib.h"

#include <stdint.h>


/*---------------------------------------------------------------------------*/
#ifdef BUTTON_SENSOR_CONF_ENABLE_SHUTDOWN
#define BUTTON_SENSOR_ENABLE_SHUTDOWN BUTTON_SENSOR_CONF_ENABLE_SHUTDOWN
#else
#define BUTTON_SENSOR_ENABLE_SHUTDOWN 1
#endif
/*---------------------------------------------------------------------------*/
#define BUTTON_GPIO_CFG         (IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO | \
                                 IOC_IOPULL_UP    | IOC_SLEW_DISABLE  | \
                                 IOC_HYST_DISABLE | IOC_BOTH_EDGES    | \
                                 IOC_INT_ENABLE   | IOC_IOMODE_NORMAL | \
                                 IOC_NO_WAKE_UP   | IOC_INPUT_ENABLE)
/*---------------------------------------------------------------------------*/
#define DEBOUNCE_DURATION (CLOCK_SECOND >> 5)

struct btn_timer {
  struct timer debounce;
  clock_time_t start;
  clock_time_t duration;
};

static struct btn_timer lock_timer, urgent_timer, setup_timer, reset_timer, zbp_timer;
/*---------------------------------------------------------------------------*/
static void
button_press_handler(uint8_t ioid)
{
//------------------------------------------------------
  if(ioid == BOARD_IOID_KEY_LOCK) {
    if(!timer_expired(&lock_timer.debounce)) {
      return;
    }

    timer_set(&lock_timer.debounce, DEBOUNCE_DURATION);

    // Start press duration counter on press (falling), notify on release (rising)

    if(ti_lib_gpio_read_dio(BOARD_IOID_KEY_LOCK) == 0) {
      lock_timer.start = clock_time();
      lock_timer.duration = 0;
    } else {
      lock_timer.duration = clock_time() - lock_timer.start;
      sensors_changed(&button_lock_sensor); // --> notify on release
    }
  }
//------------------------------------------------------
  if(ioid == BOARD_IOID_KEY_URGENT) {
    if(!timer_expired(&urgent_timer.debounce)) {
      return;
    }

    timer_set(&urgent_timer.debounce, DEBOUNCE_DURATION);

    // Start press duration counter on press (falling), notify on release (rising) 
  
    if(ti_lib_gpio_read_dio(BOARD_IOID_KEY_URGENT) == 0) {
      urgent_timer.start = clock_time();
      urgent_timer.duration = 0;
    } else {
      urgent_timer.duration = clock_time() - urgent_timer.start;
      sensors_changed(&button_urgent_sensor); // --> notify on release
    }
  }
//------------------------------------------------------
  if(ioid == BOARD_IOID_KEY_SETUP) {
    if(!timer_expired(&setup_timer.debounce)) {
      return;
    }

    timer_set(&setup_timer.debounce, DEBOUNCE_DURATION);

    // Start press duration counter on press (falling), notify on release (rising) 
  
    if(ti_lib_gpio_read_dio(BOARD_IOID_KEY_SETUP) == 0) {
      setup_timer.start = clock_time();
      setup_timer.duration = 0;
    } else {
      setup_timer.duration = clock_time() - setup_timer.start;
      sensors_changed(&button_setup_sensor); // --> notify on release
    }
  }
//------------------------------------------------------
  if(ioid == BOARD_IOID_KEY_RESET) {
    if(!timer_expired(&reset_timer.debounce)) {
      return;
    }

    timer_set(&reset_timer.debounce, DEBOUNCE_DURATION);

    // Start press duration counter on press (falling), notify on release (rising) 
  
    if(ti_lib_gpio_read_dio(BOARD_IOID_KEY_RESET) == 0) {
      reset_timer.start = clock_time();
      reset_timer.duration = 0;
    } else {
      reset_timer.duration = clock_time() - reset_timer.start;
      sensors_changed(&button_reset_sensor); // --> notify on release
    }
  }
//------------------------------------------------------
  if(ioid == BOARD_IOID_KEY_ZBP) {
    if(!timer_expired(&zbp_timer.debounce)) {
      return;
    }

    timer_set(&zbp_timer.debounce, DEBOUNCE_DURATION);

    // Start press duration counter on press (falling), notify on release (rising) 
  
    if(ti_lib_gpio_read_dio(BOARD_IOID_KEY_ZBP) == 0) {
      zbp_timer.start = clock_time();
      zbp_timer.duration = 0;
    } else {
      zbp_timer.duration = clock_time() - zbp_timer.start;
      sensors_changed(&button_zbp_sensor); // --> notify on release
    }
  }
//------------------------------------------------------
/*
  if(ioid == BOARD_IOID_KEY_RIGHT) {
    if(BUTTON_SENSOR_ENABLE_SHUTDOWN == 0) {
      if(!timer_expired(&right_timer.debounce)) {
        return;
      }

      timer_set(&right_timer.debounce, DEBOUNCE_DURATION);

      // Start press duration counter on press (falling), notify on release (rising)
 
      if(ti_lib_gpio_read_dio(BOARD_IOID_KEY_RIGHT) == 0) {
        right_timer.start = clock_time();
        right_timer.duration = 0;
      } else {
        right_timer.duration = clock_time() - right_timer.start;
        sensors_changed(&button_right_sensor);
      }
    } else {
      lpm_shutdown(BOARD_IOID_KEY_RIGHT, IOC_IOPULL_UP, IOC_WAKE_ON_LOW);
    }
  }
*/
}

/*---------------------------------------------------------------------------*/
static void
config_buttons(int type, int c, uint32_t key)
{
  switch(type) {
  case SENSORS_HW_INIT:
    ti_lib_gpio_clear_event_dio(key);
    ti_lib_rom_ioc_pin_type_gpio_input(key);
    ti_lib_rom_ioc_port_configure_set(key, IOC_PORT_GPIO, BUTTON_GPIO_CFG);
    gpio_interrupt_register_handler(key, button_press_handler);
    break;
  case SENSORS_ACTIVE:
    if(c) {
      ti_lib_gpio_clear_event_dio(key);
      ti_lib_rom_ioc_pin_type_gpio_input(key);
      ti_lib_rom_ioc_port_configure_set(key, IOC_PORT_GPIO, BUTTON_GPIO_CFG);
      ti_lib_rom_ioc_int_enable(key);
    } else {
      ti_lib_rom_ioc_int_disable(key);
    }
    break;
  default:
    break;
  }
}

/*---------------------------------------------------------------------------*/
static int
status(int type, uint32_t key_io_id)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    if(ti_lib_rom_ioc_port_configure_get(key_io_id) & IOC_INT_ENABLE) {
      return 1;
    }
    break;
  default:
    break;
  }
  return 0;
}


//=============================================================================
// KEY LOCK                                                                  
//=============================================================================
/*---------------------------------------------------------------------------*/
static int
config_lock(int type, int value)
{
  config_buttons(type, value, BOARD_IOID_KEY_LOCK);

  return 1;
}

/*---------------------------------------------------------------------------*/
static int
value_lock(int type)
{
  if(type == BUTTON_SENSOR_VALUE_STATE) {
    return ti_lib_gpio_read_dio(BOARD_IOID_KEY_LOCK) == 0 ?
           BUTTON_SENSOR_VALUE_PRESSED : BUTTON_SENSOR_VALUE_RELEASED;
  } else if(type == BUTTON_SENSOR_VALUE_DURATION) {
    return (int)lock_timer.duration;
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
static int
status_lock(int type)
{
  return status(type, BOARD_IOID_KEY_LOCK);
}

//=============================================================================
// KEY URGENT                                                               
//=============================================================================
/*---------------------------------------------------------------------------*/
static int
config_urgent(int type, int value)
{
  config_buttons(type, value, BOARD_IOID_KEY_URGENT);

  return 1;
}

/*---------------------------------------------------------------------------*/
static int
value_urgent(int type)
{
  if(type == BUTTON_SENSOR_VALUE_STATE) {
    return ti_lib_gpio_read_dio(BOARD_IOID_KEY_URGENT) == 0 ?
           BUTTON_SENSOR_VALUE_PRESSED : BUTTON_SENSOR_VALUE_RELEASED;
  } else if(type == BUTTON_SENSOR_VALUE_DURATION) {
    return (int)lock_timer.duration;
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
static int
status_urgent(int type)
{
  return status(type, BOARD_IOID_KEY_URGENT);
}

//=============================================================================
// KEY SETUP                                                               
//=============================================================================
/*---------------------------------------------------------------------------*/
static int
config_setup(int type, int value)
{
  config_buttons(type, value, BOARD_IOID_KEY_SETUP);

  return 1;
}

/*---------------------------------------------------------------------------*/
static int
value_setup(int type)
{
  if(type == BUTTON_SENSOR_VALUE_STATE) {
    return ti_lib_gpio_read_dio(BOARD_IOID_KEY_SETUP) == 0 ?
           BUTTON_SENSOR_VALUE_PRESSED : BUTTON_SENSOR_VALUE_RELEASED;
  } else if(type == BUTTON_SENSOR_VALUE_DURATION) {
    return (int)lock_timer.duration;
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
static int
status_setup(int type)
{
  return status(type, BOARD_IOID_KEY_SETUP);
}



//=============================================================================
// KEY ZBP                                                               
//=============================================================================
/*---------------------------------------------------------------------------*/
static int
config_zbp(int type, int value)
{
  config_buttons(type, value, BOARD_IOID_KEY_ZBP);

  return 1;
}

/*---------------------------------------------------------------------------*/
static int
value_zbp(int type)
{
  if(type == BUTTON_SENSOR_VALUE_STATE) {
    return ti_lib_gpio_read_dio(BOARD_IOID_KEY_ZBP) == 0 ?
           BUTTON_SENSOR_VALUE_PRESSED : BUTTON_SENSOR_VALUE_RELEASED;
  } else if(type == BUTTON_SENSOR_VALUE_DURATION) {
    return (int)lock_timer.duration;
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
static int
status_zbp(int type)
{
  return status(type, BOARD_IOID_KEY_ZBP);
}

//=============================================================================
// KEY RESET                                                               
//=============================================================================
/*---------------------------------------------------------------------------*/
static int
config_reset(int type, int value)
{
  config_buttons(type, value, BOARD_IOID_KEY_RESET );

  return 1;
}

/*---------------------------------------------------------------------------*/
static int
value_reset(int type)
{
  if(type == BUTTON_SENSOR_VALUE_STATE) {
    return ti_lib_gpio_read_dio(BOARD_IOID_KEY_RESET) == 0 ?
           BUTTON_SENSOR_VALUE_PRESSED : BUTTON_SENSOR_VALUE_RELEASED;
  } else if(type == BUTTON_SENSOR_VALUE_DURATION) {
    return (int)lock_timer.duration;
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
static int
status_reset(int type)
{
  return status(type, BOARD_IOID_KEY_RESET);
}



/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(button_lock_sensor,   BUTTON_SENSOR, value_lock   , config_lock  ,status_lock  );
SENSORS_SENSOR(button_urgent_sensor, BUTTON_SENSOR, value_urgent , config_urgent,status_urgent);
SENSORS_SENSOR(button_setup_sensor,  BUTTON_SENSOR, value_setup  , config_setup ,status_setup );
SENSORS_SENSOR(button_zbp_sensor,    BUTTON_SENSOR, value_zbp    , config_zbp   ,status_zbp   );
SENSORS_SENSOR(button_reset_sensor,  BUTTON_SENSOR, value_reset  , config_reset ,status_reset );







/*---------------------------------------------------------------------------*/
/** @} */
