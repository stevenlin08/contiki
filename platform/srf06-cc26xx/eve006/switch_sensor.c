#include "contiki.h"
#include "lib/sensors.h"
#include "eve006/switch_sensor.h"
#include "gpio-interrupt.h"
#include "sys/timer.h"
#include "lpm.h"
#include "sys/ctimer.h"
#include "sys/clock.h"
#include "ti-lib.h"

#include <stdint.h>
#include <stdio.h>

/**
 * \brief Configuration function for the switch sensor.
 *
 * \param type Activate, enable or disable the sensor. See below
 * \param enable
 *
 * When type == SENSORS_HW_INIT we turn on the hardware
 * When type == SENSORS_ACTIVE and enable==1 we enable the sensor
 * When type == SENSORS_ACTIVE and enable==0 we disable the sensor
 */
/*---------------------------------------------------------------------------*/
#define SWITCH_GPIO_CFG         (IOC_CURRENT_2MA | IOC_STRENGTH_AUTO |      \
                                 IOC_NO_IOPULL | IOC_SLEW_DISABLE |         \
                                 IOC_HYST_DISABLE | IOC_NO_EDGE |           \
                                 IOC_INT_DISABLE | IOC_IOMODE_NORMAL |      \
                                 IOC_NO_WAKE_UP | IOC_INPUT_ENABLE )

#define SENSOR_SWITCH_STARTUP_DELAY 3

#define SENSOR_STATUS_DISABLED     0
#define SENSOR_STATUS_INITIALISED  1
#define SENSOR_STATUS_NOT_READY    2
#define SENSOR_STATUS_READY        3
/*---------------------------------------------------------------------------*/

static int enabled = SENSOR_STATUS_DISABLED;
static struct ctimer startup_switch_timer;
static int encode1_value=0;
/*---------------------------------------------------------------------------*/
static int sensor_switch_encode1(void)
{
	int keys=0;

    if(((int)ti_lib_gpio_read_dio(BOARD_IOID_KEY_SW0)) == 1) 
	{
        keys |= 0x1;
    }
    if(((int)ti_lib_gpio_read_dio(BOARD_IOID_KEY_SW1)) == 1) 
	{
        keys |= 0x2;
    }
    if(((int)ti_lib_gpio_read_dio(BOARD_IOID_KEY_SW2)) == 1) 
	{
        keys |= 0x4;
    }
    if(((int)ti_lib_gpio_read_dio(BOARD_IOID_KEY_SW3)) == 1) 
	{
        keys |= 0x8;
    }	
    if(((int)ti_lib_gpio_read_dio(BOARD_IOID_KEY_SW4)) == 1) 
	{
        keys |= 0x10;
    }	
    if(((int)ti_lib_gpio_read_dio(BOARD_IOID_KEY_SW5)) == 1) 
	{
        keys |= 0x20;
    }	
    if(((int)ti_lib_gpio_read_dio(BOARD_IOID_KEY_SW6)) == 1) 
	{
        keys |= 0x40;
    }	
    if(((int)ti_lib_gpio_read_dio(BOARD_IOID_KEY_SW7)) == 1) 
	{
        keys |= 0x80;
    }	
	//printf("encode1 %d\n\r",keys);
	return (keys);
}
/*---------------------------------------------------------------------------*/

static void notify_ready(void *not_used)
{
  encode1_value = sensor_switch_encode1();
 
  enabled = SENSOR_STATUS_READY;
  sensors_changed(&switch_encode);
}

/*---------------------------------------------------------------------------*/
static void configure_switch_input(uint32_t key)
{
    ti_lib_gpio_clear_event_dio(key);
    ti_lib_rom_ioc_pin_type_gpio_input(key);
    ti_lib_rom_ioc_port_configure_set(key, IOC_PORT_GPIO, SWITCH_GPIO_CFG);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the switch encode1.
*/
static void
configure_encode(void)
{
  printf("configure_encode...............................\n\r");
  configure_switch_input(BOARD_IOID_KEY_SW0);
  configure_switch_input(BOARD_IOID_KEY_SW1);
  configure_switch_input(BOARD_IOID_KEY_SW2);
  configure_switch_input(BOARD_IOID_KEY_SW3);
  configure_switch_input(BOARD_IOID_KEY_SW4);
  configure_switch_input(BOARD_IOID_KEY_SW5);
  configure_switch_input(BOARD_IOID_KEY_SW6);
  configure_switch_input(BOARD_IOID_KEY_SW7); 
}

/*---------------------------------------------------------------------------*/
//static int config_switch(int type, int c, uint32_t key)
static int config_switch(int type, int enable)
{
  switch(type) {
  case SENSORS_HW_INIT:

	enabled = SENSOR_STATUS_INITIALISED;
	configure_encode();
    break;
  case SENSORS_ACTIVE:
    /* Must be initialised first */
    if(enable) 
	{
		//configure_encode();
		enabled = SENSOR_STATUS_NOT_READY;
		ctimer_set(&startup_switch_timer,SENSOR_SWITCH_STARTUP_DELAY, notify_ready, NULL);
    } 
	else 
	{

    }
    break;
  default:
    break;
  }
  return enabled;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the status of the sensor
 * \param type SENSORS_ACTIVE or SENSORS_READY
 * \return 1 if the sensor is enabled
 */
static int status_switch(int type)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \param type BMP_280_SENSOR_TYPE_TEMP or BMP_280_SENSOR_TYPE_PRESS
 * \return Temperature (centi degrees C) or Pressure (Pascal).
 */
static int value_switch(int type)
{
  int rv;
  //int32_t temp = 0;
  //uint32_t pres = 0;

	if(enabled != SENSOR_STATUS_READY) {
		printf("Sensor disabled or starting up (%d)\n", enabled);
		return CC26XX_SENSOR_READING_ERROR;
	}

	if((type != SWITCH_SENSOR_TYPE_ENCODE1) && (type != SWITCH_SENSOR_TYPE_ENCODE2)) {
		printf("Invalid type\n");
		return CC26XX_SENSOR_READING_ERROR;
	} 
	else 
	{
		if(type == SWITCH_SENSOR_TYPE_ENCODE1) {
		  	rv = encode1_value;
		} 
	}
  return rv;
}

/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(switch_encode, "SWITCH", value_switch, config_switch, status_switch);
/*---------------------------------------------------------------------------*/
