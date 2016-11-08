/*---------------------------------------------------------------------------*/
/**
 * \addtogroup launchpad-peripherals
 * @{
 *
 * \file
 * Generic module controlling LaunchPad sensors
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "batmon-sensor.h"
#include "adc-sensor.h"
#include "eve003/button-sensor.h"
#include "eve003/switch-sensor.h"
//#include "eve003/adcin-sensor.h"
//#include "eve003/led_sensor.h"
#include <string.h>
/*---------------------------------------------------------------------------*/
/** \brief Exports a global symbol to be used by the sensor API */
//SENSORS(&button_left_sensor, &button_right_sensor ,&led_sensor);
SENSORS(&button_lock_sensor,&batmon_sensor, &adc_sensor ,&switch_sensor);

/*---------------------------------------------------------------------------*/
/** @} */
