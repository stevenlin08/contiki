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
#include "eve006/button-sensor.h"
#include "eve006/sensor_si7020.h"
//#include "eve006/mled_sensor.h"
#include "eve006/sensor_adxl345.h"
#include "eve006/switch_sensor.h"
#include <string.h>
/*---------------------------------------------------------------------------*/
/** \brief Exports a global symbol to be used by the sensor API */
SENSORS(&si7020_sensor,&adxl345_sensor,&switch_encode);
//SENSORS(&mled_sensor,&si7020_sensor,&adxl345_sensor,&switch_encode);
/*---------------------------------------------------------------------------*/
/** @} */
