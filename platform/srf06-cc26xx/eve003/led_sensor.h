/*---------------------------------------------------------------------------*/
/**
 * \addtogroup launchpad-peripherals
 * @{
 *
 * \defgroup launchpad-button-sensor LaunchPad Button Driver
 *
 * One of the buttons can be configured as general purpose or as an on/off key
 * @{
 *
 * \file
 * Header file for the LaunchPad Button Driver
 */
/*---------------------------------------------------------------------------*/
#ifndef LED_SENSOR_H_
#define LED_SENSOR_H_
/*---------------------------------------------------------------------------*/
#include "lib/sensors.h"

/*---------------------------------------------------------------------------*/
#define LED_SENSOR "LED"
/*---------------------------------------------------------------------------*/
#define	mFlashMode0	0
#define	mFlashMode1	1
#define	mFlashMode2	2

/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor led_sensor;

/*---------------------------------------------------------------------------*/
//uint32_t m_leds_init(void);
//uint32_t m_leds_flash(led_type_t p_type, led_color_t p_color, uint16_t p_times);


#endif /* BUTTON_SENSOR_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
