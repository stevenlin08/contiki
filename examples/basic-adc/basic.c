#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "button-sensor.h"
#include "adcin-sensor.h"
#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
  int value;

  PROCESS_BEGIN();
  SENSORS_ACTIVATE(button_right_sensor);
  SENSORS_ACTIVATE(adcin_sensor);
 printf("The sensor is: %u\n", leds_get());
PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);

    value = adcin_sensor.value(0);
    if(value != CC26XX_SENSOR_READING_ERROR) {
	 printf("The adcin val is: %u\n", value);
	}
  leds_toggle(LEDS_GREEN); 
 printf("The sensor is: %u\n", leds_get());
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/