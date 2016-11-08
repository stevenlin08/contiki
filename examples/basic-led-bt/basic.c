#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
  PROCESS_BEGIN();
  //SENSORS_ACTIVATE(button_right_sensor);
 printf("The sensor is: %u\n", leds_get());
//PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);
  leds_toggle(LEDS_GREEN); 
 printf("The sensor is: %u\n", leds_get());
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
