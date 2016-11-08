#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include <stdio.h> /* For printf() */
/* The event timer library */
#include "sys/etimer.h"

/* The seconds timer library */
#include "sys/stimer.h"

/* The regular timer library */
#include "sys/timer.h"

/* The callback timer library */
#include "sys/ctimer.h"

/* The "real-time" timer library */
#include "sys/rtimer.h"

static struct timer  nt;
static struct stimer st;
static struct etimer et;
static struct ctimer ct;
static struct rtimer rt;
/*---------------------------------------------------------------------------*/
PROCESS(test_timers_process, "Timers example  process");
AUTOSTART_PROCESSES(&test_timers_process);
static void
rtimer_callback_example(struct rtimer *timer, void *ptr)
{
  uint32_t *rtimer_ticks = ptr;
  printf("rtimer, now: \t%ld\n", *rtimer_ticks);
 leds_toggle(LEDS_RED);
  /* We can restart the ctimer and keep the counting going */
  (*rtimer_ticks)++;
  ctimer_restart(&ct);
}

static void
ctimer_callback_example(void *ptr)
{
  uint32_t *ctimer_ticks = ptr;
  printf("ctimer, now: \t%ld\n", *ctimer_ticks);

  /* The real timer allows execution of real-time tasks (with predictable
   * execution times).
   * The function RTIMER_NOW() is used to get the current system time in ticks
   * and RTIMER_SECOND specifies the number of ticks per second.
   */
 leds_toggle(LEDS_GREEN);
  (*ctimer_ticks)++;
  rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND, 0,
             rtimer_callback_example, ctimer_ticks);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(test_timers_process, ev, data)
{
  PROCESS_BEGIN();
 static uint32_t ticks = 0;
  /* This is the most basic timer, it doesn't trigger any expiration event so
   * you need to check if it has expired.  The "CLOCK_SECOND" constant is
   * platform-dependant, and corresponds to the number of "ticks" in a second.
   * For periods less than a second, use this timer
   */
  timer_set(&nt, CLOCK_SECOND);

  /* To test the timer we increase the "ticks" variable each time the code
   * inside the loop, the "timer_expired(...)" function checks if the timer has
   * expired.
   */
  while(!timer_expired(&nt)) {
    ticks++;
  }

printf("timer, ticks: \t%ld\n", ticks);
 leds_toggle(LEDS_GREEN);
  /* And we keep the process halt while we wait for the callback timer to
   * expire.
   */
  /* The seconds timer already uses seconds as a time base, so there's no need
   * to use the "CLOCK_SECOND" constant.
   */
  stimer_set(&st, 1);

  ticks = 0;

  while(!stimer_expired(&st)) {
    ticks++;
  }

  printf("stimer, ticks: \t%ld\n", ticks);
 leds_toggle(LEDS_GREEN);

  ticks = 0;
  etimer_set(&et, CLOCK_SECOND * 2);

  ticks++;
  PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
printf("etimer, now: \t%ld\n", ticks);
 leds_toggle(LEDS_GREEN);

  ticks++;
ctimer_set(&ct, CLOCK_SECOND * 1, ctimer_callback_example, &ticks); 

  while(1) {
    PROCESS_YIELD();
}
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
