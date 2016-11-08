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
//#include "launchpad/button-sensor.h"
//#include "gpio-interrupt.h"
#include "sys/timer.h"
#include "lpm.h"
#include "ti-lib.h"
#include <stdint.h>


#include "dev/leds.h"
#include "mled_sensor.h"
#include <stdio.h> /* For printf() */

/* The callback timer library */
#include "sys/ctimer.h"

#define DEBUG 1
/*---------------------------------------------------------------------------*/
typedef enum
{
    led_type_NoCoord,  //reset no bind,no Coord
    led_type_Coord,    //rest  no bind,Coord
} led_type_t;

typedef enum
{
    led_color_red,
    led_color_green,
    led_color_yellow
} led_color_t;


struct led_flash_s {
  led_type_t  p_type;
  led_color_t p_color;
  uint16_t    p_times;
};

const struct led_flash_s led_flash_mode[2] = {
    {led_type_NoCoord,led_color_red  ,5},
    {led_type_Coord   ,led_color_green,5},
};

/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
#define BOARD_LED_BLINK_PERIOD 1000     // in milliseconds
#define MAX_LEDS 3

#define LED_R1_IDX 0
#define LED_G1_IDX 1
#define LED_Y1_IDX 2

static  uint8_t        s_active_leds;           /** Bitmask of active LEDs. */
static  uint8_t        s_blink_leds;            /** bitmask of blink led */
static  uint16_t       s_blinks_left[MAX_LEDS]; /** How many times the active LEDs should flash. */


static struct ctimer ct;

#define ms(x)  (CLOCK_SECOND*x/1000)

/*---------------------------------------------------------------------------*/
//==========================================================
// 
//==========================================================
static void drv_leds_clr_all(void)
{
    uint8_t i;
    for (i = 0; i< MAX_LEDS; ++i)
    {
      leds_off(i+1);
    }

#if (DEBUG==1)
    printf("drv_leds_clr_all..\n");
#endif
}

//==========================================================
//
//==========================================================
static void drv_leds_set(uint8_t i)
{
    leds_on(i+1); // +1 mapping to board.h led defined

#if (DEBUG==1)
    printf("drv_leds_set..\n");
#endif
}

//==========================================================
//
//==========================================================
static void drv_leds_clr(uint8_t i)
{
    leds_off(i+1);

#if (DEBUG==1)
    printf("drv_leds_clr..\n");
#endif
}


/*---------------------------------------------------------------------------*/
static void
m_led_changeHandler(void *ptr)
{
    uint_fast8_t i;

#if (DEBUG==1)
    printf("m_led_changeHandler..\n");
    //leds_toggle(1); // for test (red)
#endif 
//--------------------------------------
    for (i = 0; i< MAX_LEDS; ++i)
    {
        if ((s_active_leds &(1 << i)) == 0)
        {
            // This LED is not active
            continue;
        }
       
        if ((s_blinks_left[i] & 1) == 0)
        {
            // Setting the LED on even-numbered blink count
            drv_leds_set(i); 
        }
        else
        {
            // Clearing the LED on odd-numbered blink count
            drv_leds_clr(i);
        }
    }
//--------------------------------------
    for(i = 0; i < MAX_LEDS; ++i)
    {
        // Updating the remaining flips
        s_blinks_left[i] -= s_active_leds & (1 << i) ? 1 : 0;
		
        // Marking LEDs with no remaining flips as inactive
        s_active_leds &= ~((s_blinks_left[i] <= 0) << i);   
        s_blink_leds  &= ~((s_blinks_left[i] <= 0) << i);  

#if (DEBUG==1)
    	printf("s_active_leds=%d..\n",s_active_leds);
    	printf("s_blinks_left[%d]=%d..\n",i,s_blinks_left[i]);
#endif
    }
 
//--------------------------------------
    if (s_active_leds == 0)
    {
	ctimer_stop(&ct);
#if (DEBUG==1)
        printf("ctimer_stop..\n");
#endif
    }
    else
    {
	ctimer_restart(&ct);	// Restart the timer from the previous expiration time.
#if (DEBUG==1)
	printf("ctimer_restart..\n");
#endif
    }
}


//==========================================================
//
//==========================================================
uint32_t m_leds_flash(led_type_t p_type, led_color_t p_color, uint16_t p_times)
{
    uint32_t err_code = false;

    uint8_t  index1=0; // Index to s_active_leds and s_blinks_left for 1st LED
    uint8_t  index2=0; // Index to s_active_leds and s_blinks_left for 2nd LED
   
    // Updating s_active_leds bitmask to indicate which LEDs are to be flashed
    // and updating s_blinks_left to indicate how many times the LED should be flashed


#if !defined(__LED_TEST)
    if (!ctimer_expired(&ct) == true);
    {
	ctimer_stop(&ct);
    }

    if (p_type == led_type_NoCoord)
    {   
        drv_leds_clr(LED_R1_IDX);
        s_active_leds &= ~((1 << LED_R1_IDX));
        s_blink_leds  &= ~((1 << LED_R1_IDX));

        switch (p_color)
        {
            case led_color_red:
                //ctimer_set(&ct, ms(2500), m_led_changeHandler, NULL);
                ctimer_set(&ct, ms(250), m_led_changeHandler, NULL);
	
                index1 = index2 = LED_R1_IDX;
                s_blink_leds |= ((1 << index1)|(1 << index2));                           
                break;
            /*case led_color_green:
                index1 = index2 = LED_G1_IDX;
                break;
           
            case led_color_orange:
                index1 = LED_R1_IDX;
                index2 = LED_G1_IDX;
                break;*/           
            default:
                err_code= true;
        }
    }
    else if(p_type == led_type_Coord)
    {
        drv_leds_clr(LED_R1_IDX);

        s_active_leds &= ~((1 << LED_R1_IDX));
        s_blink_leds  &= ~((1 << LED_R1_IDX));
    
        ctimer_set(&ct, ms(500), m_led_changeHandler, NULL);

        index1 = index2 = LED_R1_IDX;
        err_code= false;
        //s_blink_leds |= ((1 << index1)|(1 << index2));                                  
    }
    else
        ;
   
    if (s_active_leds == 0)
    {
        // Timer is probably not running
	ctimer_restart(&ct);	// Restart the timer from current time. 
    }
   
    s_blinks_left[index1] = p_times * 2;
    s_blinks_left[index2] = p_times * 2;
    s_active_leds        |= (1 << index1);
    s_active_leds        |= (1 << index2);
#else
      leds_on(LED_Y1_IDX);

#endif

#if (DEBUG==1)  
    printf("m_leds_flash..\n");
#endif
    return err_code;
}



/*---------------------------------------------------------------------------*/
static int
config_led(int type, int value)
{
  switch(type) {
//------------------------------
  case SENSORS_HW_INIT:
#if (DEBUG==1)   
    printf("LED SENSORS_HW_INIT..\n");
#endif

    leds_init();
    drv_leds_clr_all();

    break;

//------------------------------
  case SENSORS_ACTIVE:
#if (DEBUG==1)   
    printf("LED SENSORS_ACTIVE..\n");
#endif
    //if (c)
    //{
      s_active_leds = 0;
      ctimer_set(&ct, 0, m_led_changeHandler, NULL);						
    //}

    break;

  default:
    break;
  }

  return 1;
}

/*---------------------------------------------------------------------------*/
static int
value_led(int type)
{
  return m_leds_flash(led_flash_mode[type].p_type,led_flash_mode[type].p_color,led_flash_mode[type].p_times);
}

/*---------------------------------------------------------------------------*/
static int
status_led(int type)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(mled_sensor, LED_SENSOR, value_led, config_led, status_led);

/*---------------------------------------------------------------------------*/
/** @} */
