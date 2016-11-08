#include "contiki-conf.h"
#include "lib/sensors.h"
#include "lpm.h"
#include "ti-lib.h"
#include "board-peripherals.h"
#include "i2c01.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
/*---------------------------------------------------------------------------*/
static void
wakeup_handler(void)
{
  /* Turn on the PERIPH PD */

  /*add ,to turn off i2c driver*/
    i2c_shutdown();

  ti_lib_prcm_power_domain_on(PRCM_DOMAIN_PERIPH);
  while((ti_lib_prcm_power_domain_status(PRCM_DOMAIN_PERIPH)
         != PRCM_DOMAIN_POWER_ON));
}
/*---------------------------------------------------------------------------*/
/*
 * Declare a data structure to register with LPM.
 * We don't care about what power mode we'll drop to, we don't care about
 * getting notified before deep sleep. All we need is to be notified when we
 * wake up so we can turn power domains back on
 */
LPM_MODULE(launchpad_module, NULL, NULL, wakeup_handler, LPM_DOMAIN_NONE);
/*---------------------------------------------------------------------------*/
static void
configure_unused_pins(void)
{
  /*uint32_t pins[] = {
    BOARD_IOID_CS, BOARD_IOID_TDO, BOARD_IOID_TDI, BOARD_IOID_DIO12,
    BOARD_IOID_DIO15, BOARD_IOID_DIO21, BOARD_IOID_DIO22, BOARD_IOID_DIO23,
    BOARD_IOID_DIO24, BOARD_IOID_DIO25, BOARD_IOID_DIO26, BOARD_IOID_DIO27,
    BOARD_IOID_DIO28, BOARD_IOID_DIO29, BOARD_IOID_DIO30,
    IOID_UNUSED
  };

  uint32_t *pin;

  for(pin = pins; *pin != IOID_UNUSED; pin++) {
    ti_lib_ioc_pin_type_gpio_input(*pin);
    ti_lib_ioc_io_port_pull_set(*pin, IOC_IOPULL_DOWN);
  }*/
}
/*---------------------------------------------------------------------------*/
void
board_init()
{
  /* Disable global interrupts */
  bool int_disabled = ti_lib_int_master_disable();

  /* Turn on relevant PDs */
  wakeup_handler();

  /* Enable GPIO peripheral */
  ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);

  /* Apply settings and wait for them to take effect */
  ti_lib_prcm_load_set();
  while(!ti_lib_prcm_load_get());
  
  /*add i2c dirver */
  i2c_wakeup();
  /* Make sure the external flash is in the lower power mode */
  //ext_flash_init();

  lpm_register_module(&launchpad_module);

  /* For unsupported peripherals, select a default pin configuration */
  configure_unused_pins();

  /* Re-enable interrupt if initially enabled. */
  if(!int_disabled) {
    ti_lib_int_master_enable();
  }
}
/*---------------------------------------------------------------------------*/
/** @} */
