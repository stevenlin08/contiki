/* -----------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------
*/
#include "contiki-conf.h"
#include "lib/sensors.h"
#include "ti-lib.h"
#include "board.h"
#include "sys/clock.h"
//#include <stdbool.h>
//#include <ti/sysbios/knl/Clock.h>
//#include "gpio.h"
//#include "util.h"

//#include "board-i2c.h"
#include "i2c01.h"
//#include <ti/mw/sensors/SensorI2C.h>
#include "G_Sensor.h"
#include "sensor_adxl345.h"
//#include "math.h"
#include <stdio.h>

/* -----------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------
*/
#define ADXL345_ADDRESS_ALT_LOW     0x53 // alt address pin low (GND)
#define ADXL345_ADDRESS_ALT_HIGH    0x1D // alt address pin high (VCC)

/* Slave address */
#define SENSOR_I2C_ADDRESS          ADXL345_ADDRESS_ALT_LOW

#define AXDL345_VAL_PROD_ID         0xE5
/* axdl345 register addresses */
#define ADXL345_RA_DEVID            0x00
#define ADXL345_RA_RESERVED1        0x01
#define ADXL345_RA_THRESH_TAP       0x1D
#define ADXL345_RA_OFSX             0x1E
#define ADXL345_RA_OFSY             0x1F
#define ADXL345_RA_OFSZ             0x20
#define ADXL345_RA_DUR              0x21
#define ADXL345_RA_LATENT           0x22
#define ADXL345_RA_WINDOW           0x23
#define ADXL345_RA_THRESH_ACT       0x24
#define ADXL345_RA_THRESH_INACT     0x25
#define ADXL345_RA_TIME_INACT       0x26
#define ADXL345_RA_ACT_INACT_CTL    0x27
#define ADXL345_RA_THRESH_FF        0x28
#define ADXL345_RA_TIME_FF          0x29
#define ADXL345_RA_TAP_AXES         0x2A
#define ADXL345_RA_ACT_TAP_STATUS   0x2B
#define ADXL345_RA_BW_RATE          0x2C
#define ADXL345_RA_POWER_CTL        0x2D
#define ADXL345_RA_INT_ENABLE       0x2E
#define ADXL345_RA_INT_MAP          0x2F
#define ADXL345_RA_INT_SOURCE       0x30
#define ADXL345_RA_DATA_FORMAT      0x31
#define ADXL345_RA_DATAX0           0x32
#define ADXL345_RA_DATAX1           0x33
#define ADXL345_RA_DATAY0           0x34
#define ADXL345_RA_DATAY1           0x35
#define ADXL345_RA_DATAZ0           0x36
#define ADXL345_RA_DATAZ1           0x37
#define ADXL345_RA_FIFO_CTL         0x38
#define ADXL345_RA_FIFO_STATUS      0x39


/* Bit values */
#define ADXL345_AIC_ACT_AC_BIT      7
#define ADXL345_AIC_ACT_X_BIT       6
#define ADXL345_AIC_ACT_Y_BIT       5
#define ADXL345_AIC_ACT_Z_BIT       4
#define ADXL345_AIC_INACT_AC_BIT    3
#define ADXL345_AIC_INACT_X_BIT     2
#define ADXL345_AIC_INACT_Y_BIT     1
#define ADXL345_AIC_INACT_Z_BIT     0

#define ADXL345_TAPAXIS_SUP_BIT     3
#define ADXL345_TAPAXIS_X_BIT       2
#define ADXL345_TAPAXIS_Y_BIT       1
#define ADXL345_TAPAXIS_Z_BIT       0

#define ADXL345_TAPSTAT_ACTX_BIT    6
#define ADXL345_TAPSTAT_ACTY_BIT    5
#define ADXL345_TAPSTAT_ACTZ_BIT    4
#define ADXL345_TAPSTAT_ASLEEP_BIT  3
#define ADXL345_TAPSTAT_TAPX_BIT    2
#define ADXL345_TAPSTAT_TAPY_BIT    1
#define ADXL345_TAPSTAT_TAPZ_BIT    0

#define ADXL345_BW_LOWPOWER_BIT     4
#define ADXL345_BW_RATE_BIT         3

#define ADXL345_PCTL_LINK_BIT       5
#define ADXL345_PCTL_AUTOSLEEP_BIT  4
#define ADXL345_PCTL_MEASURE_BIT    3
#define ADXL345_PCTL_SLEEP_BIT      2
#define ADXL345_PCTL_WAKEUP_BIT     1

#define ADXL345_INT_DATA_READY_BIT  7
#define ADXL345_INT_SINGLE_TAP_BIT  6
#define ADXL345_INT_DOUBLE_TAP_BIT  5
#define ADXL345_INT_ACTIVITY_BIT    4
#define ADXL345_INT_INACTIVITY_BIT  3
#define ADXL345_INT_FREE_FALL_BIT   2
#define ADXL345_INT_WATERMARK_BIT   1
#define ADXL345_INT_OVERRUN_BIT     0

#define ADXL345_FORMAT_SELFTEST_BIT 7
#define ADXL345_FORMAT_SPIMODE_BIT  6
#define ADXL345_FORMAT_INTMODE_BIT  5
#define ADXL345_FORMAT_FULL_RES_BIT 3
#define ADXL345_FORMAT_JUSTIFY_BIT  2
#define ADXL345_FORMAT_RANGE_BIT    1


#define ADXL345_FIFO_MODE_BIT       7
#define ADXL345_FIFO_MODE_LENGTH    2
#define ADXL345_FIFO_TRIGGER_BIT    5
#define ADXL345_FIFO_SAMPLES_BIT    4
#define ADXL345_FIFOSTAT_TRIGGER_BIT        7
#define ADXL345_FIFOSTAT_LENGTH_BIT         5

/* Register length */
#define ADXL345_INT_SET_LENGTH      8
#define ADXL345_PCTL_WAKEUP_LENGTH  2
#define ADXL345_FORMAT_RANGE_LENGTH 2
#define ADXL345_BW_RATE_LENGTH      4
#define ADXL345_FIFO_SAMPLES_LENGTH 5
#define ADXL345_FIFOSTAT_LENGTH_LENGTH      6

#define REGISTER_LENGTH             1

/* Sensor data size */
#define DATA_SIZE                       2

#define ADXL345_INT_SET_STARTBIT         7

#define ADXL345_INT_DATA_READY_MASK (1<<ADXL345_INT_DATA_READY_BIT)
#define ADXL345_INT_SINGLE_TAP_MASK (1<<ADXL345_INT_SINGLE_TAP_BIT)
#define ADXL345_INT_DOUBLE_TAP_MASK (1<<ADXL345_INT_DOUBLE_TAP_BIT)
#define ADXL345_INT_ACTIVITY_MASK (1<<ADXL345_INT_ACTIVITY_BIT)
#define ADXL345_INT_INACTIVITY_MASK (1<<ADXL345_INT_INACTIVITY_BIT)
#define ADXL345_INT_FREE_FALL_MASK (1<<ADXL345_INT_FREE_FALL_BIT)
#define ADXL345_INT_WATERMARK_MASK (1<<ADXL345_INT_WATERMARK_BIT)
#define ADXL345_INT_OVERRUN_MASK (1<<ADXL345_INT_OVERRUN_BIT)

// Sensor selection/deselection
#define ADXL345_SELECT()     i2c_select(BOARD_IOID_SDA, BOARD_IOID_SCL, SENSOR_I2C_ADDRESS,I2C_SPEED_NORMAL, I2C_PULL_DOWN)//board_i2c_select(BOARD_I2C_INTERFACE_0,SENSOR_I2C_ADDRESS)
#define ADXL345_DESELECT()   i2c_deselect()//board_i2c_deselect()
/*---------------------------------------------------------------------------*/
#define SENSOR_STATUS_DISABLED     0
#define SENSOR_STATUS_INITIALISED  1
#define SENSOR_STATUS_NOT_READY    2
#define SENSOR_STATUS_READY        3
/* -----------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------
*/
static uint8_t buf[6];
static uint8_t val;
static int enabled = SENSOR_STATUS_DISABLED;
//G-sensor
static accel_data_t G_Status={0};
/* acc pin state */
//static PIN_State AccPinState;

/* acc Pin Handle */
//PIN_Handle AccPinHandle;

//static Clock_Struct AccPinClock;

// Pointer to application callback


/*static PIN_Config AccPinTable[] =
{   
    Board_ADXL345_INT1 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_POSEDGE,
  
};*/

/* -----------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------
*/
/** Set full resolution mode setting.
 * @param resolution New full resolution enabled setting
 * @see getFullResolution()
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_FULL_RES_BIT
 * Get full resolution mode setting.
 * When this bit is set to a value of 1, the device is in full resolution mode,
 * where the output resolution increases with the g range set by the range bits
 * to maintain a 4 mg/LSB scale factor. When the FULL_RES bit is set to 0, the
 * device is in 10-bit mode, and the range bits determine the maximum g range
 * and scale factor.
 * @return Full resolution enabled setting
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_FULL_RES_BIT
 */
static bool setFullResolution(uint8_t resolution) {
    return sensorWriteBits(ADXL345_RA_DATA_FORMAT,ADXL345_FORMAT_FULL_RES_BIT,REGISTER_LENGTH,resolution);
}
/** Set measurement data rate.
 * 0x7 =  12.5Hz
 * 0x8 =  25Hz, increasing or decreasing by factors of 2, so:
 * 0x9 =  50Hz
 * 0xA = 100Hz
 * @param rate New data rate (0x0 - 0xF)
 * @see ADXL345_RATE_100
 * @see ADXL345_RA_BW_RATE
 * @see ADXL345_BW_RATE_BIT
 * @see ADXL345_BW_RATE_LENGTH
 */
static bool  setSampleRate(uint8_t bw)
{
  return sensorWriteBits(ADXL345_RA_BW_RATE,ADXL345_BW_RATE_BIT,ADXL345_BW_RATE_LENGTH,bw);
   
}

static bool setIntMask(uint8_t mask)
{
   return sensorWriteBits(ADXL345_RA_INT_ENABLE,ADXL345_INT_SET_STARTBIT,ADXL345_INT_SET_LENGTH,mask);
}

static bool setSINGLE_TAP_IntPin(uint8_t pin)
{
   return sensorWriteBits(ADXL345_RA_INT_MAP,ADXL345_INT_SINGLE_TAP_BIT,ADXL345_INT_SET_LENGTH,pin);
}

/** Set interrupt mode setting.
 * @param mode New interrupt mode setting
 * @see getInterruptMode()
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_INTMODE_BIT
 */
static bool setInterruptMode(uint8_t mode) {
   return sensorWriteBits(ADXL345_RA_DATA_FORMAT,ADXL345_FORMAT_INTMODE_BIT,REGISTER_LENGTH,mode);

}

/** Set tap threshold.
  * @param threshold Tap magnitude threshold (scaled at 62.5 mg/LSB)
  * @see ADXL345_RA_THRESH_TAP
  * @see getTapThreshold()
  */
static bool setTapThreshold(uint8_t th)
{
  return sensorWriteReg(ADXL345_RA_THRESH_TAP,&th,sizeof(th)/sizeof(uint8_t));
}

/** Set tap detection X axis inclusion.
 * @param enabled X axis tap detection enabled value
 * @see getTapAxisXEnabled()
 * @see ADXL345_RA_TAP_AXES
 * @see ADXL345_TAPAXIS_X_BIT/Y/Z
 */

/*static bool setTapAxisXEnabled(uint8_t enable)
{
 return sensorWriteBits(ADXL345_RA_TAP_AXES, ADXL345_TAPAXIS_X_BIT,enable);
}

static bool setTapAxisYEnabled(uint8_t enable)
{
 return sensorWriteBits(ADXL345_RA_TAP_AXES, ADXL345_TAPAXIS_Y_BIT,enable);
}*/

static bool setTapAxisEnabled(uint8_t mask)
{
 return sensorWriteReg(ADXL345_RA_TAP_AXES, &mask,sizeof(mask)/sizeof(uint8_t));
}

/** Set tap duration.
 * @param duration Tap duration (scaled at 625 us/LSB)
 * @see getTapDuration()
 * @see ADXL345_RA_DUR
 */
static bool setTapDuration(uint8_t val)
{
 return sensorWriteReg(ADXL345_RA_DUR, &val,sizeof(val)/sizeof(uint8_t));  
}
/** Set tap duration.
 * @param latency Tap latency (scaled at 1.25 ms/LSB)
 * @see getDoubleTapLatency()
 * @see ADXL345_RA_LATENT
 */
static bool setDoubleTapLatency(uint8_t val)
{
 return sensorWriteReg(ADXL345_RA_LATENT, &val,sizeof(val)/sizeof(uint8_t));    
}

/** Set auto-sleep enabled status.
 * @param enabled New auto-sleep status
 * @see getAutoSleepEnabled()
 * @see ADXL345_RA_POWER_CTL
 * @see ADXL345_PCTL_AUTOSLEEP_BIT
 */
static bool setAutoSleepEnabled(bool enabled)
{
   return sensorWriteBits(ADXL345_RA_POWER_CTL,ADXL345_PCTL_AUTOSLEEP_BIT,REGISTER_LENGTH,enabled);
}

/** Set activity/inactivity serial linkage status.
 * @param enabled New link status
 * @see ADXL345_RA_POWER_CTL
 * @see ADXL345_PCTL_LINK_BIT
 */
static bool setLinkEnabled(bool enabled) {
   return sensorWriteBits(ADXL345_RA_POWER_CTL, ADXL345_PCTL_LINK_BIT,REGISTER_LENGTH,enabled);  
}

/** Set activity threshold.
 * @param threshold Activity threshold (scaled at 62.5 mg/LSB)
 * @see getActivityThreshold()
 * @see ADXL345_RA_THRESH_ACT
 */
static bool setActivityThreshold(uint8_t threshold) {
 return sensorWriteReg(ADXL345_RA_THRESH_ACT, &threshold,sizeof(threshold)/sizeof(uint8_t));    
}

/** Set inactivity threshold.
 * @param threshold Inctivity threshold (scaled at 62.5 mg/LSB)
 * @see getInctivityThreshold()
 * @see ADXL345_RA_THRESH_INACT
 */
static bool setInactivityThreshold(uint8_t threshold) {
 return sensorWriteReg(ADXL345_RA_THRESH_INACT, &threshold,sizeof(threshold)/sizeof(uint8_t));    
}


/** Set inactivity time.
 * @param time Inactivity time (scaled at 1 sec/LSB)
 * @see setInctivityTime()
 * @see ADXL345_RA_TIME_INACT
 */
static bool setInctivityTime(uint8_t time) {
   return sensorWriteReg(ADXL345_RA_TIME_INACT, &time,sizeof(time)/sizeof(uint8_t));    
}

/** Set activity AC/DC coupling.
 * @param enabled Activity AC/DC coupling (1 for AC, 0 for DC)
 * @see getActivityAC()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_ACT_AC_BIT
 */
static bool setActivityAC(bool enabled) {
   return sensorWriteBits(ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_ACT_AC_BIT,REGISTER_LENGTH,enabled);  
}


/** Set ACTIVITY interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_ACTIVITY_BIT
 */
static bool setIntActivityEnabled(bool enabled) {
  return sensorWriteBits(ADXL345_RA_INT_ENABLE, ADXL345_INT_ACTIVITY_BIT,REGISTER_LENGTH,enabled);  
  
}

/** Set INACTIVITY interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_INACTIVITY_BIT
 */
static bool setIntInactivityEnabled(bool enabled) {
  return sensorWriteBits(ADXL345_RA_INT_ENABLE, ADXL345_INT_INACTIVITY_BIT,REGISTER_LENGTH,enabled);  
  
}

/** Set ACTIVITY interrupt pin.
 * @param pin Interrupt pin setting (0 as INT1;1 as INT2)
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_ACTIVITY_BIT
 */
static bool setIntActivityPin(uint8_t pin) {
  return sensorWriteBits(ADXL345_RA_INT_MAP, ADXL345_INT_ACTIVITY_BIT,REGISTER_LENGTH,pin);  

}

/** Set ACTIVITY interrupt pin.
 * @param pin Interrupt pin setting (0 as INT1;1 as INT2)
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_ACTIVITY_BIT
 */
static bool setIntInActivityPin(uint8_t pin) {
  return sensorWriteBits(ADXL345_RA_INT_MAP, ADXL345_INT_INACTIVITY_BIT,REGISTER_LENGTH,pin);  

}

/** Set X axis activity monitoring inclusion.
 * @param enabled X axis activity monitoring inclusion value
 * @see getActivityAC()
 * @see getActivityXEnabled()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_ACT_X_BIT
 */
static bool setActivityXEnabled(bool enabled) {
  return sensorWriteBits(ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_ACT_X_BIT,REGISTER_LENGTH,enabled);    
}

static bool setActivityYEnabled(bool enabled) {
  return sensorWriteBits(ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_ACT_Y_BIT,REGISTER_LENGTH,enabled);    
}

static bool setActivityZEnabled(bool enabled) {
  return sensorWriteBits(ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_ACT_Z_BIT,REGISTER_LENGTH,enabled);    
}

/** Get X axis inactivity monitoring.
 * @return Y axis inactivity monitoring enabled value
 * @see getActivityAC()
 * @see getActivityXEnabled()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_INACT_X_BIT
 */
static bool   setInactivityXEnabled(bool enabled) {
  return sensorWriteBits(ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_INACT_X_BIT,REGISTER_LENGTH,enabled);    
}
static bool   setInactivityYEnabled(bool enabled) {
  return sensorWriteBits(ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_INACT_Y_BIT,REGISTER_LENGTH,enabled);    
}
static bool   setInactivityZEnabled(bool enabled) {
  return sensorWriteBits(ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_INACT_Z_BIT,REGISTER_LENGTH,enabled);      
}

/** Set low power enabled status.
 * @see getLowPowerEnabled()
 * @param enabled Low power enable setting
 * @see ADXL345_RA_BW_RATE
 * @see ADXL345_BW_LOWPOWER_BIT
 */
static bool setLowPowerEnabled(bool enabled) {
    //I2Cdev::writeBit(devAddr, ADXL345_RA_BW_RATE, ADXL345_BW_LOWPOWER_BIT, enabled);
    return sensorWriteBits(ADXL345_RA_BW_RATE, ADXL345_BW_LOWPOWER_BIT,REGISTER_LENGTH,enabled);    
    
}
/* -----------------------------------------------------------------------------
*                                           Public functions
* ------------------------------------------------------------------------------
*/
/*void board_Acc_pinFxn(PIN_Handle keyPinHandle, PIN_Id keyPinId)
{
    if(Util_isClockActive(&AccPinClock) != true)
    {
        Util_startClock(&AccPinClock);
    }

  return;
}

static void acc_int_changeHandler(UArg a0)
{
  static uint8_t i =0;
  uint8_t data[6];
  SENSOR_SELECT();
  
  if(i ==0)
  {
  i=1;
  sensorReadBytes(ADXL345_RA_DATAX0, 2,
                          buf);
  //Task_sleep(10000/ti_sysbios_knl_Clock_tickPeriod); //1 second
  
  sensorReadBytes(ADXL345_RA_DATAY0, 2,
                          buf+2);
  sensorReadBytes(ADXL345_RA_DATAZ0, 2,
                          buf+4);

  }

  else if( i==1)
  {
  i =2;
  }

  else if(i ==2)
  {
  i=0;
  }

  SENSOR_DESELECT();
  if(i != 0)
  {
        if(Util_isClockActive(&AccPinClock) != true)
    {
        Util_startClock(&AccPinClock);
    }
  }
}*/

/*******************************************************************************
 * @fn          Accelerometer_Init
 *
 * @brief       Initialise the temperature sensor driver
 *
 * @return      none
 ******************************************************************************/
uint8_t reg01=0;
bool Accelerometer_Init(void)
{
  // Configure sensor
  uint8_t reg;
  // Select this sensor on the I2C bus
	ADXL345_SELECT();


  //if(! setFullResolution(true))//0x31
  //    return false;
 /* 
  setSampleRate(ADXL345_RATE_6P25);
  //setSampleRate(ADXL345_RATE_0P20);
  
  //setLowPowerEnabled(true);
  
  setSINGLE_TAP_IntPin(0x00);//int map  //need chage the inaction /action mapping different io
  setTapThreshold(0x13);
  setTapDuration(0x13);
  
  setDoubleTapLatency(0xa0);
  setTapAxisEnabled(0x07);
  
  setInterruptMode(true); //set io as low action  
  setIntMask(ADXL345_INT_SINGLE_TAP_MASK);//int enable(inaction /action)  
//start to measue
  sensorWriteBits(ADXL345_RA_POWER_CTL,ADXL345_PCTL_MEASURE_BIT,1,1);
*/ 
/*
  //setSampleRate(ADXL345_RATE_400); 
  
  setSampleRate(ADXL345_RATE_0P20);
  setIntActivityPin(0);
  setIntInActivityPin(1);
  
  setIntMask(ADXL345_INT_ACTIVITY_MASK|ADXL345_INT_INACTIVITY_MASK);//int enable(inaction /action)
  setInterruptMode(true); //set io as low action
  
  setInactivityThreshold(0x3);
  setInctivityTime(0x03);
  
  setActivityThreshold(0x6);

  setActivityXEnabled(1);
  setActivityYEnabled(1);
  setActivityZEnabled(1);  
  setInactivityXEnabled(1);
  setInactivityYEnabled(1);
  setInactivityZEnabled(1);    
  setActivityAC(true);
  sensorWriteBits(ADXL345_RA_POWER_CTL,ADXL345_PCTL_MEASURE_BIT,1,1);  
  sensorReadReg(ADXL345_RA_POWER_CTL,&reg01,1);
  
  //setLinkEnabled(true);//to auto sleep  
  //setAutoSleepEnabled(true);
  //reg = 0x3f;
  //sensorWriteReg(ADXL345_RA_POWER_CTL, &reg,1);    
*/

  //reg = 0x18;
  //reg = 0x08;
  //sensorWriteReg(ADXL345_RA_INT_MAP,&reg,1);
  setIntActivityPin(0);
  setIntInActivityPin(1);
   
  //reg = 0x18;
  //sensorWriteReg(ADXL345_RA_INT_ENABLE,&reg,1);
  setIntMask(ADXL345_INT_ACTIVITY_MASK|ADXL345_INT_INACTIVITY_MASK);//int enable(inaction /action)
  
  //reg = 0x21;
  //sensorWriteReg(ADXL345_RA_DATA_FORMAT,&reg,1);
  setInterruptMode(true); //set io as low action
 
  reg = 0x15;
  sensorWriteReg(ADXL345_RA_BW_RATE,&reg,1);
  
  reg = 0x01;
  sensorWriteReg(ADXL345_RA_TIME_INACT,&reg,1);
  reg = 0x01;
  sensorWriteReg(ADXL345_RA_THRESH_INACT,&reg,1);
  
  reg=0x06;
  sensorWriteReg(ADXL345_RA_THRESH_ACT,&reg,1);
  
  reg=0xff;
  sensorWriteReg(ADXL345_RA_ACT_INACT_CTL,&reg,1);
  //setActivityXEnabled(1);
  //setActivityYEnabled(1);
  //setActivityZEnabled(1);  
  //setInactivityXEnabled(1);
  //setInactivityYEnabled(1);
  //setInactivityZEnabled(1);    
  //setActivityAC(true); 
  
  //reg=0x3f;
  reg=0x38;
  sensorWriteReg(ADXL345_RA_POWER_CTL,&reg,1);
  //sensorWriteBits(ADXL345_RA_POWER_CTL,ADXL345_PCTL_MEASURE_BIT,1,1); 
  //read data to clear int

  
  ST_ASSERT(sensorReadReg(ADXL345_RA_DATAX0, buf,
                          DATA_SIZE));
  sensorReadReg(ADXL345_RA_DATAY0, buf+2,
                          DATA_SIZE);
  sensorReadReg(ADXL345_RA_DATAZ0, buf+4,
                          DATA_SIZE);
    
  ST_ASSERT(sensorReadReg(ADXL345_RA_INT_SOURCE, buf,
                          1));
  
  ST_ASSERT(sensorReadReg(ADXL345_RA_ACT_TAP_STATUS, buf,
                          1));
 

  ADXL345_DESELECT();
  
  return true;
}


/*******************************************************************************
 * @fn          Accelerometer_Test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 ******************************************************************************/
bool Accelerometer_Test(void)
{
  uint8_t reg=0;
  //sprintf(dgstringAcm,"accetest");
  //Log_info0(dgstringAcm);
  
  // Select this sensor on the I2C bus
	ADXL345_SELECT();
  // Check product ID
  ST_ASSERT(sensorReadReg(ADXL345_RA_DEVID, (uint8_t *)&val, REGISTER_LENGTH));
  ST_ASSERT(val == AXDL345_VAL_PROD_ID);

//clock_delay_usec(50000);
  //Clear POWER_CTL Reg.
  reg = 0x00;

  sensorWriteReg(ADXL345_RA_POWER_CTL, &reg,1);  

//clock_delay_usec(50000);
 
  //bool sensorWriteBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
  sensorWriteBits(ADXL345_RA_POWER_CTL,ADXL345_PCTL_MEASURE_BIT,1,1);

  //bool sensorReadBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
  sensorReadBits(ADXL345_RA_POWER_CTL,ADXL345_PCTL_MEASURE_BIT,1,&val);
  
  ST_ASSERT( val == 1);

  sensorWriteBits(ADXL345_RA_POWER_CTL,ADXL345_PCTL_MEASURE_BIT,1,0);
  
  sensorReadBits(ADXL345_RA_POWER_CTL,ADXL345_PCTL_MEASURE_BIT,1,&val);
  
  ST_ASSERT( val == 0);
#if 0 
  /*ST_ASSERT(sensorReadReg(ADXL345_RA_POWER_CTL, (uint8_t *)&val,
                          REGISTER_LENGTH));

  //set check
  val |= (0xFF & (1<<SELF_WRITEON_TESTBIT));
  
  ST_ASSERT(sensorWriteReg(ADXL345_RA_POWER_CTL, (uint8_t *)&val,
                           REGISTER_LENGTH));

  ST_ASSERT(sensorReadReg(ADXL345_RA_POWER_CTL, (uint8_t *)&val,
                          REGISTER_LENGTH));

  ST_ASSERT((val & (1<<SELF_WRITEON_TESTBIT)) == (1<<SELF_WRITEON_TESTBIT));

  //clear check
  val =val &( ~ (1<<SELF_WRITEON_TESTBIT));
  
  ST_ASSERT(sensorWriteReg(ADXL345_RA_POWER_CTL, (uint8_t *)&val,
                           REGISTER_LENGTH));

  ST_ASSERT(sensorReadReg(ADXL345_RA_POWER_CTL, (uint8_t *)&val,
                          REGISTER_LENGTH));

  ST_ASSERT((val &( ~(1<<SELF_WRITEON_TESTBIT))) == 0);*/
#endif
  ADXL345_DESELECT();

  return true;
}

bool Accelerometer_Read(accel_data_t* dat)
{
    // Select this sensor on the I2C bus

  ADXL345_SELECT();
 /* 
  sensorReadBytes(ADXL345_RA_DATAX0, 2,
                          buf);
  sensorReadBytes(ADXL345_RA_DATAY0, 2,
                          buf);
  sensorReadBytes(ADXL345_RA_DATAZ0, 2,
                          buf);

  sensorReadBytes(ADXL345_RA_DATAX1, 2,
                          buf);
  sensorReadBytes(ADXL345_RA_DATAY1, 2,
                          buf);
  sensorReadBytes(ADXL345_RA_DATAZ1, 2,
                          buf);  
 */ 
  // Check product ID
  //ST_ASSERT(sensorReadReg(ADXL345_RA_DEVID, (uint8_t *)&val, REGISTER_LENGTH));
  //ST_ASSERT(val == AXDL345_VAL_PROD_ID); 
	//sensorReadReg(ADXL345_RA_DEVID, (uint8_t *)&val, REGISTER_LENGTH);

   sensorReadReg(ADXL345_RA_DATAX0, buf,
                          2);
  
  sensorReadReg(ADXL345_RA_DATAY0, buf,
                          2); 
   sensorReadReg(ADXL345_RA_DATAZ0, buf,
                          2);
  
  sensorReadReg(ADXL345_RA_DATAX1, buf,
                          2); 
    sensorReadReg(ADXL345_RA_DATAY1, buf,
                          2);
  
  sensorReadReg(ADXL345_RA_DATAZ1, buf,
                          2);  
  
  sensorReadReg(ADXL345_RA_ACT_TAP_STATUS, buf,
                          2);

    sensorReadReg(ADXL345_RA_FIFO_STATUS, buf,
                          2);
    
  sensorReadReg(ADXL345_RA_INT_SOURCE, buf,
                          2);

  sensorReadReg(ADXL345_RA_ACT_TAP_STATUS, buf,
                          2);

  sensorReadReg(ADXL345_RA_DEVID, buf, REGISTER_LENGTH+1);
	
  ADXL345_DESELECT();
  if(buf[0]==AXDL345_VAL_PROD_ID)
  {
	return true;
  }

	return false;
  //printf("Accelerometer_Read state_1=%x\n\r",buf[0]);
}
/*******************************************************************************
*******************************************************************************/

/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \param type BMP_280_SENSOR_TYPE_TEMP or BMP_280_SENSOR_TYPE_PRESS
 * \return Temperature (centi degrees C) or Pressure (Pascal).
 */
static int value(int type)
{
  int rv=0,ReTryCnt=0;
  //int32_t temp = 0;
  //uint32_t pres = 0;

	if(enabled != SENSOR_STATUS_READY) {
		printf("Sensor disabled or starting up (%d)\n", enabled);
		return CC26XX_SENSOR_READING_ERROR;
	}
	else 
	{
		if(type ==ADXL_345_SENSOR_TYPE_READ)
		{
			for(ReTryCnt=0;ReTryCnt<20;ReTryCnt++)
			{
				if(true==Accelerometer_Read(&G_Status))
				{
					rv = 0;
					//printf("ReTryCnt %d rv %d\n\r",ReTryCnt,rv);
					break;
				}
				clock_delay_usec(5000);
				printf("Delay ReTryCnt %d\n\r",ReTryCnt);
			}
		}
	}

  return rv;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the TMP007 sensor.
 *
 * \param type Activate, enable or disable the sensor. See below
 * \param enable
 *
 * When type == SENSORS_HW_INIT we turn on the hardware
 * When type == SENSORS_ACTIVE and enable==1 we enable the sensor
 * When type == SENSORS_ACTIVE and enable==0 we disable the sensor
 */
static int configure(int type, int enable)
{
  switch(type) {
  case SENSORS_HW_INIT:
	enabled = SENSOR_STATUS_INITIALISED;
    break;
  case SENSORS_ACTIVE:
    /* Must be initialised first */
    if(enable) {
		enabled = SENSOR_STATUS_READY;
		sensorTestExecute();
    } else {

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
static int status(int type)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(adxl345_sensor, "ADXL345", value, configure, status);
/*---------------------------------------------------------------------------*/





