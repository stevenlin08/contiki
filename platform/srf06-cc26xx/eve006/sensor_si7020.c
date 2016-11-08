/** ============================================================================
*  @file       si7020.c
*
*  @brief      This file is a simple gateway to include the appropriate si7020.c
*              file which is located in the following directories relative to this file:
*                  - Devices
*
*              The project should set the include path to si7020.h.
*
*  ============================================================================
*/

#include "contiki-conf.h"
#include "lib/sensors.h"
#include "ti-lib.h"
#include "sensor_si7020.h"
#include "sys/ctimer.h"
#include "sys/clock.h"

#include <stdio.h>
#include <string.h>

#define SI7020_ID_LEN 2
#define SI7020_TEMP_LEN 1

uint8_t SI7020_CMD_ID1[]= {0xFA,0x0F};
uint8_t SI7020_CMD_ID2[]= {0xFC,0xC9};

uint8_t SI7020_CMD_TEMP_NO_HOLD= 0xF3;
uint8_t SI7020_CMD_RH_NO_HOLD= 0xF5;
uint8_t SI7020_CMD_TEMP_PREV_RH= 0xE0;


uint8_t SI7020_CMD_READ_REG1= 0xE7;
uint8_t SI7020_CMD_WRITE_REG1= 0xE6;

uint8_t data[6]={0};
float HR_Value2=0, Temp_Vlaue2=0;
uint8_t  sensorSi7020_data2[6]={0};




/*---------------------------------------------------------------------------*/
#define SENSOR_STATUS_DISABLED     0
#define SENSOR_STATUS_INITIALISED  1
#define SENSOR_STATUS_NOT_READY    2
#define SENSOR_STATUS_READY        3
/* Wait SENSOR_STARTUP_DELAY clock ticks for the sensor to be ready - ~80ms */
#define SENSOR_STARTUP_DELAY 3

static int enabled = SENSOR_STATUS_DISABLED;
static struct ctimer startup_timer;
static bool si7020_int=false;
/*******************************************************************************
 * @fn          sensorTmpSi7020Init
 *
 * @brief       Initialise the temperature sensor driver
 *
 * @return      none
 ******************************************************************************/
bool sensorTmpSi7020Init(void)
{
    bool success = true;


    uint8_t Si7020_ID1[8]={0x00};
    uint8_t Si7020_ID2[6]= {0x00,0x00,0x00,0x00,0x00,0x00};

 	SI7020_SELECT();

    //read Si7020 ID
    success = i2c_write(SI7020_CMD_ID1,2);
    success = i2c_read(Si7020_ID1,8);

    success = i2c_write(SI7020_CMD_ID2,2);
    success = i2c_read(Si7020_ID2,6);
    //success = sensorReadReg(SI7020_CMD_ID1,cmd_buf,8);

    SI7020_DESELECT();
	//printf("Si7020_ID2[0]=%x\n\r",Si7020_ID2[0]);
    // Si7020 ID can't be zero
    if(Si7020_ID2[0] == 0x00 )
    {
        return false;
    }

    return success;

}


/*******************************************************************************
* @fn          sensorSi7020_Get_Temp
*
* @brief       Initialise the temperature sensor driver
*
* @return      none
******************************************************************************/
uint8_t * sensorSi7020_Get_RH_Temp(void)
{


    bool success = true;

    // data : 0-2 for humudity
    // data : 3-5 for temperature

    SI7020_SELECT();

    // read Relative Humidity , no hold mode 0xF5
	//i2c_write(&SI7020_CMD_RH_NO_HOLD,1);
	i2c_write_single(SI7020_CMD_RH_NO_HOLD);
  	//clock_delay(61000);
      clock_delay_usec(65000);
    success = i2c_read(data,3);

    // read Temp from previous RH measurment 0xF3
	i2c_write_single(SI7020_CMD_TEMP_NO_HOLD);
	//i2c_write(&SI7020_CMD_TEMP_NO_HOLD,1);
  	//clock_delay(61000);
      clock_delay_usec(65000);
    success = i2c_read(&data[3],3);

    SI7020_DESELECT();


    return data;

}


/*******************************************************************************
 * @fn          sensorSi7020_Get_RH
 *
 * @brief       Initialise the temperature sensor driver
 *
 * @return      none
 ******************************************************************************/
bool sensorSi7020_Get_RH(void)
{
    bool success = true;
//   success=true;


    uint8_t buff[3];

    SI7020_SELECT();

    // read Relative Humidity , no hold mode 0xF5
    success = i2c_write(&SI7020_CMD_RH_NO_HOLD,1);

    //Task_sleep(100 * 1000 / Clock_tickPeriod);
    success = i2c_read(buff,3);

    SI7020_DESELECT();


    return success;

}


/*******************************************************************************
 * @fn          sensorSi7020_Get_Temp
 *
 * @brief       Initialise the temperature sensor driver
 *
 * @return      none
 ******************************************************************************/
void sensorSi7020_Convert(uint8_t* temp)
{
    //bool success;
    uint16_t code;
    code=temp[0];
    code=code<<8;
    code=code | temp[1];



}

/***************************************************************************
 * @fn      Converted Humidity and Temperature Function
 *
 * @brief
 *
 * @param   led - board_led type
 *
 * @return  PIN Led Type
 */
void SensorSi7020_Converted_RH_Temp(uint8_t * Si7020_RawData,float *HR,float *Temp)
{
     uint16_t  HR_Row=0 ,Temp_Row=0;

     HR_Row=((Si7020_RawData[3]<<8) | (Si7020_RawData[4]));
     Temp_Row =  ((Si7020_RawData[0]<<8) | (Si7020_RawData[1]));

     *Temp =  ((175.72*HR_Row)/65536.0)-46.85;
     *HR = ((125.0*Temp_Row)/65536.0)-6.0;
}
/*---------------------------------------------------------------------------*/
static void notify_ready(void *not_used)
{
  memcpy(sensorSi7020_data2,sensorSi7020_Get_RH_Temp(),6);
  SensorSi7020_Converted_RH_Temp(sensorSi7020_data2,&HR_Value2,&Temp_Vlaue2);
  enabled = SENSOR_STATUS_READY;
  sensors_changed(&si7020_sensor);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \param type BMP_280_SENSOR_TYPE_TEMP or BMP_280_SENSOR_TYPE_PRESS
 * \return Temperature (centi degrees C) or Pressure (Pascal).
 */
static int value(int type)
{
  int rv;
  //int32_t temp = 0;
  //uint32_t pres = 0;

	if(enabled != SENSOR_STATUS_READY) {
		printf("Sensor disabled or starting up (%d)\n", enabled);
		return CC26XX_SENSOR_READING_ERROR;
	}

	if((type != SI_7020_SENSOR_TYPE_TEMP) && (type != SI_7020_SENSOR_TYPE_HR)) {
		printf("Invalid type\n");
		return CC26XX_SENSOR_READING_ERROR;
	} 
	else 
	{

		if(type == SI_7020_SENSOR_TYPE_TEMP) {
		  	rv = (int)Temp_Vlaue2;
		} 
		else if(type == SI_7020_SENSOR_TYPE_HR) {
		  rv = (int)HR_Value2;
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
		enabled = SENSOR_STATUS_NOT_READY;
		if(si7020_int==false)
		{
			//printf("si7020_int...\n\r");
  			sensorTmpSi7020Init();
			si7020_int=true;
		}

		ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
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
SENSORS_SENSOR(si7020_sensor, "SI7020", value, configure, status);
/*---------------------------------------------------------------------------*/


