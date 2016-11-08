#ifndef SI7020_H
#define SI7020_H

#include "i2c01.h"
#define SI7020_I2C_ADDRESS    (0x40)  // 1000000x
#define POLYVAL               (0x131)  
  
// Sensor selection/deselection
#define SI7020_SELECT()     i2c_select(BOARD_IOID_SDA, BOARD_IOID_SCL, SI7020_I2C_ADDRESS,I2C_SPEED_NORMAL, I2C_PULL_DOWN)//board_i2c_select(BOARD_I2C_INTERFACE_0,SI7020_I2C_ADDRESS)
#define SI7020_DESELECT()   i2c_deselect()


#ifdef __cplusplus
extern "C"
{
#endif



/*******************************************************************************
 * @fn          sensorTmpSi7020Init
 *
 * @brief       Initialise the temperature sensor driver
 *
 * @return      none
 ******************************************************************************/
bool sensorTmpSi7020Init(void);
  

/*******************************************************************************
 * @fn          sensorSi7020_Get_RH
 *
 * @brief       Initialise the temperature sensor driver
 *
 * @return      none
 ******************************************************************************/
uint8_t *  sensorSi7020_Get_RH_Temp(void);
/*******************************************************************************
 * @fn          sensorSi7020_Get_Temp
 *
 * @brief       Initialise the temperature sensor driver
 *
 * @return      none
 ******************************************************************************/
bool sensorSi7020_Get_Temp(void);
/*******************************************************************************
 * @fn          sensorSi7020_Get_Temp
 *
 * @brief       Initialise the temperature sensor driver
 *
 * @return      none
 ******************************************************************************/
void sensorSi7020_Convert(uint8_t* temp);


void SensorSi7020_Converted_RH_Temp(uint8_t * Si7020_RawData,float *HR,float *Temp);
/*******************************************************************************
 * @fn          SensorSi7020_Converted_RH_Temp
 *
 * @brief       Initialise the temperature sensor driver
 *
 * @return      none
 ******************************************************************************/

/*---------------------------------------------------------------------------*/
#define SI_7020_SENSOR_TYPE_TEMP    1
#define SI_7020_SENSOR_TYPE_HR   	2
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor si7020_sensor;
/*---------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* SI7020_H */







