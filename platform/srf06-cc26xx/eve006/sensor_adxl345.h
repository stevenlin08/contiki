#ifndef SENSOR_ADXL345_H
#define SENSOR_ADXL345_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "stdint.h"

/*********************************************************************
 * CONSTANTS
 */
#define ADXL345_RATE_3200           0x0F
#define ADXL345_RATE_1600           0x0E
#define ADXL345_RATE_800            0x0D
#define ADXL345_RATE_400            0x0C
#define ADXL345_RATE_200            0x0B
#define ADXL345_RATE_100            0x0A
#define ADXL345_RATE_50             0x09
#define ADXL345_RATE_25             0x08
#define ADXL345_RATE_12P5           0x07
#define ADXL345_RATE_6P25           0x06
#define ADXL345_RATE_3P13           0x05
#define ADXL345_RATE_1P56           0x04
#define ADXL345_RATE_0P78           0x03
#define ADXL345_RATE_0P39           0x02
#define ADXL345_RATE_0P20           0x01
#define ADXL345_RATE_0P10           0x00

#define ADXL345_WAKEUP_8HZ          0x00
#define ADXL345_WAKEUP_4HZ          0x01
#define ADXL345_WAKEUP_2HZ          0x02
#define ADXL345_WAKEUP_1HZ          0x03

#define ADXL345_RANGE_2G            0x00
#define ADXL345_RANGE_4G            0x01
#define ADXL345_RANGE_8G            0x02
#define ADXL345_RANGE_16G           0x03

#define ADXL345_FIFO_MODE_BYPASS    0x00
#define ADXL345_FIFO_MODE_FIFO      0x01
#define ADXL345_FIFO_MODE_STREAM    0x02
#define ADXL345_FIFO_MODE_TRIGGER   0x03


/*********************************************************************
 * TYPEDEFS
 */
typedef struct _accel_data
{
	signed int x_raw;
	signed int y_raw;
	signed int z_raw;
	signed int x_offset;
	signed int y_offset;
	signed int z_offset;
	signed int x;
	signed int y;
	signed int z;
	unsigned int ready;
}accel_data_t;
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor adxl345_sensor;
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
#define ADXL_345_SENSOR_TYPE_READ    1

/*---------------------------------------------------------------------------*/
/*********************************************************************
 * FUNCTIONS
 */

bool Accelerometer_Init(void);
bool Accelerometer_Read(accel_data_t*);
//extern void Accelerometer_Standby(void);
//extern void Accelerometer_Wake(void);
bool Accelerometer_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_ADXL345_H */

