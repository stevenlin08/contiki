#ifndef I2C01_H_
#define I2C01_H_
/*---------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
/*---------------------------------------------------------------------------*/
#define I2C_SPEED_NORMAL	false
#define I2C_SPEED_FAST		true
#define I2C_PULL_DOWN 		IOC_IOPULL_DOWN
#define I2C_PULL_UP			IOC_IOPULL_UP
#define I2C_PULL_NO_PULL	IOC_NO_IOPULL

void i2c_deselect();
void i2c_select(uint32_t new_pin_sda, uint32_t new_pin_scl, uint8_t new_slave_address, bool new_speed,uint32_t new_pin_pull);
bool i2c_read(uint8_t *buf, uint8_t len);
bool i2c_write(uint8_t *buf, uint8_t len);
bool i2c_write_single(uint8_t data);
bool i2c_write_read(uint8_t *wdata, uint8_t wlen, uint8_t *rdata,uint8_t rlen);

bool i2c_write_single_read_multi(uint8_t wdata, uint8_t *buf, uint8_t len);
bool i2c_write_single_read_multi2(uint8_t wdata, uint8_t *buf, uint8_t len);

void i2c_wakeup(void);
void i2c_shutdown(void);


#endif /* I2C01_H_ */




