#ifndef __I2C_PROTOCOL_H__
#define __I2C_PROTOCOL_H__

#include <stdint.h>

void i2c_protocol_init(void);
void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len);

#endif
