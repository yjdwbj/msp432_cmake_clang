/*
 * dht11.h
 *
 *  Created on: Jan 4, 2020
 *      Author: michael
 */

#ifndef DHT11_H_
#define DHT11_H_
#include <stdint.h>

/* TI-RTOS Header files */

extern uint8_t DHT11_Read_Data(uint8_t *temp,uint8_t *humi);
void DHT11_Reset();
uint8_t DHT11_Read_Byte(void);


#endif /* DHT11_H_ */
