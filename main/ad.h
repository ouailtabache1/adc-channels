/*
 * ad.h
 *
 *  Created on: Oct 18, 2022
 *      Author: zamek
 */

#ifndef MAIN_AD_H_
#define MAIN_AD_H_

#include <stdint.h>
#include "freertos/FreeRTOS.h"

//!!! Define the channel number


#define AD_MAX (4096)
#define AD_MIN (0)

BaseType_t ad_init();

//!!! pass channel index to function & use it
BaseType_t ad_get(uint16_t *value, TickType_t ticks, uint8_t ch);

//!!! pass channel index to function & use it
BaseType_t ad_get_temperature(float *value, TickType_t ticks,uint8_t ch);

BaseType_t ad_deinit(uint8_t ch);




#endif /* MAIN_AD_H_ */
