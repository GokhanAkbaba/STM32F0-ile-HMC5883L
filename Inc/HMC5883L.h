/*
 * HMC5883L.h
 *
 *  Created on: 5 Ara 2018
 *      Author: gökhan
 */

#ifndef HMC5883L_H_
#define HMC5883L_H_

#include "math.h"

#define GAUSS_TO_MICROTESLA 100
#define PI 3.14159265359

typedef enum {
	 OUT_X_LSB   = 0x00,
	 OUT_X_MSB   = 0x01,
	 OUT_Y_LSB   = 0x02,
	 OUT_Y_MSB   = 0x03,
	 OUT_Z_LSB   = 0x04,
	 OUT_Z_MSB   = 0x05,
	 STATUS      = 0x06,
	 TEMP_LSB    = 0x07,
	 TEMP_MSB    = 0x08,
	 CONTROL_A   = 0x09,
	 CONTROL_B   = 0x0A,
	 SET_RESET   = 0x0B,
	 CHIP_ID     = 0x0D
}hmc5883MagRegisters_t;

typedef enum
    {
      HMC5883_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
      HMC5883_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
      HMC5883_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
      HMC5883_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
      HMC5883_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
      HMC5883_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
      HMC5883_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
    } hmc5883MagGain;



#endif /* HMC5883L_H_ */
