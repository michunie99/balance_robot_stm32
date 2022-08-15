/*
 * ICM20948_I2C.c
 *
 *  Created on: Jul 20, 2022
 *      Author: michunie
 */

#include "ICM20948_I2C.h"

uint8_t init_imc20948(IMC20498 *dev, I2C_HandleTypeDef *i2cHandle){

	/* Set up structure */
	dev->i2cHandle 			= i2cHandle;

	dev->acc_g[0]			= 0.0f;
	dev->acc_g[1]			= 0.0f;
	dev->acc_g[2]			= 0.0f;

	dev->gyro_dps[0]		= 0.0f;
	dev->gyro_dps[1]		= 0.0f;
	dev->gyro_dps[2]		= 0.0f;

	/* Store number of transactions errors */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/* Check device ID */
	uint8_t regData;

	status = imc20948_register_read(dev, B0_WHO_AM_I, &regData);
	errNum += (status != HAL_OK);

	if (regData != IMC20948_DEVICE_ID){

		return 255;

	}

	/* Gyroscope configuration */

	status = imc20948_select_bank(dev, 2); // Select USER_BANK_2
	if(status != HAL_OK){

		return 254;

	}

//	/*
//	 * Gyroscope settings
//	 * Scale 					+/- 500 dps
//	 * Low pass					361.4 Hz 3db BW
//	 * Sample rate 			 	102.27 Hz
//	*/
	regData = 0x3B;

//	/*
//	 * Gyroscope settings
//	 * Scale 					+/- 500 dps
//	 * Low pass					OFF
//	 * Sample rate 			 	9 kHz
//	*/
//	regData = 0x3A;

	status = imc20948_register_write(dev, B2_GYRO_CONFIG_1, &regData);
	errNum += (status != HAL_OK);

	/* Uncomment for the first setting */
	regData = 0x0A;
	status = imc20948_register_write(dev, B2_GYRO_SMPLRT_DIV, &regData);
	errNum += (status != HAL_OK);

//	/*
//	 * Acceleration settings
//	 * Scale 					+/- 2 g
//	 * Low pass					361.4 Hz 3db BW
//	 * Sample rate 			 	102.27 Hz
//	*/
//
	regData = 0x19;

	/*
	 * Acceleration settings
	 * Scale 					+/- 2 g
	 * Low pass					OFF
	 * Sample rate 			 	4.5 kHz
	*/
//	regData = 0x18;
	status = imc20948_register_write(dev, B2_ACCEL_CONFIG, &regData);
	errNum += (status != HAL_OK);

	/* Uncomment for the first setting */
	regData = 0x0A;
	status = imc20948_register_write(dev, B2_ACCEL_SMPLRT_DIV_2, &regData);
	errNum += (status != HAL_OK);

	status = imc20948_select_bank(dev, 0); // Select USER_BANK_0
	if(status != HAL_OK){

		return 254;

	}
	/* Wake up chip */
	regData = 0x01;
	status = imc20948_register_write(dev, B0_PWR_MGMT_1, &regData);
	errNum += (status != HAL_OK);

	/* Enable data ready interrupt */
	regData = 0x01;
	status = imc20948_register_write(dev, B0_INT_ENABLE_1, &regData);
	errNum += (status != HAL_OK);

	/* Return errors */
	return errNum;

}

HAL_StatusTypeDef imc20948_acc_read(IMC20498 *dev){

	HAL_StatusTypeDef status;
	uint8_t regData[6];

	status = imc20948_select_bank(dev, 0); // Select USER_BANK 0

	status = imc20948_registers_read(dev, B0_ACCEL_XOUT_H, regData, 6);

	int16_t raw_acc[3];

	raw_acc[0] = ((int16_t) regData[0] << 8) | regData[1];
	raw_acc[1] = ((int16_t) regData[2] << 8) | regData[3];
	raw_acc[2] = ((int16_t) regData[4] << 8) | regData[5];

	/* Convert raw to G acceleration (for 2G range divide by 16,384)*/
	dev->acc_g[0] = (float) raw_acc[0]/16384;
	dev->acc_g[1] = (float)	raw_acc[1]/16384;
	dev->acc_g[2] = (float)	raw_acc[2]/16384;

	return status;

}

HAL_StatusTypeDef imc20948_gyro_read(IMC20498 *dev){

	HAL_StatusTypeDef status;
	uint8_t regData[6];

	status = imc20948_select_bank(dev, 0); // Select USER_BANK 0

	status = imc20948_registers_read(dev, B0_GYRO_XOUT_H, regData, 6);

	int16_t raw_acc[3];

	raw_acc[0] = ((int16_t) regData[0] << 8) | regData[1];
	raw_acc[1] = ((int16_t) regData[2] << 8) | regData[3];
	raw_acc[2] = ((int16_t) regData[4] << 8) | regData[5];

	/* Convert raw to dps readings (for 2G range divide by 65.5)*/
	dev->gyro_dps[0] = raw_acc[0]/65.5;
	dev->gyro_dps[1] = raw_acc[1]/65.5;
	dev->gyro_dps[2] = raw_acc[2]/65.5;

	return status;
}

/* Low level functions */

HAL_StatusTypeDef imc20948_register_read(IMC20498 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Read(dev->i2cHandle, IMC20948_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}

HAL_StatusTypeDef imc20948_registers_read(IMC20498 *dev, uint8_t reg, uint8_t *data, uint8_t len){

	return HAL_I2C_Mem_Read(dev->i2cHandle, IMC20948_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);

}

HAL_StatusTypeDef imc20948_register_write(IMC20498 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Write(dev->i2cHandle, IMC20948_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}

HAL_StatusTypeDef imc20948_select_bank(IMC20498 *dev, uint8_t bank_num){

	uint8_t data = 0;

	data += (bank_num << 4);

	return HAL_I2C_Mem_Write(dev->i2cHandle, IMC20948_I2C_ADDR, REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}
