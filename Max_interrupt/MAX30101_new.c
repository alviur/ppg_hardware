/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "C:\Users\Lex-alviur\Desktop\Max_interrupt\max30101_new.h.h"


uint8_t MAX30101_REG_MODE_CFG = 0x09;
uint8_t MAX30101_REG_FIFO_DATA	=	0x07;
uint8_t MAX30101_REG_FIFO_RD =	0x06;
uint8_t MAX30101_REG_FIFO_WR = 0x04;
HAL_StatusTypeDef returnValue;
uint8_t address;

/*******************************************************************************
* Function Name		: max30101_sample_fetch
* Description		: 				
* Input			: void
* Output		: zero
* Return		: Status [SENSOR_ERROR, SENSOR_SUCCESS]
*******************************************************************************/


uint32_t max30101_sample_fetch(uint32_t *fifo_data)
{
	
		
	uint8_t buffer[MAX30101_MAX_BYTES_PER_SAMPLE];
	uint8_t bufferRead[4],bufferWrite[4];
	uint32_t fifo_data2;
	int fifo_chan;
	int num_bytes;
	int i;
			

	
	// Read FIFO read pointer
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,&MAX30101_REG_FIFO_RD, 1,100);
	
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}else{
		
		HAL_I2C_Master_Receive(&hi2c1, MAX30101_I2C_ADDRESS<<1, bufferRead,	4,100);
		
	}

//	// Read FIFO write pointer
//	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,&MAX30101_REG_FIFO_WR, 1,100);
//	
//	if (returnValue != HAL_OK) {
//		return SENSOR_ERROR;
//	}else{
//		
//		HAL_I2C_Master_Receive(&hi2c1, MAX30101_I2C_ADDRESS<<1, bufferWrite,	4,100);
//		
//	}
	// Read all the active channels for one sample 
	num_bytes = MAX30101_BYTES_PER_CHANNEL;
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,&MAX30101_REG_FIFO_RD, 1,100);
	
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}else{
		
		HAL_I2C_Master_Receive(&hi2c1, MAX30101_I2C_ADDRESS<<1, buffer,	num_bytes,100);
		
	}

	fifo_chan = 0;
	for (i = 0; i < num_bytes; i += 3) {
		// Each channel is 18-bits 
		*fifo_data = (buffer[i] << 16) | (buffer[i + 1] << 8) |
			    (buffer[i + 2]);
		*fifo_data &= MAX30101_FIFO_DATA_MASK;

		// Save the raw data 
		//data->raw[fifo_chan++] = fifo_data;
	}

//	// Set ppgReady flag to 0 again
//	ppgReady = 0;	
//	// Read Interrupt status register
//	address= MAX30101_REG_INT_STS1;
//	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,&address, 1,100);
//	
//	if (returnValue != HAL_OK) {
//		return SENSOR_ERROR;
//	}
//	else{
//		
//		HAL_I2C_Master_Receive(&hi2c1, MAX30101_I2C_ADDRESS<<1, &bufferRead[0],	1,100);
//		
//	}

	

	return fifo_data2;
}



/*******************************************************************************
* Function Name		: max30101_init
* Description		: Initialize MAX30101 sensor				
* Input			: void
* Output		: zero
* Return		: Status [SENSOR_ERROR, SENSOR_SUCCESS]
*******************************************************************************/

int max30101_init(void)
{
	const struct max30101_config *config;
	struct max30101_data *data;
	uint8_t part_id;
	uint8_t mode_cfg;
	uint32_t led_chan;	
	int fifo_chan;

	

	/* Reset the sensor */
  uint8_t dataBuffer[2] = {0, 0};
	dataBuffer[0] = MAX30101_REG_MODE_CFG;
	dataBuffer[1] = MAX30101_MODE_CFG_RESET_MASK;
	

	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 2,100);

	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}
	
	/* Wait for reset to be cleared */	
	do {

		returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,&MAX30101_REG_MODE_CFG, 1,100);			
		

		if(returnValue == HAL_OK){
			HAL_I2C_Master_Receive(&hi2c1, MAX30101_I2C_ADDRESS<<1, &mode_cfg,	1,100);	
		}else{	
			return SENSOR_ERROR;
		}

	} while (mode_cfg & MAX30101_MODE_CFG_RESET_MASK);
	
	// checking id device
		address=MAX30101_REG_PART_ID;
		returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,&address, 1,100);
			
		if(returnValue != HAL_OK){
			return SENSOR_ERROR;
		}
		
		HAL_I2C_Master_Receive(&hi2c1, MAX30101_I2C_ADDRESS<<1, dataBuffer,	1,100);
		if(dataBuffer[0]!=	MAX30101_PART_ID){
			return SENSOR_ERROR;
		}
		
	// Clear FIFO Read
	dataBuffer[0] = MAX30101_REG_FIFO_RD;
	dataBuffer[1] = 0x00;
	
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 2,100);

	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}

	// Clear FIFO Write
	dataBuffer[0] = MAX30101_REG_FIFO_WR;
	dataBuffer[1] = 0x00;
	
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 2,100);

	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}		
	
	// Write the FIFO configuration register 
	dataBuffer[0] = MAX30101_REG_FIFO_CFG;
	dataBuffer[1] = 0x00000000;

	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 2,100);
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}

	/* Write the mode configuration register */
	dataBuffer[0] = MAX30101_REG_MODE_CFG;
	dataBuffer[1] = 0x82; // 0x00000010 = HR
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 2,100);
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}
	
	// Write the Interrupt Enable register
	dataBuffer[0] = MAX30101_REG_INT_EN1;
	dataBuffer[1] = 0x40; // 0x40
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 2,100);
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}


	// Read Interrupt status register
	address= MAX30101_REG_INT_EN1;
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,&address, 1,100);
	
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}else{
		
		HAL_I2C_Master_Receive(&hi2c1, MAX30101_I2C_ADDRESS<<1, dataBuffer,	1,100);
		
	}
	
	// Write the SpO2 configuration register 
	dataBuffer[0] = MAX30101_REG_SPO2_CFG;
	dataBuffer[1] = 0x3;
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 2,100);
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}


	/* Write the LED pulse amplitude registers */
	dataBuffer[0] = MAX30101_REG_LED1_PA;
	dataBuffer[1] =  0x7f;
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 2,100);
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}

	dataBuffer[0] = MAX30101_REG_LED2_PA;
	dataBuffer[1] =  0;
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 2,100);
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}

	dataBuffer[0] = MAX30101_REG_LED3_PA;
	dataBuffer[1] =  0;
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 2,100);
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}


//	/* Initialize the channel map and active channel count */
//	data->num_channels = 0;
//	for (led_chan = 0; led_chan < MAX30101_MAX_NUM_CHANNELS; led_chan++) {
//		data->map[led_chan] = MAX30101_MAX_NUM_CHANNELS;
//	}

//	/* Count the number oD channel number (red/ir/green) to the fifo channel number.
//	 */
//	for (fifo_chan = 0; fifo_chan < MAX30101_MAX_NUM_CHANNELS;
//	     fifo_chan++) {
//		led_chan = (config->slot[fifo_chan] & MAX30101_SLOT_LED_MASK)-1;
//		if (led_chan < MAX30101_MAX_NUM_CHANNELS) {
//			data->map[led_chan] = fifo_chan;
//			data->num_channels++;
//		}
//	}



/*
// Definition of configuration structures

static struct max30101_config max30101_config = {
	.fifo = (CONFIG_MAX30101_SMP_AVE << MAX30101_FIFO_CFG_SMP_AVE_SHIFT) |
#ifdef CONFIG_MAX30101_FIFO_ROLLOVER_EN
		MAX30101_FIFO_CFG_ROLLOVER_EN_MASK |
#endif
		(CONFIG_MAX30101_FIFO_A_FULL <<
		 MAX30101_FIFO_CFG_FIFO_FULL_SHIFT),

#if defined(CONFIG_MAX30101_HEART_RATE_MODE)
	.mode = MAX30101_MODE_HEART_RATE,
	.slot[0] = MAX30101_SLOT_RED_LED1_PA,
	.slot[1] = MAX30101_SLOT_DISABLED,
	.slot[2] = MAX30101_SLOT_DISABLED,
	.slot[3] = MAX30101_SLOT_DISABLED,
#elif defined(CONFIG_MAX30101_SPO2_MODE)
	.mode = MAX30101_MODE_SPO2,
	.slot[0] = MAX30101_SLOT_RED_LED1_PA,
	.slot[1] = MAX30101_SLOT_IR_LED2_PA,
	.slot[2] = MAX30101_SLOT_DISABLED,
	.slot[3] = MAX30101_SLOT_DISABLED,
#else
	.mode = MAX30101_MODE_MULTI_LED,
	.slot[0] = CONFIG_MAX30101_SLOT1,
	.slot[1] = CONFIG_MAX30101_SLOT2,
	.slot[2] = CONFIG_MAX30101_SLOT3,
	.slot[3] = CONFIG_MAX30101_SLOT4,
#endif

	.spo2 = (CONFIG_MAX30101_ADC_RGE << MAX30101_SPO2_ADC_RGE_SHIFT) |
		(CONFIG_MAX30101_SR << MAX30101_SPO2_SR_SHIFT) |
		(MAX30101_PW_18BITS << MAX30101_SPO2_PW_SHIFT),

	.led_pa[0] = CONFIG_MAX30101_LED1_PA,
	.led_pa[1] = CONFIG_MAX30101_LED2_PA,
	.led_pa[2] = CONFIG_MAX30101_LED3_PA,
};
*/
static struct max30101_data max30101_data;
}


int max30101_off(void){
	uint8_t dataBuffer[2] = {0, 0};
	
	/* Write the mode configuration register */
	dataBuffer[0] = MAX30101_REG_MODE_CFG;
	
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 1,100);
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}
	
	returnValue = HAL_I2C_Master_Receive(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer+1, 1,100);
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}
	
	dataBuffer[1]=dataBuffer[1]|0x80; //0x80 turn off, 0x00 turn on
	
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 2,100);
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}
}

int max30101_on(void){
	uint8_t dataBuffer[2] = {0, 0};
	
	/* Write the mode configuration register */
	dataBuffer[0] = MAX30101_REG_MODE_CFG;
	
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 1,100);
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}
	
	returnValue = HAL_I2C_Master_Receive(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer+1, 1,100);
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}
	
	dataBuffer[1]=dataBuffer[1]&~0x80; //0x80 turn off, 0x00 turn on
	
	returnValue = HAL_I2C_Master_Transmit(&hi2c1,MAX30101_I2C_ADDRESS<<1,dataBuffer, 2,100);
	if (returnValue != HAL_OK) {
		return SENSOR_ERROR;
	}
}



