/*
 * Driver MAX30101
 *
 * 
 */

#include "stm32l4xx_hal.h"
extern I2C_HandleTypeDef hi2c1;
volatile extern char ppgReady;


#define MAX30101_I2C_ADDRESS		0x57

#define MAX30101_REG_INT_STS1		0x00
#define MAX30101_REG_INT_STS2		0x01
#define MAX30101_REG_INT_EN1		0x02
#define MAX30101_REG_INT_EN2		0x03

#define MAX30101_REG_FIFO_OVF		0x05
#define MAX30101_REG_FIFO_CFG		0x08 // FIFO Configuration
#define MAX30101_REG_SPO2_CFG		0x0a
#define MAX30101_REG_LED1_PA		0x0c
#define MAX30101_REG_LED2_PA		0x0d
#define MAX30101_REG_LED3_PA		0x0e
#define MAX30101_REG_PILOT_PA		0x10 // Proximity mode LED amplitude
#define MAX30101_REG_MULTI_LED		0x11
#define MAX30101_REG_TINT		0x1f // Temperature_Integer
#define MAX30101_REG_TFRAC		0x20 // Temperature_Fraction
#define MAX30101_REG_TEMP_CFG		0x21 // Die Temperature Config
#define MAX30101_REG_PROX_INT		0x30 // Proximity interrupt threshold
#define MAX30101_REG_REV_ID		0xfe
#define MAX30101_REG_PART_ID		0xff

#define MAX30101_INT_PPG_MASK		(1 << 6)

#define MAX30101_FIFO_CFG_SMP_AVE_SHIFT		5
#define MAX30101_FIFO_CFG_FIFO_FULL_SHIFT	0
#define MAX30101_FIFO_CFG_ROLLOVER_EN_MASK	(1 << 4)

#define MAX30101_MODE_CFG_SHDN_MASK	(1 << 7)
#define MAX30101_MODE_CFG_RESET_MASK	(1 << 6)

#define MAX30101_SPO2_ADC_RGE_SHIFT	5
#define MAX30101_SPO2_SR_SHIFT		2
#define MAX30101_SPO2_PW_SHIFT		0

#define MAX30101_PART_ID		0x15

#define MAX30101_BYTES_PER_CHANNEL	3
#define MAX30101_MAX_NUM_CHANNELS	3
#define MAX30101_MAX_BYTES_PER_SAMPLE	(MAX30101_MAX_NUM_CHANNELS * \
					 MAX30101_BYTES_PER_CHANNEL)

#define MAX30101_SLOT_LED_MASK		0x03

#define MAX30101_FIFO_DATA_BITS		18
#define MAX30101_FIFO_DATA_MASK		((1 << MAX30101_FIFO_DATA_BITS) - 1)
#define SENSOR_ERROR 0
#define SENSOR_SUCCESS 1



enum max30101_mode {
	MAX30101_MODE_HEART_RATE	= 2,
	MAX30101_MODE_SPO2		= 3,
	MAX30101_MODE_MULTI_LED		= 7,
};

enum max30101_slot {
	MAX30101_SLOT_DISABLED		= 0,
	MAX30101_SLOT_RED_LED1_PA,
	MAX30101_SLOT_IR_LED2_PA,
	MAX30101_SLOT_GREEN_LED3_PA,
	MAX30101_SLOT_RED_PILOT_PA,
	MAX30101_SLOT_IR_PILOT_PA,
	MAX30101_SLOT_GREEN_PILOT_PA,
};

enum max30101_led_channel {
	MAX30101_LED_CHANNEL_RED	= 0,
	MAX30101_LED_CHANNEL_IR,
	MAX30101_LED_CHANNEL_GREEN,
};

enum max30101_pw {
	MAX30101_PW_15BITS		= 0,
	MAX30101_PW_16BITS,
	MAX30101_PW_17BITS,
	MAX30101_PW_18BITS,
};

struct max30101_config {
	uint8_t fifo;
	uint8_t modefunc;
	uint8_t led_pa[MAX30101_MAX_NUM_CHANNELS];
	enum max30101_mode mode;
	enum max30101_slot slot[4];
};

struct max30101_data {
	struct device *i2c;
	uint32_t raw[MAX30101_MAX_NUM_CHANNELS];
	uint8_t map[MAX30101_MAX_NUM_CHANNELS];
	uint8_t num_channels;
};

//Sensor Configuration Functions
int max30101_init(void);
int max30101_sample_fetch(uint8_t *fifo_data, uint8_t size);
int max30101_off(void);
int max30101_on(void);