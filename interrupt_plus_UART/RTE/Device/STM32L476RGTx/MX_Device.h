/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 18/01/2018 12:06:09
 * Description : STM32Cube MX parameter definitions
 * Note        : This file is generated by STM32CubeMX (DO NOT EDIT!)
 ******************************************************************************/

#ifndef __MX_DEVICE_H
#define __MX_DEVICE_H

/*---------------------------- Clock Configuration ---------------------------*/

#define MX_LSI_VALUE                            32000
#define MX_LSE_VALUE                            32768
#define MX_HSI_VALUE                            16000000
#define MX_HSE_VALUE                            8000000
#define MX_SYSCLKFreq_VALUE                     4000000
#define MX_HCLKFreq_Value                       4000000
#define MX_FCLKCortexFreq_Value                 4000000
#define MX_CortexFreq_Value                     4000000
#define MX_AHBFreq_Value                        4000000
#define MX_APB1Freq_Value                       4000000
#define MX_APB2Freq_Value                       4000000
#define MX_APB1TimFreq_Value                    4000000
#define MX_APB2TimFreq_Value                    4000000
#define MX_PWRFreq_Value                        4000000
#define MX_RTCFreq_Value                        32000
#define MX_USBFreq_Value                        16000000
#define MX_WatchDogFreq_Value                   32000
#define MX_MCO1PinFreq_Value                    4000000

/*-------------------------------- I2C1       --------------------------------*/

#define MX_I2C1                                 1

/* GPIO Configuration */

/* Pin PB6 */
#define MX_I2C1_SCL_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_I2C1_SCL_GPIO_FM6                    __NULL
#define MX_I2C1_SCL_Pin                         PB6
#define MX_I2C1_SCL_GPIOx                       GPIOB
#define MX_I2C1_SCL_GPIO_PuPdOD                 GPIO_PULLUP
#define MX_I2C1_SCL_GPIO_Pin                    GPIO_PIN_6
#define MX_I2C1_SCL_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SCL_GPIO_Mode                   GPIO_MODE_AF_OD

/* Pin PB7 */
#define MX_I2C1_SDA_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_I2C1_SDA_Pin                         PB7
#define MX_I2C1_SDA_GPIOx                       GPIOB
#define MX_I2C1_SDA_GPIO_PuPdOD                 GPIO_PULLUP
#define MX_I2C1_SDA_GPIO_Pin                    GPIO_PIN_7
#define MX_I2C1_SDA_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SDA_GPIO_FM7                    __NULL
#define MX_I2C1_SDA_GPIO_Mode                   GPIO_MODE_AF_OD

/*-------------------------------- SYS        --------------------------------*/

#define MX_SYS                                  1

/* GPIO Configuration */

/* Pin PA13 (JTMS-SWDIO) */
#define MX_SYS_JTMS-SWDIO_Pin                   PA13_(JTMS_SWDIO)

/* Pin PA14 (JTCK-SWCLK) */
#define MX_SYS_JTCK-SWCLK_Pin                   PA14_(JTCK_SWCLK)

/*-------------------------------- UART4      --------------------------------*/

#define MX_UART4                                1

#define MX_UART4_VM                             VM_ASYNC

/* GPIO Configuration */

/* Pin PA1 */
#define MX_UART4_RX_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_UART4_RX_Pin                         PA1
#define MX_UART4_RX_GPIOx                       GPIOA
#define MX_UART4_RX_GPIO_PuPd                   GPIO_PULLUP
#define MX_UART4_RX_GPIO_Pin                    GPIO_PIN_1
#define MX_UART4_RX_GPIO_AF                     GPIO_AF8_UART4
#define MX_UART4_RX_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PA0 */
#define MX_UART4_TX_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_UART4_TX_Pin                         PA0
#define MX_UART4_TX_GPIOx                       GPIOA
#define MX_UART4_TX_GPIO_PuPd                   GPIO_PULLUP
#define MX_UART4_TX_GPIO_Pin                    GPIO_PIN_0
#define MX_UART4_TX_GPIO_AF                     GPIO_AF8_UART4
#define MX_UART4_TX_GPIO_Mode                   GPIO_MODE_AF_PP

/*-------------------------------- NVIC       --------------------------------*/

#define MX_NVIC                                 1

/*-------------------------------- GPIO       --------------------------------*/

#define MX_GPIO                                 1

/* GPIO Configuration */

/* Pin PA10 */
#define MX_PA10_Pin                             PA10
#define MX_PA10_GPIOx                           GPIOA
#define MX_PA10_GPIO_PuPd                       GPIO_NOPULL
#define MX_PA10_GPIO_Pin                        GPIO_PIN_10
#define MX_PA10_GPIO_ModeDefaultEXTI            GPIO_MODE_IT_FALLING

#endif  /* __MX_DEVICE_H */

