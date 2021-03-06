/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 12/02/2018 14:04:14
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
#define MX_SYSCLKFreq_VALUE                     80000000
#define MX_HCLKFreq_Value                       80000000
#define MX_FCLKCortexFreq_Value                 80000000
#define MX_CortexFreq_Value                     80000000
#define MX_AHBFreq_Value                        80000000
#define MX_APB1Freq_Value                       80000000
#define MX_APB2Freq_Value                       80000000
#define MX_APB1TimFreq_Value                    80000000
#define MX_APB2TimFreq_Value                    80000000
#define MX_PWRFreq_Value                        80000000
#define MX_RTCFreq_Value                        32000
#define MX_USBFreq_Value                        64000000
#define MX_WatchDogFreq_Value                   32000
#define MX_MCO1PinFreq_Value                    80000000

/*-------------------------------- QUADSPI    --------------------------------*/

#define MX_QUADSPI                              1

/* GPIO Configuration */

/* Pin PB10 */
#define MX_QUADSPI_CLK_GPIO_Speed               GPIO_SPEED_FREQ_VERY_HIGH
#define MX_QUADSPI_CLK_Pin                      PB10
#define MX_QUADSPI_CLK_GPIOx                    GPIOB
#define MX_QUADSPI_CLK_GPIO_PuPd                GPIO_NOPULL
#define MX_QUADSPI_CLK_GPIO_Pin                 GPIO_PIN_10
#define MX_QUADSPI_CLK_GPIO_AF                  GPIO_AF10_QUADSPI
#define MX_QUADSPI_CLK_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PA6 */
#define MX_QUADSPI_BK1_IO3_GPIO_Speed           GPIO_SPEED_FREQ_VERY_HIGH
#define MX_QUADSPI_BK1_IO3_Pin                  PA6
#define MX_QUADSPI_BK1_IO3_GPIOx                GPIOA
#define MX_QUADSPI_BK1_IO3_GPIO_PuPd            GPIO_NOPULL
#define MX_QUADSPI_BK1_IO3_GPIO_Pin             GPIO_PIN_6
#define MX_QUADSPI_BK1_IO3_GPIO_AF              GPIO_AF10_QUADSPI
#define MX_QUADSPI_BK1_IO3_GPIO_Mode            GPIO_MODE_AF_PP

/* Pin PA7 */
#define MX_QUADSPI_BK1_IO2_GPIO_Speed           GPIO_SPEED_FREQ_VERY_HIGH
#define MX_QUADSPI_BK1_IO2_Pin                  PA7
#define MX_QUADSPI_BK1_IO2_GPIOx                GPIOA
#define MX_QUADSPI_BK1_IO2_GPIO_PuPd            GPIO_NOPULL
#define MX_QUADSPI_BK1_IO2_GPIO_Pin             GPIO_PIN_7
#define MX_QUADSPI_BK1_IO2_GPIO_AF              GPIO_AF10_QUADSPI
#define MX_QUADSPI_BK1_IO2_GPIO_Mode            GPIO_MODE_AF_PP

/* Pin PB0 */
#define MX_QUADSPI_BK1_IO1_GPIO_Speed           GPIO_SPEED_FREQ_VERY_HIGH
#define MX_QUADSPI_BK1_IO1_Pin                  PB0
#define MX_QUADSPI_BK1_IO1_GPIOx                GPIOB
#define MX_QUADSPI_BK1_IO1_GPIO_PuPd            GPIO_NOPULL
#define MX_QUADSPI_BK1_IO1_GPIO_Pin             GPIO_PIN_0
#define MX_QUADSPI_BK1_IO1_GPIO_AF              GPIO_AF10_QUADSPI
#define MX_QUADSPI_BK1_IO1_GPIO_Mode            GPIO_MODE_AF_PP

/* Pin PB11 */
#define MX_QUADSPI_NCS_GPIO_Speed               GPIO_SPEED_FREQ_VERY_HIGH
#define MX_QUADSPI_NCS_Pin                      PB11
#define MX_QUADSPI_NCS_GPIOx                    GPIOB
#define MX_QUADSPI_NCS_GPIO_PuPd                GPIO_NOPULL
#define MX_QUADSPI_NCS_GPIO_Pin                 GPIO_PIN_11
#define MX_QUADSPI_NCS_GPIO_AF                  GPIO_AF10_QUADSPI
#define MX_QUADSPI_NCS_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PB1 */
#define MX_QUADSPI_BK1_IO0_GPIO_Speed           GPIO_SPEED_FREQ_VERY_HIGH
#define MX_QUADSPI_BK1_IO0_Pin                  PB1
#define MX_QUADSPI_BK1_IO0_GPIOx                GPIOB
#define MX_QUADSPI_BK1_IO0_GPIO_PuPd            GPIO_NOPULL
#define MX_QUADSPI_BK1_IO0_GPIO_Pin             GPIO_PIN_1
#define MX_QUADSPI_BK1_IO0_GPIO_AF              GPIO_AF10_QUADSPI
#define MX_QUADSPI_BK1_IO0_GPIO_Mode            GPIO_MODE_AF_PP

/*-------------------------------- SYS        --------------------------------*/

#define MX_SYS                                  1

/* GPIO Configuration */

/* Pin PA13 (JTMS-SWDIO) */
#define MX_SYS_JTMS-SWDIO_Pin                   PA13_(JTMS_SWDIO)

/* Pin PA14 (JTCK-SWCLK) */
#define MX_SYS_JTCK-SWCLK_Pin                   PA14_(JTCK_SWCLK)

/*-------------------------------- NVIC       --------------------------------*/

#define MX_NVIC                                 1

/*-------------------------------- GPIO       --------------------------------*/

#define MX_GPIO                                 1

/* GPIO Configuration */

/* Pin PC15-OSC32_OUT (PC15) */
#define MX_PC15_OSC32_OUT_(PC15)_GPIO_Speed     GPIO_SPEED_FREQ_LOW
#define MX_PC15_OSC32_OUT_(PC15)_Pin            PC15_OSC32_OUT_(PC15)
#define MX_PC15_OSC32_OUT_(PC15)_GPIOx          GPIOC
#define MX_PC15_OSC32_OUT_(PC15)_PinState       GPIO_PIN_RESET
#define MX_PC15_OSC32_OUT_(PC15)_GPIO_PuPd      GPIO_NOPULL
#define MX_PC15_OSC32_OUT_(PC15)_GPIO_Pin       GPIO_PIN_15
#define MX_PC15_OSC32_OUT_(PC15)_GPIO_ModeDefaultOutputPP GPIO_MODE_OUTPUT_PP

/* Pin PC14-OSC32_IN (PC14) */
#define MX_PC14_OSC32_IN_(PC14)_GPIO_Speed      GPIO_SPEED_FREQ_LOW
#define MX_PC14_OSC32_IN_(PC14)_Pin             PC14_OSC32_IN_(PC14)
#define MX_PC14_OSC32_IN_(PC14)_GPIOx           GPIOC
#define MX_PC14_OSC32_IN_(PC14)_PinState        GPIO_PIN_RESET
#define MX_PC14_OSC32_IN_(PC14)_GPIO_PuPd       GPIO_NOPULL
#define MX_PC14_OSC32_IN_(PC14)_GPIO_Pin        GPIO_PIN_14
#define MX_PC14_OSC32_IN_(PC14)_GPIO_ModeDefaultOutputPP GPIO_MODE_OUTPUT_PP

#endif  /* __MX_DEVICE_H */

