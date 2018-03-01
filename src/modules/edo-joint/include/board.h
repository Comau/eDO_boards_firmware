/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */
 
/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for R2P uDC module board.
 */

/*
 * Board identifier.
 */
#define BOARD_NAME                  "edo-joint"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0
#endif

#define STM32_LSEDRV                (3 << 3)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                24000000
#endif

#define STM32_HSE_BYPASS

/*
#define PWM_DRIVER PWMD1
#define QEI_DRIVER QEID4
*/

/*
 * MCU type as defined in the ST header.
 */
#define STM32F303xC

/*
 * IO pins assignments.
 */
#define GPIOA_ENCODER_ANALOG        0
#define GPIOA_BRIDGE_TEMP           1
#define GPIOA_INT1                  2
#define GPIOA_INT2                  3
#define GPIOA_MOTOR_SF              4
#define GPIO_OPAMP_FB               5
#define GPIO_OPAMP_OUT              6
#define GPIOA_CURRENT_SENSE         7
#define GPIOA_MOTOR_IN1             8
#define GPIOA_MOTOR_IN2             9
#define GPIOA_MOTOR_D1              10
#define GPIOA_LED1                  11
#define GPIOA_LED2                  12
#define GPIOA_SWDIO                 13
#define GPIOA_SWCLK                 14
#define GPIOA_PIN15                 15

#define GPIOA_CURRENT_SENSE_FAULT   0
#define GPIOB_MOTOR_EN              1
#define GPIOB_BRAKE                 2
#define GPIOB_ENCODER_I             3
#define GPIOB_ENCODER_A             4
#define GPIOB_ENCODER_B             5
#define GPIOB_SCL                   6
#define GPIOB_SDA                   7
#define GPIOB_CAN_RX                8
#define GPIOB_CAN_TX                9
#define GPIOB_MOTOR_D2              10
#define GPIOB_PIN11                 11
#define GPIOB_ISENSE                12
#define GPIOB_VSENSE                13
#define GPIOB_PIN14                 14
#define GPIOB_SLEW                  15

#define GPIOC_PIN0                  0
#define GPIOC_PIN1                  1
#define GPIOC_PIN2                  2
#define GPIOC_PIN3                  3
#define GPIOC_PIN4                  4
#define GPIOC_PIN5                  5
#define GPIOC_PIN6                  6
#define GPIOC_PIN7                  7
#define GPIOC_PIN8                  8
#define GPIOC_PIN9                  9
#define GPIOC_PIN10                 10
#define GPIOC_PIN11                 11
#define GPIOC_PIN12                 12
#define GPIOC_PIN13                 13
#define GPIOC_OSC32_IN              14
#define GPIOC_OSC32_OUT             15

#define GPIOD_PIN0                  0
#define GPIOD_PIN1                  1
#define GPIOD_PIN2                  2
#define GPIOD_PIN3                  3
#define GPIOD_PIN4                  4
#define GPIOD_PIN5                  5
#define GPIOD_PIN6                  6
#define GPIOD_PIN7                  7
#define GPIOD_PIN8                  8
#define GPIOD_PIN9                  9
#define GPIOD_PIN10                 10
#define GPIOD_PIN11                 11
#define GPIOD_PIN12                 12
#define GPIOD_PIN13                 13
#define GPIOD_PIN14                 14
#define GPIOD_PIN15                 15

#define GPIOE_PIN0                  0
#define GPIOE_PIN1                  1
#define GPIOE_PIN2                  2
#define GPIOE_PIN3                  3
#define GPIOE_PIN4                  4
#define GPIOE_PIN5                  5
#define GPIOE_PIN6                  6
#define GPIOE_PIN7                  7
#define GPIOE_PIN8                  8
#define GPIOE_PIN9                  9
#define GPIOE_PIN10                 10
#define GPIOE_PIN11                 11
#define GPIOE_PIN12                 12
#define GPIOE_PIN13                 13
#define GPIOE_PIN14                 14
#define GPIOE_PIN15                 15

#define GPIOF_PIN0                  0
#define GPIOF_PIN1                  1
#define GPIOF_PIN2                  2
#define GPIOF_PIN3                  3
#define GPIOF_PIN4                  4
#define GPIOF_PIN5                  5
#define GPIOF_PIN6                  6
#define GPIOF_PIN7                  7
#define GPIOF_PIN8                  8
#define GPIOF_PIN9                  9
#define GPIOF_PIN10                 10
#define GPIOF_PIN11                 11
#define GPIOF_PIN12                 12
#define GPIOF_PIN13                 13
#define GPIOF_PIN14                 14
#define GPIOF_PIN15                 15

#define LED_GPIO	GPIOA
#define LED_PIN		11

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

#define VAL_GPIOA_MODER             (PIN_MODE_ANALOG(GPIOA_ENCODER_ANALOG) | \
                                     PIN_MODE_ANALOG(GPIOA_BRIDGE_TEMP) |    \
                                     PIN_MODE_INPUT(GPIOA_INT1) |            \
                                     PIN_MODE_INPUT(GPIOA_INT2) |            \
                                     PIN_MODE_INPUT(GPIOA_MOTOR_SF) |        \
                                     PIN_MODE_ANALOG(GPIO_OPAMP_FB) |        \
                                     PIN_MODE_ANALOG(GPIO_OPAMP_OUT) |       \
                                     PIN_MODE_ANALOG(GPIOA_CURRENT_SENSE) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_MOTOR_IN1) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_MOTOR_IN2) |   \
                                     PIN_MODE_OUTPUT(GPIOA_MOTOR_D1) |       \
                                     PIN_MODE_OUTPUT(GPIOA_LED1) |           \
                                     PIN_MODE_OUTPUT(GPIOA_LED2) |           \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |       \
                                     PIN_MODE_INPUT(GPIOA_PIN15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_ENCODER_ANALOG) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_BRIDGE_TEMP) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_INT1) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_INT2) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_MOTOR_SF) |       \
                                     PIN_OTYPE_PUSHPULL(GPIO_OPAMP_FB) |        \
                                     PIN_OTYPE_PUSHPULL(GPIO_OPAMP_OUT) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CURRENT_SENSE) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_MOTOR_IN1) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_MOTOR_IN2) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_MOTOR_D1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED1) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED2) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_2M(GPIOA_ENCODER_ANALOG) | \
                                     PIN_OSPEED_2M(GPIOA_BRIDGE_TEMP) |    \
                                     PIN_OSPEED_2M(GPIOA_INT1) |           \
                                     PIN_OSPEED_2M(GPIOA_INT2) |           \
                                     PIN_OSPEED_2M(GPIOA_MOTOR_SF) |       \
                                     PIN_OSPEED_2M(GPIO_OPAMP_FB) |        \
                                     PIN_OSPEED_2M(GPIO_OPAMP_OUT) |       \
                                     PIN_OSPEED_2M(GPIOA_CURRENT_SENSE) |  \
                                     PIN_OSPEED_100M(GPIOA_MOTOR_IN1) |    \
                                     PIN_OSPEED_100M(GPIOA_MOTOR_IN2) |    \
                                     PIN_OSPEED_2M(GPIOA_MOTOR_D1) |       \
                                     PIN_OSPEED_2M(GPIOA_LED1) |           \
                                     PIN_OSPEED_2M(GPIOA_LED1) |           \
                                     PIN_OSPEED_100M(GPIOA_SWDIO) |        \
                                     PIN_OSPEED_100M(GPIOA_SWCLK) |        \
                                     PIN_OSPEED_2M(GPIOA_PIN15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_ENCODER_ANALOG) | \
                                     PIN_PUPDR_FLOATING(GPIOA_BRIDGE_TEMP) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_INT1) |             \
                                     PIN_PUPDR_PULLUP(GPIOA_INT2) |             \
                                     PIN_PUPDR_PULLUP(GPIOA_MOTOR_SF) |         \
                                     PIN_PUPDR_FLOATING(GPIO_OPAMP_FB) |        \
                                     PIN_PUPDR_FLOATING(GPIO_OPAMP_OUT) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_CURRENT_SENSE) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_MOTOR_IN1) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_MOTOR_IN2) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOA_MOTOR_D1) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_LED1) |           \
                                     PIN_PUPDR_FLOATING(GPIOA_LED2) |           \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |            \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |          \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN15))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_ENCODER_ANALOG) | \
                                     PIN_ODR_HIGH(GPIOA_BRIDGE_TEMP) |    \
                                     PIN_ODR_HIGH(GPIOA_INT1) |           \
                                     PIN_ODR_HIGH(GPIOA_INT2) |           \
                                     PIN_ODR_HIGH(GPIOA_MOTOR_SF) |       \
                                     PIN_ODR_HIGH(GPIO_OPAMP_FB) |        \
                                     PIN_ODR_HIGH(GPIO_OPAMP_OUT) |       \
                                     PIN_ODR_HIGH(GPIOA_CURRENT_SENSE) |  \
                                     PIN_ODR_HIGH(GPIOA_MOTOR_IN1) |      \
                                     PIN_ODR_HIGH(GPIOA_MOTOR_IN2) |      \
                                     PIN_ODR_LOW(GPIOA_MOTOR_D1) |        \
                                     PIN_ODR_HIGH(GPIOA_LED1) |           \
                                     PIN_ODR_HIGH(GPIOA_LED2) |           \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |          \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |          \
                                     PIN_ODR_HIGH(GPIOA_PIN15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_ENCODER_ANALOG, 0) | \
                                     PIN_AFIO_AF(GPIOA_BRIDGE_TEMP, 0) |    \
                                     PIN_AFIO_AF(GPIOA_INT1, 0) |           \
                                     PIN_AFIO_AF(GPIOA_INT2, 0) |           \
                                     PIN_AFIO_AF(GPIOA_MOTOR_SF, 0) |       \
                                     PIN_AFIO_AF(GPIO_OPAMP_FB, 0) |        \
                                     PIN_AFIO_AF(GPIO_OPAMP_OUT, 0) |       \
                                     PIN_AFIO_AF(GPIOA_CURRENT_SENSE, 0))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_MOTOR_IN1, 6) |      \
                                     PIN_AFIO_AF(GPIOA_MOTOR_IN2, 6) |      \
                                     PIN_AFIO_AF(GPIOA_MOTOR_D1, 0) |       \
                                     PIN_AFIO_AF(GPIOA_LED1, 0) |           \
                                     PIN_AFIO_AF(GPIOA_LED2, 0) |           \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) |          \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) |          \
                                     PIN_AFIO_AF(GPIOA_PIN15, 0))

#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOA_CURRENT_SENSE_FAULT) | \
                                     PIN_MODE_OUTPUT(GPIOB_MOTOR_EN) |           \
                                     PIN_MODE_OUTPUT(GPIOB_BRAKE) |              \
                                     PIN_MODE_INPUT(GPIOB_ENCODER_I) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_ENCODER_A) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_ENCODER_B) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_SCL) |             \
                                     PIN_MODE_ALTERNATE(GPIOB_SDA) |             \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN_RX) |          \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN_TX) |          \
                                     PIN_MODE_OUTPUT(GPIOB_MOTOR_D2) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN11) |               \
                                     PIN_MODE_ANALOG(GPIOB_ISENSE) |             \
                                     PIN_MODE_ANALOG(GPIOB_VSENSE) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN14) |               \
                                     PIN_MODE_OUTPUT(GPIOB_SLEW))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_CURRENT_SENSE_FAULT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MOTOR_EN) |            \
                                     PIN_OTYPE_PUSHPULL(GPIOB_BRAKE) |               \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ENCODER_I) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ENCODER_A) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ENCODER_B) |           \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_SCL) |                \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_SDA) |                \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN_RX) |              \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN_TX) |              \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MOTOR_D2) |            \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11) |               \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ISENSE) |              \
                                     PIN_OTYPE_PUSHPULL(GPIOB_VSENSE) |              \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN14) |               \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SLEW))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_2M(GPIOA_CURRENT_SENSE_FAULT) | \
                                     PIN_OSPEED_2M(GPIOB_MOTOR_EN) |            \
                                     PIN_OSPEED_2M(GPIOB_BRAKE) |               \
                                     PIN_OSPEED_2M(GPIOB_ENCODER_I) |           \
                                     PIN_OSPEED_2M(GPIOB_ENCODER_A) |           \
                                     PIN_OSPEED_2M(GPIOB_ENCODER_B) |           \
                                     PIN_OSPEED_100M(GPIOB_SCL) |               \
                                     PIN_OSPEED_100M(GPIOB_SDA) |               \
                                     PIN_OSPEED_100M(GPIOB_CAN_RX) |            \
                                     PIN_OSPEED_100M(GPIOB_CAN_TX) |            \
                                     PIN_OSPEED_2M(GPIOB_MOTOR_D2) |            \
                                     PIN_OSPEED_2M(GPIOB_PIN11) |               \
                                     PIN_OSPEED_2M(GPIOB_ISENSE) |              \
                                     PIN_OSPEED_2M(GPIOB_VSENSE) |              \
                                     PIN_OSPEED_2M(GPIOB_PIN14) |               \
                                     PIN_OSPEED_2M(GPIOB_SLEW))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_CURRENT_SENSE_FAULT) | \
                                     PIN_PUPDR_PULLDOWN(GPIOB_MOTOR_EN) |            \
                                     PIN_PUPDR_PULLDOWN(GPIOB_BRAKE) |               \
                                     PIN_PUPDR_PULLUP(GPIOB_ENCODER_I) |             \
                                     PIN_PUPDR_PULLUP(GPIOB_ENCODER_A) |             \
                                     PIN_PUPDR_PULLUP(GPIOB_ENCODER_B) |             \
                                     PIN_PUPDR_PULLUP(GPIOB_SCL) |                   \
                                     PIN_PUPDR_PULLUP(GPIOB_SDA) |                   \
                                     PIN_PUPDR_FLOATING(GPIOB_CAN_RX) |              \
                                     PIN_PUPDR_FLOATING(GPIOB_CAN_TX) |              \
                                     PIN_PUPDR_PULLUP(GPIOB_MOTOR_D2) |              \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN11) |                 \
                                     PIN_PUPDR_FLOATING(GPIOB_ISENSE) |              \
                                     PIN_PUPDR_PULLUP(GPIOB_VSENSE) |                \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN14) |                 \
                                     PIN_PUPDR_PULLUP(GPIOB_SLEW))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOA_CURRENT_SENSE_FAULT) | \
                                     PIN_ODR_LOW(GPIOB_MOTOR_EN) |             \
                                     PIN_ODR_LOW(GPIOB_BRAKE) |                \
                                     PIN_ODR_HIGH(GPIOB_ENCODER_I) |           \
                                     PIN_ODR_HIGH(GPIOB_ENCODER_A) |           \
                                     PIN_ODR_HIGH(GPIOB_ENCODER_B) |           \
                                     PIN_ODR_HIGH(GPIOB_SCL) |                 \
                                     PIN_ODR_HIGH(GPIOB_SDA) |                 \
                                     PIN_ODR_HIGH(GPIOB_CAN_RX) |              \
                                     PIN_ODR_HIGH(GPIOB_CAN_TX) |              \
                                     PIN_ODR_HIGH(GPIOB_MOTOR_D2) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN11) |               \
                                     PIN_ODR_HIGH(GPIOB_ISENSE) |              \
                                     PIN_ODR_HIGH(GPIOB_VSENSE) |              \
                                     PIN_ODR_HIGH(GPIOB_PIN14) |               \
                                     PIN_ODR_HIGH(GPIOB_SLEW))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOA_CURRENT_SENSE_FAULT, 0) | \
                                     PIN_AFIO_AF(GPIOB_MOTOR_EN, 0) |            \
                                     PIN_AFIO_AF(GPIOB_BRAKE, 0) |               \
                                     PIN_AFIO_AF(GPIOB_ENCODER_I, 0) |           \
                                     PIN_AFIO_AF(GPIOB_ENCODER_A, 2) |           \
                                     PIN_AFIO_AF(GPIOB_ENCODER_B, 2) |           \
                                     PIN_AFIO_AF(GPIOB_SCL, 4) |                 \
                                     PIN_AFIO_AF(GPIOB_SDA, 4))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_CAN_RX, 9) |   \
                                     PIN_AFIO_AF(GPIOB_CAN_TX, 9) |   \
                                     PIN_AFIO_AF(GPIOB_MOTOR_D2, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN11, 0) |    \
                                     PIN_AFIO_AF(GPIOB_ISENSE, 0) |   \
                                     PIN_AFIO_AF(GPIOB_ISENSE, 0) |   \
                                     PIN_AFIO_AF(GPIOB_PIN14, 0) |    \
                                     PIN_AFIO_AF(GPIOB_SLEW, 0))

#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_2M(GPIOC_PIN0) |            \
                                     PIN_OSPEED_2M(GPIOC_PIN1) |            \
                                     PIN_OSPEED_2M(GPIOC_PIN2) |            \
                                     PIN_OSPEED_2M(GPIOC_PIN3) |            \
                                     PIN_OSPEED_2M(GPIOC_PIN4) |            \
                                     PIN_OSPEED_2M(GPIOC_PIN5) |            \
                                     PIN_OSPEED_2M(GPIOC_PIN6) |            \
                                     PIN_OSPEED_2M(GPIOC_PIN7) |            \
                                     PIN_OSPEED_2M(GPIOC_PIN8) |            \
                                     PIN_OSPEED_2M(GPIOC_PIN9) |            \
                                     PIN_OSPEED_2M(GPIOC_PIN10) |           \
                                     PIN_OSPEED_2M(GPIOC_PIN11) |           \
                                     PIN_OSPEED_2M(GPIOC_PIN12) |           \
                                     PIN_OSPEED_2M(GPIOC_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOC_OSC32_IN) |      \
                                     PIN_OSPEED_100M(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLUP(GPIOC_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN12) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN7, 0))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOC_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOC_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOC_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0) |       \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0))

#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_2M(GPIOD_PIN0) |            \
                                     PIN_OSPEED_2M(GPIOD_PIN1) |            \
                                     PIN_OSPEED_2M(GPIOD_PIN2) |            \
                                     PIN_OSPEED_2M(GPIOD_PIN3) |            \
                                     PIN_OSPEED_2M(GPIOD_PIN4) |            \
                                     PIN_OSPEED_2M(GPIOD_PIN5) |            \
                                     PIN_OSPEED_2M(GPIOD_PIN6) |            \
                                     PIN_OSPEED_2M(GPIOD_PIN7) |            \
                                     PIN_OSPEED_2M(GPIOD_PIN8) |            \
                                     PIN_OSPEED_2M(GPIOD_PIN9) |            \
                                     PIN_OSPEED_2M(GPIOD_PIN10) |           \
                                     PIN_OSPEED_2M(GPIOD_PIN11) |           \
                                     PIN_OSPEED_2M(GPIOD_PIN12) |           \
                                     PIN_OSPEED_2M(GPIOD_PIN13) |           \
                                     PIN_OSPEED_2M(GPIOD_PIN14) |           \
                                     PIN_OSPEED_2M(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0))

#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_2M(GPIOE_PIN0) |            \
                                     PIN_OSPEED_2M(GPIOE_PIN1) |            \
                                     PIN_OSPEED_2M(GPIOE_PIN2) |            \
                                     PIN_OSPEED_2M(GPIOE_PIN3) |            \
                                     PIN_OSPEED_2M(GPIOE_PIN4) |            \
                                     PIN_OSPEED_2M(GPIOE_PIN5) |            \
                                     PIN_OSPEED_2M(GPIOE_PIN6) |            \
                                     PIN_OSPEED_2M(GPIOE_PIN7) |            \
                                     PIN_OSPEED_2M(GPIOE_PIN8) |            \
                                     PIN_OSPEED_2M(GPIOE_PIN9) |            \
                                     PIN_OSPEED_2M(GPIOE_PIN10) |           \
                                     PIN_OSPEED_2M(GPIOE_PIN11) |           \
                                     PIN_OSPEED_2M(GPIOE_PIN12) |           \
                                     PIN_OSPEED_2M(GPIOE_PIN13) |           \
                                     PIN_OSPEED_2M(GPIOE_PIN14) |           \
                                     PIN_OSPEED_2M(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(GPIOE_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN7, 0))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0))

#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_2M(GPIOF_PIN0) |            \
                                     PIN_OSPEED_2M(GPIOF_PIN1) |            \
                                     PIN_OSPEED_2M(GPIOF_PIN2) |            \
                                     PIN_OSPEED_2M(GPIOF_PIN3) |            \
                                     PIN_OSPEED_2M(GPIOF_PIN4) |            \
                                     PIN_OSPEED_2M(GPIOF_PIN5) |            \
                                     PIN_OSPEED_2M(GPIOF_PIN6) |            \
                                     PIN_OSPEED_2M(GPIOF_PIN7) |            \
                                     PIN_OSPEED_2M(GPIOF_PIN8) |            \
                                     PIN_OSPEED_2M(GPIOF_PIN9) |            \
                                     PIN_OSPEED_2M(GPIOF_PIN10) |           \
                                     PIN_OSPEED_2M(GPIOF_PIN11) |           \
                                     PIN_OSPEED_2M(GPIOF_PIN12) |           \
                                     PIN_OSPEED_2M(GPIOF_PIN13) |           \
                                     PIN_OSPEED_2M(GPIOF_PIN14) |           \
                                     PIN_OSPEED_2M(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_PULLUP(GPIOF_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0))


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
