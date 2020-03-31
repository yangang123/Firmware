/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * holybro durandal-v1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>


#  define BOARD_HAS_LTC44XX_VALIDS      0 // No LTC or N Bricks
#  define BOARD_HAS_USB_VALID           0 // LTC Has No USB valid
#  define BOARD_HAS_NBAT_V              0 // Only one Vbat to ADC
#  define BOARD_HAS_NBAT_I              0 // No Ibat ADC


#define BOARD_HAS_CONTROL_STATUS_LEDS      0
// #define BOARD_OVERLOAD_LED     LED_RED
// #define BOARD_ARMED_STATE_LED  LED_BLUE

/*
*SENSORS are on SPI1, 5, 6
 * MEMORY is on bus SPI2
 * MS5611 is on bus SPI4
 */

#define PX4_SPI_BUS_SENSORS   4
#define PX4_SPI_BUS_SENSORS3   PX4_SPI_BUS_SENSORS // for BMI088
#define PX4_SPI_BUS_BARO      2
#define PX4_SPI_BUS_EXTERNAL1 5
#define RC_SERIAL_PORT		"/dev/ttyS0"


#define GPIO_nLED_BLUE       /* PC7 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN2)
#define GPIO_nLED_GREEN      /* PC6 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN3)

/*  Define the Chip Selects, Data Ready and Control signals per SPI bus */

/* SPI 1 CS */
/*qspi flash */
#define GPIO_SPI1_CS               /*PG6 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN6)


/* SPI1 off */

/* SPI 2 CS */
/* baro cs */
#define GPIO_SPI2_CS_BARO               /*PI0 */   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN0)

/* SPI 4 CS */
/**sensor bus*/
#define GPIO_SPI4_CS1_ICM_42605     /*PI6 */   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN6)
#define GPIO_SPI4_CS2_BMI088_ACC    /*PI4 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN4)
#define GPIO_SPI4_CS3_BMI088_GYRO   /*PE1 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN1)
#define GPIO_SPI4_CS4_BMI055_ACC    /*PI9 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN9)
#define GPIO_SPI4_CS5_BMI055_GYRO   /*PI5 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN5)

/* SPI 5 CS */
/*reserved spi*/
#define GPIO_SPI5_CS		   /*PK1 */   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTK|GPIO_PIN1)






/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */





/* v BEGIN Legacy SPI defines TODO: fix this with enumeration */
#define PX4_SPI_BUS_RAMTRON  PX4_SPI_BUS_MEMORY
/* ^ END Legacy SPI defines TODO: fix this with enumeration */

#define PX4_SPIDEV_ICM_42605        PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,0)
#define PX4_SPIDEV_BMI088_ACC       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,1)
#define PX4_SPIDEV_BMI088_GYR       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,2)
#define PX4_SPIDEV_BMI055_ACC       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,3)
#define PX4_SPIDEV_BMI055_GYR       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,4)
#define PX4_SENSOR_BUS_CS_GPIO      {GPIO_SPI4_CS1_ICM_42605, GPIO_SPI4_CS2_BMI088_ACC, GPIO_SPI4_CS3_BMI088_GYRO, GPIO_SPI4_CS4_BMI055_ACC,GPIO_SPI4_CS5_BMI055_GYRO}


#define PX4_SPIDEV_BARO             PX4_MK_SPI_SEL(PX4_SPI_BUS_BARO,0)
#define PX4_BARO_BUS_CS_GPIO        {GPIO_SPI2_CS_BARO}


#define PX4_SPIDEV_EXTERNAL2_1      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL2,0)
#define PX4_EXTERNAL2_BUS_CS_GPIO   {GPIO_SPI5_CS}


/* I2C busses */

#define PX4_I2C_BUS_EXPANSION       3
#define PX4_I2C_BUS_ONBOARD         4

#define PX4_I2C_OBDEV_BMP280        0x76

#define BOARD_NUMBER_I2C_BUSES      2
#define BOARD_I2C_BUS_CLOCK_INIT    {100000, 100000}

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */




/* HEATER
* PWM in future
*/
#define GPIO_HEATER_OUTPUT   /* PA7  T14CH1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN2)

/* ESC FIRE RGB
* 4-way pwm for esc PA6 PA7 PB0 PB1
* 4-way pwm for rgbd
* 4-way pwm for fire
*/


#define DIRECT_PWM_OUTPUT_CHANNELS  8
#define DIRECT_INPUT_TIMER_CHANNELS 8



#define BOARD_NUMBER_BRICKS             0




/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

/* RC Serial port */

/* Input Capture Channels. */

#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_LIB_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_LIB_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))

/* Board never powers off the Servo rail */



#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 0

/* The list of GPIO that will be initialized */


#define PX4_GPIO_INIT_LIST {\
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_HEATER_OUTPUT\
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Holybro Durandal V1
 *   board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

void board_spi_reset(int ms);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);


/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
