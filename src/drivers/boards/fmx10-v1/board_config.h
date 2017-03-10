/****************************************************************************
 *
 *   Copyright (c) 2016 FMX10 Development Team. All rights reserved.
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
 * 3. Neither the name FMX10 nor the names of its contributors may be
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
 * FMX10NUCLEOF767ZI-v1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <fmx10_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

/* these headers are not C++ safe */
#include <chip.h>
#include <stm32_gpio.h>
#include <arch/board/board.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/
//{GPIO_RSSI_IN,           0,                       0}, - pio Analog used as PWM
//{0,                      GPIO_LED_SAFETY,         0},	pio replacement
//{GPIO_SAFETY_SWITCH_IN,  0,                       0},   pio replacement
//{0,                      GPIO_PERIPH_3V3_EN,      0},	Owned by the 8266 driver
//{0,                      GPIO_SBUS_INV,           0},	https://github.com/FMX10/Firmware/blob/master/src/modules/fmx10iofirmware/sbus.c
//{GPIO_8266_GPIO0,        0,                       0},   Owned by the 8266 driver
//{0,                      GPIO_SPEKTRUM_PWR_EN,     0},	Owned Spektum driver input to auto pilot
//{0,                      GPIO_8266_PD,            0},	Owned by the 8266 driver
//{0,                      GPIO_8266_RST,           0},	Owned by the 8266 driver

/* FMX10FMU GPIOs ***********************************************************************************/
/* LEDs */
/*                                Port[CON-PIN] FMUv5 Delta */
#define GPIO_LED1              /* PB14[CN12-28] DRDY2   */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN14)
#define GPIO_LED2              /* PB0[CN11-34]  RSSI    */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_LED3              /* PB7[CN11-21]  GPS1_TX */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN7)

#define GPIO_LED_RED 	GPIO_LED1
#define GPIO_LED_GREEN 	GPIO_LED2
#define GPIO_LED_BLUE   GPIO_LED3

/*  Define the Chip Selects */

#define GPIO_SPI_CS_MPU9250     /* PF2[CN11-52]  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN10)
#define GPIO_SPI_CS_HMC5983     /* PF3[CN12-58]  */	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN4)
#define GPIO_SPI_CS_LIS3MDL     /* PF4[CN12-38]  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN13)

#define GPIO_SPI_CS_FRAM        /* PF5[CN12-36] */(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)

#define GPIO_SPI_CS_MS5611      /* PF10[CN12-42] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN13)

#define GPIO_SPI_CS_ICM_20608_G /* PF13[CN12-57] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN8)

#define GPIO_SPI_CS_MPU6500	/*PG10*/    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN10)
//#define GPIO_SPI_CS_MPU6500	/*PG10*/    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN6)
#define GPIO_SPI_CS_ICM20602	/*PD4*/	    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN4)
#define GPIO_SPI_CS_MS5607	/*PG13*/    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN13)
#define GPIO_SPI_CS_SPL06001	/*PE3*/    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)


/*  Define the Ready interrupts */

#define GPIO_DRDY_MPU9250       /* PB4[CN12-27]  */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN4)
#define GPIO_DRDY_HMC5983       /* PB15[CN12-26] */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN15)
#define GPIO_DRDY_ICM_20608_G   /* PC5[CN12-6]   */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN5)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))

#define GPIO_SPI_CS_OFF_MPU9250		_PIN_OFF(GPIO_SPI_CS_MPU9250)
#define GPIO_SPI_CS_OFF_HMC5983		_PIN_OFF(GPIO_SPI_CS_HMC5983)
#define GPIO_SPI_CS_OFF_LIS3MDL		_PIN_OFF(GPIO_SPI_CS_LIS3MDL)
#define GPIO_SPI_CS_OFF_MS5611		_PIN_OFF(GPIO_SPI_CS_MS5611)
#define GPIO_SPI_CS_OFF_ICM_20608_G _PIN_OFF(GPIO_SPI_CS_ICM_20608_G)

#define GPIO_SPI_CS_OFF_MPU6500		_PIN_OFF(GPIO_SPI_CS_MPU6500)
#define GPIO_SPI_CS_OFF_ICM20602	_PIN_OFF(GPIO_SPI_CS_ICM20602)
#define GPIO_SPI_CS_OFF_MS5607		_PIN_OFF(GPIO_SPI_CS_MS5607)
#define GPIO_SPI_CS_OFF_SPL06001	_PIN_OFF(GPIO_SPI_CS_SPL06001)


#define GPIO_DRDY_OFF_MPU9250		_PIN_OFF(GPIO_DRDY_MPU9250)
#define GPIO_DRDY_OFF_HMC5983		_PIN_OFF(GPIO_DRDY_HMC5983)
#define GPIO_DRDY_OFF_ICM_20608_G	_PIN_OFF(GPIO_DRDY_ICM_20608_G)


/* SPI1 off */
#define GPIO_SPI1_SCK_OFF	_PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF	_PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF	_PIN_OFF(GPIO_SPI1_MOSI)

/* SPI2 off */
#define GPIO_SPI2_SCK_OFF	_PIN_OFF(GPIO_SPI2_SCK)
#define GPIO_SPI2_MISO_OFF	_PIN_OFF(GPIO_SPI2_MISO)
#define GPIO_SPI2_MOSI_OFF	_PIN_OFF(GPIO_SPI2_MOSI)

/* SPI3 off */
#define GPIO_SPI3_SCK_OFF	_PIN_OFF(GPIO_SPI3_SCK)
#define GPIO_SPI3_MISO_OFF	_PIN_OFF(GPIO_SPI3_MISO)
#define GPIO_SPI3_MOSI_OFF	_PIN_OFF(GPIO_SPI3_MOSI)

/* SPI4 off */
#define GPIO_SPI4_SCK_OFF	_PIN_OFF(GPIO_SPI4_SCK)
#define GPIO_SPI4_MISO_OFF	_PIN_OFF(GPIO_SPI4_MISO)
#define GPIO_SPI4_MOSI_OFF	_PIN_OFF(GPIO_SPI4_MOSI)

/* SPI5 off */
#define GPIO_SPI5_SCK_OFF	_PIN_OFF(GPIO_SPI5_SCK)
#define GPIO_SPI5_MISO_OFF	_PIN_OFF(GPIO_SPI5_MISO)
#define GPIO_SPI5_MOSI_OFF	_PIN_OFF(GPIO_SPI5_MOSI)

/* SPI6 off */
#define GPIO_SPI6_SCK_OFF	_PIN_OFF(GPIO_SPI6_SCK)
#define GPIO_SPI6_MISO_OFF	_PIN_OFF(GPIO_SPI6_MISO)
#define GPIO_SPI6_MOSI_OFF	_PIN_OFF(GPIO_SPI6_MOSI)

/* SENSORS are on SPI1
 * FRAM is on bus SPI4
 * MS5611 is on bus SPI3
 */
//=================================================================================
#define FMX10_SPI_BUS_MPU		1
#define FMX10_SPI_BUS_ICM		2
#define FMX10_SPI_BUS_MS		3
#define FMX10_SPI_BUS_SPL		4

#define FMX10_SPIDEV_MPU		FMX10_MK_SPI_SEL(FMX10_SPI_BUS_MPU,0)
#define FMX10_MPU_BUS_CS_GPIO		{GPIO_SPI_CS_MPU6500}
#define FMX10_MPU_BUS_FIRST_CS		FMX10_SPIDEV_MPU
#define FMX10_MPU_BUS_LAST_CS		FMX10_SPIDEV_MPU

#define FMX10_SPIDEV_ICM		FMX10_MK_SPI_SEL(FMX10_SPI_BUS_ICM,0)
#define FMX10_ICM_BUS_CS_GPIO		{GPIO_SPI_CS_ICM20602}
#define FMX10_ICM_BUS_FIRST_CS		FMX10_SPIDEV_ICM
#define FMX10_ICM_BUS_LAST_CS		FMX10_SPIDEV_ICM

#define FMX10_SPIDEV_MS			FMX10_MK_SPI_SEL(FMX10_SPI_BUS_MS,0)
#define FMX10_MS_BUS_CS_GPIO		{GPIO_SPI_CS_MS5611}
#define FMX10_MS_BUS_FIRST_CS		FMX10_SPIDEV_MS
#define FMX10_MS_BUS_LAST_CS		FMX10_SPIDEV_MS

#define FMX10_SPIDEV_SPL		FMX10_MK_SPI_SEL(FMX10_SPI_BUS_SPL,0)
#define FMX10_SPL_BUS_CS_GPIO		{GPIO_SPI_CS_SPL06001}
#define FMX10_SPL_BUS_FIRST_CS		FMX10_SPIDEV_SPL
#define FMX10_SPL_BUS_LAST_CS		FMX10_SPIDEV_SPL

//=================================================================================
#define FMX10_SPI_BUS_SENSORS	1
#define FMX10_SPI_BUS_RAMTRON	4
#define FMX10_SPI_BUS_BARO    3
#define FMX10_SPI_BUS_ICM0     6

#define FMX10_SPIDEV_GYRO			FMX10_MK_SPI_SEL(FMX10_SPI_BUS_SENSORS,0)
#define FMX10_SPIDEV_ACCEL_MAG			FMX10_MK_SPI_SEL(FMX10_SPI_BUS_SENSORS,1)
#define FMX10_SPIDEV_MPU0			FMX10_MK_SPI_SEL(FMX10_SPI_BUS_SENSORS,2)
//#define FMX10_SPIDEV_HMC			FMX10_MK_SPI_SEL(FMX10_SPI_BUS_SENSORS,3)
#define FMX10_SPIDEV_LIS           FMX10_MK_SPI_SEL(FMX10_SPI_BUS_SENSORS,4)
#define FMX10_SPIDEV_BMI           FMX10_MK_SPI_SEL(FMX10_SPI_BUS_SENSORS,5)
#define FMX10_SPIDEV_BMA           FMX10_MK_SPI_SEL(FMX10_SPI_BUS_SENSORS,6)

#define FMX10_SENSOR_BUS_CS_GPIO   {0, 0,GPIO_SPI_CS_MPU6500, GPIO_SPI_CS_ICM20602, GPIO_SPI_CS_MS5607, GPIO_SPI_CS_SPL06001, 0, 0}
#define FMX10_SENSORS_BUS_FIRST_CS FMX10_SPIDEV_GYRO
#define FMX10_SENSORS_BUS_LAST_CS  FMX10_SPIDEV_BMA

#define FMX10_SPIDEV_FRAM          FMX10_MK_SPI_SEL(FMX10_SPI_BUS_RAMTRON,0)
#define FMX10_RAMTRON_BUS_CS_GPIO  {GPIO_SPI_CS_FRAM}
#define FMX10_RAMTRON_BUS_FIRST_CS FMX10_SPIDEV_FRAM
#define FMX10_RAMTRON_BUS_LAST_CS  FMX10_SPIDEV_FRAM

#define FMX10_SPIDEV_BARO          FMX10_MK_SPI_SEL(FMX10_SPI_BUS_BARO,0)
#define FMX10_BARO_BUS_CS_GPIO     {GPIO_SPI_CS_MS5611}
#define FMX10_BARO_BUS_FIRST_CS    FMX10_SPIDEV_BARO
#define FMX10_BARO_BUS_LAST_CS     FMX10_SPIDEV_BARO

#define FMX10_SPIDEV_ICM0           FMX10_MK_SPI_SEL(FMX10_SPI_BUS_ICM0,0)
#define FMX10_ICM_BUS_CS_GPIO0      {GPIO_SPI_CS_ICM_20608_G}
#define FMX10_ICM_BUS_FIRST_CS0     FMX10_SPIDEV_ICM
#define FMX10_ICM_BUS_LAST_CS0      FMX10_SPIDEV_ICM

/* I2C busses */
#define FMX10_I2C_BUS_EXPANSION	4
#define FMX10_I2C_BUS_LED			FMX10_I2C_BUS_EXPANSION
#define FMX10_I2C_BUS_ONBOARD	1
#define FMX10_I2C_BUS_HMC5883	1
#define FMX10_I2C_BUS_IST8310	2
#define FMX10_I2C_BUS_BATTERY	3

/* Devices on the external bus.
 *
 * Note that these are unshifted addresses.
 */
#define FMX10_I2C_OBDEV_LED	    0x55
#define FMX10_I2C_OBDEV_HMC5883	0x1e
#define FMX10_I2C_OBDEV_LIS3MDL	0x1e

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | \
	(1 << 8) | \
	(1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL		0
#define ADC_BATTERY_CURRENT_CHANNEL		1
#define ADC_5V_RAIL_SENSE				10
#define ADC_RC_RSSI_CHANNEL				14

/* User GPIOs
 *
 * GPIO0-5 are the PWM servo outputs.
 */
#define GPIO_GPIO0_INPUT        /* PE14[CN12-51] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPIO1_INPUT        /* PA10[CN12-33] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN10)
#define GPIO_GPIO2_INPUT        /* PE11[CN12-56] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPIO3_INPUT        /* PE9[CN12-52]  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO4_INPUT        /* PD13[CN12-41] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO5_INPUT        /* PD14[CN12-46] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN14)

#define GPIO_GPIO0_OUTPUT       /* PE14[CN12-51] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPIO1_OUTPUT       /* PE13[CN12-55] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)
#define GPIO_GPIO2_OUTPUT       /* PE11[CN12-56] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPIO3_OUTPUT       /* PE9[CN12-52]  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO4_OUTPUT       /* PD13[CN12-41] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO5_OUTPUT       /* PD14[CN12-46] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)

/* Power supply control and monitoring GPIOs */
#define GPIO_VDD_BRICK_VALID    /* PB10[CN12-25] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN10)
#define GPIO_VDD_3V3_SENSORS_EN /* PE3[CN11-47] */	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)

/* Tone alarm output */
#define TONE_ALARM_TIMER		2	/* timer 2 */
#define TONE_ALARM_CHANNEL      /* PA5[CN12-11] TIM2_CH1 */ 1	/* channel 1 */
#define GPIO_TONE_ALARM_IDLE    /* PA5[CN12-11] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
#define GPIO_TONE_ALARM         /* PA5[CN12-11] */ (GPIO_ALT|GPIO_AF1|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN15)

/* PWM
 *
 * Six PWM outputs are configured.
 *
 * Pins:
 *
 * CH1 : PE14 : TIM1_CH4
 * CH2 : PE13 : TIM1_CH3
 * CH3 : PE11 : TIM1_CH2
 * CH4 : PE9  : TIM1_CH1
 * CH5 : PD13 : TIM4_CH2
 * CH6 : PD14 : TIM4_CH3
 */
#define GPIO_TIM1_CH1OUT        /* PE14[CN12-51] */ (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN9)
#define GPIO_TIM1_CH2OUT        /* PE13[CN12-55] */ (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN11)
#define GPIO_TIM1_CH3OUT        /* PE11[CN12-56] */ (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN13)
#define GPIO_TIM1_CH4OUT        /* PE9[CN12-52]  */ (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN14)
#define GPIO_TIM4_CH2OUT        /* PD13[CN12-41] */ (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN13)
#define GPIO_TIM4_CH3OUT        /* PD14[CN12-46] */ (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN14)
#define DIRECT_PWM_OUTPUT_CHANNELS	6

#define GPIO_TIM1_CH1IN         /* PE14[CN12-51] */ GPIO_TIM1_CH1IN_2
#define GPIO_TIM1_CH2IN         /* PE13[CN12-55] */ GPIO_TIM1_CH2IN_2
#define GPIO_TIM1_CH3IN         /* PE11[CN12-56] */ GPIO_TIM1_CH3IN_2
#define GPIO_TIM1_CH4IN         /* PE9[CN12-52]  */ GPIO_TIM1_CH4IN_2
#define GPIO_TIM4_CH2IN         /* PD13[CN12-41] */ GPIO_TIM4_CH2IN_2
#define GPIO_TIM4_CH3IN         /* PD14[CN12-46] */ GPIO_TIM4_CH3IN_2
#define DIRECT_INPUT_TIMER_CHANNELS  6

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9[CN12-21] */ (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER		    8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL   3	/* use capture/compare channel 3 */

#define HRT_PPM_CHANNEL         /* PA7[CN12-15] */  1	/* use capture/compare channel 1 */
#define GPIO_PPM_IN             /* PB0[CN11-34] */ GPIO_TIM3_CH3IN_1

#define RC_SERIAL_PORT		"/dev/ttyS4"

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER			4
#define PWMIN_TIMER_CHANNEL     /* PD13[CN12-41] */ 2
#define GPIO_PWM_IN             /* PD13[CN12-41] */ GPIO_TIM4_CH2IN_2

#define GPIO_RSSI_IN            /* PC4[CN12-34]  */	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN4)
#define GPIO_LED_SAFETY         /* PE12[CN12-49] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12)
#define GPIO_BTN_SAFETY         /* PE10[CN12-47] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN10)
#define GPIO_PERIPH_3V3_EN      /* PG4[CN12-69]  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4)

#define GPIO_SBUS_INV		    /* PD10[CN12-65] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)
#define INVERT_RC_INPUT(_s)		fmx10_arch_gpiowrite(GPIO_SBUS_INV, _s);

#define GPIO_8266_GPIO0         /* PD15[CN12-48] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN15)
#define GPIO_SPEKTRUM_PWR_EN    /* PE4[CN11-48]  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)
#define GPIO_8266_PD            /* PE7[CN12-44]  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN7)
#define GPIO_8266_RST           /* PG10[CN11-66] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN10)

#define GPIO_VDD_5V_PERIPH_OC   /* PE15[CN12-53] */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN15)

/* Power switch controls ******************************************************/

#define POWER_SPEKTRUM(_s)      fmx10_arch_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (1-_s))
#define SPEKTRUM_RX_AS_UART()   fmx10_arch_configgpio(GPIO_USART1_RX)

// FMUv4 has a separate GPIO for serial RC output
#define GPIO_RC_OUT			    /* PE5[CN11-50] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)
#define SPEKTRUM_RX_AS_GPIO()   fmx10_arch_configgpio(GPIO_RC_OUT)
#define SPEKTRUM_RX_HIGH(_s)    fmx10_arch_gpiowrite(GPIO_RC_OUT, (_s))

#define SDIO_SLOTNO             0  /* Only one slot */
#define SDIO_MINOR              0

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

//#define	BOARD_NAME "FMX10NUCLEOF767ZI_V1"
#define	BOARD_NAME "FMX10_V1"

/* By Providing BOARD_ADC_USB_CONNECTED (using the fmx10_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (fmx10_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_BRICK_VALID   (fmx10_arch_gpioread(GPIO_VDD_BRICK_VALID))
#define BOARD_ADC_SERVO_VALID   (1)
#define BOARD_ADC_PERIPH_5V_OC  (fmx10_arch_gpioread(GPIO_VDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (0)

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_FMU_GPIO_TAB { \
		{GPIO_GPIO0_INPUT,       GPIO_GPIO0_OUTPUT,       0}, \
		{GPIO_GPIO1_INPUT,       GPIO_GPIO1_OUTPUT,       0}, \
		{GPIO_GPIO2_INPUT,       GPIO_GPIO2_OUTPUT,       0}, \
		{GPIO_GPIO3_INPUT,       GPIO_GPIO3_OUTPUT,       0}, \
		{GPIO_GPIO4_INPUT,       GPIO_GPIO4_OUTPUT,       0}, \
		{GPIO_GPIO5_INPUT,       GPIO_GPIO5_OUTPUT,       0}, \
		{0,                      GPIO_VDD_3V3_SENSORS_EN, 0}, \
		{GPIO_VDD_BRICK_VALID,   0,                       0}, }

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

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
 *   Called to configure SPI chip select GPIO pins for the FMX10FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

/************************************************************************************
 * Name: stm32_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI Buses.
 *
 ************************************************************************************/

extern int stm32_spi_bus_initialize(void);

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

#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
