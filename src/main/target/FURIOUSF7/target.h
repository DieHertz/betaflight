/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "FFF7"

#define USBD_PRODUCT_STRING  "FuriousFPV F7"

#define USE_DUAL_GYRO
#define ENABLE_DSHOT_DMAR       true

#define LED0_PIN                PB8
#define LED1_PIN                PB9

#define USE_BEEPER
#define BEEPER_PIN              PB5
#define BEEPER_INVERTED

// Buses
#define USE_SPI

// Gyros SPI bus
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN			PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

// OSD SPI bus
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN			PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_I2C

// Baro & Compass I2C bus
#define USE_I2C_DEVICE_3
#define I2C_DEVICE              (I2CDEV_3)
#define I2C1_SCL                PA8
#define I2C1_SDA                PC9

// *************** Gyro & ACC **********************
#define USE_EXTI
#define GYRO_1_EXTI_PIN         PC0
#define GYRO_2_EXTI_PIN         PC1
#define MPU_INT_EXTI

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500

#define USE_ACC
#define USE_ACC_SPI_MPU6500

#define ACC_MPU6500_1_ALIGN		CW0_DEG
#define GYRO_MPU6500_1_ALIGN	CW0_DEG

#define ACC_MPU6500_2_ALIGN     CW270_DEG
#define GYRO_MPU6500_2_ALIGN    CW270_DEG

#define GYRO_1_ALIGN            GYRO_MPU6500_1_ALIGN
#define GYRO_2_ALIGN            GYRO_MPU6500_2_ALIGN
#define ACC_1_ALIGN             ACC_MPU6500_1_ALIGN
#define ACC_2_ALIGN             ACC_MPU6500_2_ALIGN

#define GYRO_1_CS_PIN			SPI1_NSS_PIN
#define GYRO_1_SPI_INSTANCE		SPI1
#define GYRO_2_CS_PIN			PB0
#define GYRO_2_SPI_INSTANCE		SPI1

// *************** Baro **************************
#define USE_BARO
#define USE_BARO_BMP280

#define BARO_I2C_INSTANCE       (I2CDEV_3)
#define BARO_I2C_ADDRESS		0x77

//*********** Magnetometer / Compass *************
#define USE_MAG
#define USE_MAG_AK8963
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

#define MAG_I2C_INSTANCE		(I2CDEV_3)
#define MAG_I2C_ADDRESS			0x0e
#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH
#define MAG_INT_EXTI            PC4

// *************** OSD *****************************
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN

// *************** UART *****************************
#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define USE_UART3
#define UART3_TX_PIN            PC10
#define UART3_RX_PIN            PC11

#define USE_UART4
#define UART4_TX_PIN            PA0
#define UART4_RX_PIN            PA1

#define USE_UART5
#define UART5_TX_PIN            PC12
#define UART5_RX_PIN            PD2

#define USE_UART6
#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7

#define SERIAL_PORT_COUNT       7

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_UART5

// *************** ADC *****************************
#define USE_ADC
//#define ADC1_DMA_STREAM         DMA2_Stream0
#define VBAT_ADC_PIN            PC3
#define CURRENT_METER_ADC_PIN   PC2

#define DEFAULT_FEATURES        (FEATURE_OSD)
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_ESCSERIAL
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS             (TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8))
