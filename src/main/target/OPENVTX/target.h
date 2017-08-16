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

#define TARGET_BOARD_IDENTIFIER "OVTX"

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT

#define LED0_PIN                PA15

#define USE_EXTI


#define GYRO
#define USE_FAKE_GYRO

#define ACC
#define USE_FAKE_ACC

#define REMAP_TIM16_DMA
#define REMAP_TIM17_DMA

//#define USB_DETECT_PIN          NONE

#define USE_VCP
#define USE_UART1
#define USE_UART3
#define SERIAL_PORT_COUNT       3

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define BUS_SWITCH_PIN          PB3 // connects and disconnects UART1 from external devices

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_SPI
#define USE_SPI_DEVICE_1 // Flash Chip
#define USE_SPI_DEVICE_2 // MAX7456

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

// IO - assuming 303 in 64pin package, TODO
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (BIT(2))
#define TARGET_IO_PORTF (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 4
#define USED_TIMERS  (TIM_N(3) | (16) | TIM_N(17))
