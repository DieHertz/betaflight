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

#include <platform.h>

#ifdef CAMERA_CONTROL

#include "camera_control.h"
#include "config/parameter_group_ids.h"
#include "io.h"
#include "pwm_output.h"
#include "system.h"
#include "math.h"
#include "nvic.h"

#if defined(STM32F40_41xxx)
#define CAMERA_CONTROL_TIMER_MHZ   84
#elif defined(STM32F7)
#define CAMERA_CONTROL_TIMER_MHZ   108
#else
#define CAMERA_CONTROL_TIMER_MHZ   72
#endif

#define CAMERA_CONTROL_PWM_RESOLUTION   128
#define CAMERA_CONTROL_SOFT_PWM_RESOLUTION 448

#ifdef CURRENT_TARGET_CPU_VOLTAGE
#define ADC_VOLTAGE CURRENT_TARGET_CPU_VOLTAGE
#else
#define ADC_VOLTAGE 3.3f
#endif

#if !defined(STM32F411xE) && !defined(STM32F7) && !defined(SITL)
#include "build/atomic.h"
#define CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
#endif

#if !defined(SITL)
#define CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
#include "timer.h"
#endif


PG_REGISTER_WITH_RESET_TEMPLATE(cameraControlConfig_t, cameraControlConfig, PG_CAMERA_CONTROL_CONFIG, 0);

PG_RESET_TEMPLATE(cameraControlConfig_t, cameraControlConfig,
    .mode = CAMERA_CONTROL_MODE_HARDWARE_PWM,
    .refVoltage = 330,
    .keyDelayMs = 150,
    .ioTag = IO_TAG_NONE
);

static pwmOutputPort_t cameraControlPwm;
// @todo protect against overflow
static uint32_t endTimeMillis;

#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
void TIM6_DAC_IRQHandler()
{
    IOHi(cameraControlPwm.io);

    TIM6->SR = 0;
}

void TIM7_IRQHandler()
{
    IOLo(cameraControlPwm.io);

    TIM7->SR = 0;
}
#endif

void cameraControlInit()
{
    if (cameraControlConfig()->ioTag == IO_TAG_NONE)
        return;

    cameraControlPwm.io = IOGetByTag(cameraControlConfig()->ioTag);
    IOInit(cameraControlPwm.io, OWNER_CAMERA_CONTROL, 0);

    if (CAMERA_CONTROL_MODE_HARDWARE_PWM == cameraControlConfig()->mode) {
#ifdef CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
        const timerHardware_t *timerHardware = timerGetByTag(cameraControlConfig()->ioTag, TIM_USE_ANY);

        if (!timerHardware) {
            return;
        }

        #ifdef USE_HAL_DRIVER
        IOConfigGPIOAF(cameraControlPwm.io, IOCFG_AF_PP, timerHardware->alternateFunction);
        #else
        IOConfigGPIO(cameraControlPwm.io, IOCFG_AF_PP);
        #endif

        pwmOutConfig(&cameraControlPwm, timerHardware, CAMERA_CONTROL_TIMER_MHZ, CAMERA_CONTROL_PWM_RESOLUTION, 0, 0);

        *cameraControlPwm.ccr = cameraControlPwm.period;
        cameraControlPwm.enabled = true;
#endif
    } else if (CAMERA_CONTROL_MODE_SOFTWARE_PWM == cameraControlConfig()->mode) {
#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
        IOConfigGPIO(cameraControlPwm.io, IOCFG_OUT_PP);
        IOHi(cameraControlPwm.io);

        cameraControlPwm.period = CAMERA_CONTROL_SOFT_PWM_RESOLUTION;
        cameraControlPwm.enabled = true;

        NVIC_InitTypeDef nvicTIM6 = {
            TIM6_DAC_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER), ENABLE
        };
        NVIC_Init(&nvicTIM6);
        NVIC_InitTypeDef nvicTIM7 = {
            TIM7_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER), ENABLE
        };
        NVIC_Init(&nvicTIM7);

        RCC->APB1ENR |= RCC_APB1Periph_TIM6 | RCC_APB1Periph_TIM7;
        TIM6->PSC = 0;
        TIM7->PSC = 0;
#endif
    } else if (CAMERA_CONTROL_MODE_DAC == cameraControlConfig()->mode) {
        // @todo not yet implemented
    }
}

void cameraControlProcess(uint32_t currentTimeUs)
{
    if (endTimeMillis && currentTimeUs >= 1000 * endTimeMillis) {
        if (CAMERA_CONTROL_MODE_HARDWARE_PWM == cameraControlConfig()->mode) {
            *cameraControlPwm.ccr = cameraControlPwm.period;
        } else if (CAMERA_CONTROL_MODE_SOFTWARE_PWM == cameraControlConfig()->mode) {

        }

        endTimeMillis = 0;
    }
}

static const int cameraPullUpResistance = 47000;
static const int buttonResistanceValues[] = { 45000, 27000, 15000, 6810, 0 };

static float calculateKeyPressVoltage(const cameraControlKey_e key)
{
    const int buttonResistance = buttonResistanceValues[key];
    return 1.0e-2f * cameraControlConfig()->refVoltage * buttonResistance / (cameraPullUpResistance + buttonResistance);
}

static float calculatePWMDutyCycle(const cameraControlKey_e key)
{
    const float voltage = calculateKeyPressVoltage(key);

    return voltage / ADC_VOLTAGE;
}

void cameraControlKeyPress(cameraControlKey_e key)
{
    if (key >= CAMERA_CONTROL_KEYS_COUNT)
        return;

    const float dutyCycle = calculatePWMDutyCycle(key);

    if (CAMERA_CONTROL_MODE_HARDWARE_PWM == cameraControlConfig()->mode) {
        *cameraControlPwm.ccr = lrintf(dutyCycle * cameraControlPwm.period);
        endTimeMillis = millis() + cameraControlConfig()->keyDelayMs;
    } else if (CAMERA_CONTROL_MODE_SOFTWARE_PWM == cameraControlConfig()->mode) {
#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
        const uint32_t hiTime = lrintf(dutyCycle * cameraControlPwm.period);

        if (0 == hiTime) {
            IOLo(cameraControlPwm.io);
            delay(cameraControlConfig()->keyDelayMs);
            IOHi(cameraControlPwm.io);
        } else {
            TIM6->CNT = hiTime;
            TIM6->ARR = cameraControlPwm.period;

            TIM7->CNT = 0;
            TIM7->ARR = cameraControlPwm.period;

            // Start two timers as simultaneously as possible
            ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
                TIM6->CR1 = TIM_CR1_CEN;
                TIM7->CR1 = TIM_CR1_CEN;
            }

            // Enable interrupt generation
            TIM6->DIER = TIM_IT_Update;
            TIM7->DIER = TIM_IT_Update;

            const uint32_t endTime = millis() + cameraControlConfig()->keyDelayMs;

            // Wait to give the camera a chance at registering the key press
            while (millis() < endTime);

            // Disable timers and interrupt generation
            TIM6->CR1 &= ~TIM_CR1_CEN;
            TIM7->CR1 &= ~TIM_CR1_CEN;
            TIM6->DIER = 0;
            TIM7->DIER = 0;

            // Reset to idle state
            IOHi(cameraControlPwm.io);
        }
#endif
    } else if (CAMERA_CONTROL_MODE_DAC == cameraControlConfig()->mode) {
        // @todo not yet implemented
    }
}

#endif
