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

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#ifdef USE_TARGET_CONFIG

#include "io/serial.h"

#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "flight/pid.h"
#include "flight/mixer.h"
#include "config/feature.h"

#include "pg/max7456.h"
#include "pg/pg.h"
#include "build/debug.h"
#include "common/filter.h"

#include "rx/rx.h"

void targetConfiguration(void)
{
#ifdef OMNIBUSF4BASE
    // OMNIBUS F4 AIO (1st gen) has a AB7456 chip that is detected as MAX7456
    max7456ConfigMutable()->clockConfig = MAX7456_CLOCK_CONFIG_FULL;
#endif

#ifdef EXUAVF4PRO
    serialConfigMutable()->portConfigs[1].functionMask = FUNCTION_TELEMETRY_SMARTPORT;
    serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_VTX_TRAMP;
    serialConfigMutable()->portConfigs[3].functionMask = FUNCTION_RCDEVICE;
    serialConfigMutable()->portConfigs[4].functionMask = FUNCTION_RX_SERIAL;
#endif

#if defined(UNDERGROUNDFPV)
    featureSet(FEATURE_AIRMODE | FEATURE_ANTI_GRAVITY | FEATURE_DYNAMIC_FILTER);
    gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;
    gyroConfigMutable()->gyro_filter_q = 300;
    gyroConfigMutable()->gyro_filter_r = 80;
    gyroConfigMutable()->gyroMovementCalibrationThreshold = 128;
    gyroConfigMutable()->gyro_use_32khz = true;
    systemConfigMutable()->cpu_overclock = 1;
    systemConfigMutable()->debug_mode = DEBUG_FFT;
    accelerometerConfigMutable()->acc_hardware = ACC_NONE;
    rxConfigMutable()->mincheck = 1020;
    rxConfigMutable()->rcInterpolation = RC_SMOOTHING_MANUAL;
    rxConfigMutable()->rcInterpolationChannels = 2;
    rxConfigMutable()->rcInterpolationInterval = 14;
    motorConfigMutable()->minthrottle = 1050;
    motorConfigMutable()->digitalIdleOffsetValue = 500;
    motorConfigMutable()->dev.useUnsyncedPwm = 1;
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_MULTISHOT;
    motorConfigMutable()->dev.motorPwmRate = 32000;
    rcControlsConfigMutable()->deadband = 5;
    rcControlsConfigMutable()->yaw_deadband = 5;
    pidConfigMutable()->pid_process_denom = 2;
    pidProfilesMutable(0)->dterm_filter_type = FILTER_PT1;
    pidProfilesMutable(0)->dterm_lpf_hz = 80;
    pidProfilesMutable(0)->pid[PID_PITCH].P = 61;
    pidProfilesMutable(0)->pid[PID_PITCH].I = 55;
    pidProfilesMutable(0)->pid[PID_PITCH].D = 20;
    pidProfilesMutable(0)->pid[PID_ROLL].P = 46;
    pidProfilesMutable(0)->pid[PID_ROLL].I = 45;
    pidProfilesMutable(0)->pid[PID_ROLL].D = 20;
    pidProfilesMutable(0)->pid[PID_YAW].P = 60;
    pidProfilesMutable(0)->pid[PID_YAW].I = 55;
    pidProfilesMutable(0)->pid[PID_YAW].D = 10;

    const int index_uart4 = findSerialPortIndexByIdentifier(SERIAL_PORT_UART4);
    if (index_uart4 >= 0) {
        serialConfigMutable()->portConfigs[index_uart4].functionMask = FUNCTION_ESC_SENSOR;
    }

    const int index_uart1 = findSerialPortIndexByIdentifier(SERIAL_PORT_USART1);
    if (index_uart1 >= 0) {
        serialConfigMutable()->portConfigs[index_uart1].functionMask = FUNCTION_RX_SERIAL;
        rxConfigMutable()->serialrx_provider = SERIALRX_SBUS;
    }
#endif
}
#endif
