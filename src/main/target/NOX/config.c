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

#include "build/debug.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "flight/pid.h"
#include "flight/mixer.h"
#include "config/feature.h"

#include "common/filter.h"

#include "rx/rx.h"

void targetConfiguration(void)
{
    // SOFTSERIAL1 is ESC telemetry input
    const int index = findSerialPortIndexByIdentifier(SERIAL_PORT_SOFTSERIAL1);
    if (index >= 0) {
        serialConfigMutable()->portConfigs[index].functionMask = FUNCTION_ESC_SENSOR;
    }

#ifdef UNDERGROUNDFPV
    featureSet(FEATURE_AIRMODE | FEATURE_ANTI_GRAVITY | FEATURE_DYNAMIC_FILTER);
    gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;
    gyroConfigMutable()->gyro_filter_q = 300;
    gyroConfigMutable()->gyro_filter_r = 80;
    gyroConfigMutable()->gyroMovementCalibrationThreshold = 128;
    systemConfigMutable()->debug_mode = DEBUG_FFT;
    accelerometerConfigMutable()->acc_hardware = ACC_NONE;
    rxConfigMutable()->mincheck = 1020;
    rxConfigMutable()->rcInterpolation = RC_SMOOTHING_MANUAL;
    rxConfigMutable()->rcInterpolationChannels = 2;
    rxConfigMutable()->rcInterpolationInterval = 14;
    motorConfigMutable()->minthrottle = 1050;
    motorConfigMutable()->dev.useUnsyncedPwm = 1;
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_MULTISHOT;
    motorConfigMutable()->dev.motorPwmRate = 32000;
    rcControlsConfigMutable()->deadband = 5;
    rcControlsConfigMutable()->yaw_deadband = 5;
    pidConfigMutable()->pid_process_denom = 1;
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
#endif
}
#endif
