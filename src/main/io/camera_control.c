#ifdef USE_CAMERA_CONTROL

#include "camera_control.h"
#include "drivers/camera_control.h"
#include "fc/rc_modes.h"

#define CAMERA_CONTROL_UPDATE_INTERVAL_US  50000   // Interval of key scans (microsec)
#define CAMERA_CONTROL_POLL_INTERVAL_US   100000   // Interval of polling dynamic values (microsec)

#define IS_HI(X)  (rcData[X] > 1750)
#define IS_LO(X)  (rcData[X] < 1250)
#define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)


void cameraControlKeyHandler(timeUs_t currentTimeUs) {
	static timeUs_t lastCalledUs = 0;

	if (currentTimeUs >= lastCalledUs + CAMERA_CONTROL_UPDATE_INTERVAL_US) {
		lastCalledUs = currentTimeUs;
		cameraControlUpdate(currentTimeUs);
	}
}

void cameraControlUpdate(timeUs_t currentTimeUs) {
	static int16_t keyDelayMs = cameraControlConfig()->keyDelayMs;
	static const int16_t keyPauseMs = 2 * keyDelayMs;

    static int holdCount = 1;
    static int repeatCount = 1;
    static int repeatBase = 0;

    static uint32_t lastCalledMs = 0;

    const uint32_t currentTimeMs = currentTimeUs / 1000;

	if (!IS_RC_MODE_ACTIVE(BOXCAMERA1) && 1) {
		if (IS_MID(THROTTLE) && IS_HI(YAW) && IS_MID(PITCH) && IS_MID(ROLL) && !ARMING_FLAG(ARMED)) {
			keyDelayMs = keyPauseMs;

			// @todo enter menu
		}

		return;
	}

    uint8_t key = CAMERA_CONTROL_KEYS_COUNT;
    uint16_t delayMs = 0;

	if (IS_HI(YAW)) {
		key = CAMERA_CONTROL_KEY_ENTER;
	} else if (IS_LO(ROLL)) {
		key = CAMERA_CONTROL_KEY_LEFT;
	} else if (IS_HI(PITCH)) {
		key = CAMERA_CONTROL_KEY_UP;
	} else if (IS_HI(ROLL)) {
		key = CAMERA_CONTROL_KEY_RIGHT;
	} else if (IS_LO(PITCH)) {
		key = CAMERA_CONTROL_KEY_DOWN;
	} else if (IS_HI(PITCH) && IS_LO(THROTTLE)) {
        key = CAMERA_CONTROL_KEY_UP
        delayMs = 2000;
    } else if (IS_LO(YAW)) {
    	// @todo exit menu
    }

    if (key == CAMERA_CONTROL_KEYS_COUNT) {
    	holdCount = 1;
    	repeatCount = 1;
    	repeatBase = 0;
    } else {
    	++holdCount;
    }

    if (keyDelayMs > 0) {
    	keyDelayMs -= (currentTimeMs - lastCalledMs);
    } else if (key) {
    	keyDelayMs = cameraControlHandleKeyWithRepeat(key, repeatCount);

    	// @todo some shitty code here
    	// if key held for more than
    	if (holdCount > 20) {

    	}
    }

    // @todo enter mode omits this line
    lastCalledMs = millis();
}

#endif
