#include "opentco_device.h"

#include "drivers/serial.h"
#include "drivers/light_led.h"
#include "io/serial.h"
#include "drivers/opentco.h"

#include "scheduler/scheduler.h"

// @todo move to reworked generic OSD_SLAVE target
#define USE_OPENTCO_DEVICE
#ifdef USE_OPENTCO_DEVICE

static struct {
    serialPort_t *port;
} device;

bool opentcoDeviceInit(void)
{
    serialPortConfig_t *config = findSerialPortConfig(FUNCTION_OPENTCO_SERVER);
    if (config) {
        device.port = openSerialPort(config->identifier, FUNCTION_OPENTCO_SERVER, NULL,
            baudRates[BAUD_115200], MODE_RXTX, SERIAL_NOT_INVERTED);
    }

    if (!device.port) {
        return false;
    }

    setTaskEnabled(TASK_OPENTCO_DEVICE, true);

    return true;
}

void opentcoDeviceProcess(void)
{
    LED1_TOGGLE;

    // flush
    while (serialRxBytesWaiting(device.port)) {
        serialRead(device.port);
    }

    uint8_t data[128];
    uint8_t crc = 0;

    while (serialRxBytesWaiting(device.port)) {
        const uint8_t byte = serialRead(device.port);
        crc = crc8_dvb_s2(crc, byte);
    }
}

#endif
