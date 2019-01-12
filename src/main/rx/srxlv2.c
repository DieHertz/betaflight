#include "platform.h"

#include "common/crc.h"
#include "common/maths.h"

#include "io/serial.h"

#include "rx/srxlv2.h"
#include "rx/srxlv2_types.h"

#include <string.h>

void cliPrintf(const char *format, ...);

#define USE_SERIALRX_SRXLv2
#ifdef USE_SERIALRX_SRXLv2

#define SRXLv2_MAX_CHANNELS             32
#define SRXLv2_TIME_BETWEEN_FRAMES_US   11000
#define SRXLv2_CHANNEL_SHIFT            5
#define SRXLv2_CHANNEL_CENTER           0x8000

#define SRXLv2_PORT_BAUDRATE_DEFAULT    115200
#define SRXLv2_PORT_BAUDRATE_HIGH       400000
#define SRXLv2_PORT_OPTIONS             (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_BIDIR)
#define SRXLv2_PORT_MODE                MODE_RXTX

#define SRXLv2_ID                       0xA6
#define SRXLv2_MAX_PACKET_LENGTH        80
#define SRXLv2_DEVICE_ID_BROADCAST      0xFF

#define SRXLv2_FRAME_TIMEOUT_US         50000

// @note QUISCENCE_FOR_TWO_CHARACTERS (1 character is 1/baud?)

// @todo set from CLI
#define SRXLv2_DEVICE_ID                0

static srxlv2State state = Disabled;

static uint8_t read_buffer[SRXLv2_MAX_PACKET_LENGTH];
static uint8_t read_buffer_idx = 0;
static uint8_t write_buffer[SRXLv2_MAX_PACKET_LENGTH];
static uint8_t write_buffer_idx = 0;

static serialPort_t *serialPort;


bool srxlv2ProcessHandshake(const srxlv2Header* header, const srxlv2HandshakeSubHeader* handshake)
{
    if (handshake->destination_device_id == Broadcast) {
        cliPrintf("broadcast handshake from %x\r\n", handshake->source_device_id);

        if (handshake->baud_supported == 1) {
            // @todo reset back to default after failure
            serialPort->vTable->serialSetBaudRate(serialPort, SRXLv2_PORT_BAUDRATE_HIGH);
        }

        return true;
    }

    // @note using a range results in FC trying to answer multiple IDs, use just one?
    if (((handshake->destination_device_id >> 4) & 0xF) != FlightController ||
        (handshake->destination_device_id & 0xF) != SRXLv2_DEVICE_ID) {
        return true;
    }

    cliPrintf("FC handshake from %x\r\n", handshake->source_device_id);

    srxlv2HandshakeFrame response = {
        .header = *header,
        .payload = {
            handshake->destination_device_id,
            handshake->source_device_id,
            /* priority */ 0,
            /* baud_supported*/ 0,
            /* info */ 0,
            U_ID_2
        }
    };

    const uint16_t crc = crc16_ccitt_update(0, &response, sizeof(response) - 2);
    response.crc_high = ((uint8_t *) &crc)[1];
    response.crc_low = ((uint8_t *) &crc)[0];

    srxlv2RxWriteData(&response, sizeof(response));

    return true;
}

void srxlv2ProcessChannelData(const srxlv2ChannelDataHeader* channel_data) {
    if (channel_data->rssi >= 0) {
        const int rssi_percent = channel_data->rssi;
        setRssi(scaleRange(rssi_percent, 0, 100, 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_PROTOCOL);
    } else {
        // @note should it be negated?
        const int rssi_dbm = channel_data->rssi;
        // @todo come up with a scheme to scale power values to 0-1023 range
        setRssi(RSSI_MAX_VALUE / 2, RSSI_SOURCE_RX_PROTOCOL);
    }

    const uint16_t *frame_channels = (const uint16_t *) (channel_data + 1);

    uint16_t channels[SRXLv2_MAX_CHANNELS] = { 0 };

    if (channel_data->channel_mask.u8.channels_0_7) {
        for (size_t i = 0; i < 8; ++i) {
            if (channel_data->channel_mask.u8.channels_0_7 >> i & 1) {
                channels[i] = *frame_channels++;
            }
        }
    }

    if (channel_data->channel_mask.u8.channels_8_15) {
        for (size_t i = 0; i < 8; ++i) {
            if (channel_data->channel_mask.u8.channels_8_15 >> i & 1) {
                channels[8 + i] = *frame_channels++;
            }
        }
    }

    if (channel_data->channel_mask.u8.channels_16_23) {
        for (size_t i = 0; i < 8; ++i) {
            if (channel_data->channel_mask.u8.channels_16_23 >> i & 1) {
                channels[16 + i] = *frame_channels++;
            }
        }
    }

    if (channel_data->channel_mask.u8.channels_24_31) {
        for (size_t i = 0; i < 8; ++i) {
            if (channel_data->channel_mask.u8.channels_24_31 >> i & 1) {
                channels[24 + i] = *frame_channels++;
            }
        }
    }

    //cliPrintf("channel data: %d %d %x\r\n",
        //channel_data_header->rssi, channel_data_header->frame_losses, channel_data_header->channel_mask.u32);
}

bool srxlv2ProcessControlData(const srxlv2ControlDataSubHeader* control_data)
{
    if (control_data->reply_id) {
        cliPrintf("command: %x reply_id: %x\r\n", control_data->command, control_data->reply_id);
    }

    switch (control_data->command) {
        case ChannelData: {
            srxlv2ProcessChannelData((const srxlv2ChannelDataHeader *) (control_data + 1));
        } break;
        
        case FailsafeChannelData: {
            srxlv2ProcessChannelData((const srxlv2ChannelDataHeader *) (control_data + 1));
            setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
            cliPrintf("fs channel data\r\n");
        } break;

        case VTXData: {
            cliPrintf("vtx data\r\n");
        } break;
    }

    return true;
}

bool srxlv2ProcessPacket(const srxlv2Header* header)
{
    switch (header->packet_type) {
    case Handshake: return srxlv2ProcessHandshake(header, (const srxlv2HandshakeSubHeader *) (header + 1)); 
    case ControlData: return srxlv2ProcessControlData((const srxlv2ControlDataSubHeader *) (header + 1));
    default: break;
    }

    return false;
}

void srxlv2Process()
{
    const srxlv2Header* header = (const srxlv2Header *) read_buffer;

    if (srxlv2ProcessPacket(header)) {
        read_buffer_idx -= header->length;
        memmove(read_buffer, read_buffer + header->length, read_buffer_idx);
        return;
    }

    // @todo reset
    cliPrintf("could not parse packet: %x\r\n", header->packet_type);
    read_buffer_idx = 0;
}

static void srxlv2DataReceive(uint16_t character, void *data)
{
    UNUSED(data);

    if (0 == read_buffer_idx && character != SRXLv2_ID) {
        return;
    }

    read_buffer[read_buffer_idx++] = character;

    if (read_buffer_idx >= sizeof(srxlv2Header)) {
        const srxlv2Header* header = (const srxlv2Header *) read_buffer;

        if (header->id != SRXLv2_ID) {
            // @todo reset
            cliPrintf("invalid header id: %x\r\n", header->id);
            read_buffer_idx = 0;
            return /*RX_FRAME_DROPPED*/;
        }

        if (read_buffer_idx < header->length) {
            return /*RX_FRAME_PENDING*/;
        }

        srxlv2Process();

        // return RX_FRAME_COMPLETE if ControlData::ChannelData
        // return RX_FRAME_FAILSAFE if ControlData::FailsafeChannelData
        // return RX_FRAME_PROCESSING_REQUIRED if requires as answer
    }
}

static uint8_t srxlv2FrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    // @note coordinate "clear to send"
    if (write_buffer_idx) {
        return RX_FRAME_PROCESSING_REQUIRED;
    }

    return RX_FRAME_PENDING;
}

static bool srxlv2ProcessFrame(const rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    if (write_buffer_idx) {
        cliPrintf("sending response bytes %d\r\n", write_buffer_idx);

        serialWriteBuf(serialPort, write_buffer, write_buffer_idx);
        write_buffer_idx = 0;
    }

    return true;
}

static uint16_t srxlv2ReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel_idx)
{
    if (channel_idx >= rxRuntimeConfig->channelCount) {
        return 0;
    }

    return 476 + (rxRuntimeConfig->channelData[channel_idx] >> SRXLv2_CHANNEL_SHIFT);
}

void srxlv2RxWriteData(const void *data, int len)
{
    len = MIN(len, (int)sizeof(write_buffer));
    memcpy(write_buffer, data, len);
    write_buffer_idx = len;
}

bool srxlv2RxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    static uint16_t channelData[SRXLv2_MAX_CHANNELS];
    for (size_t i = 0; i < SRXLv2_MAX_CHANNELS; ++i) {
        channelData[i] = SRXLv2_CHANNEL_CENTER;
    }

    state = ListenForHandshake;

    rxRuntimeConfig->channelData = channelData;
    rxRuntimeConfig->channelCount = SRXLv2_MAX_CHANNELS;
    rxRuntimeConfig->rxRefreshRate = SRXLv2_TIME_BETWEEN_FRAMES_US;

    rxRuntimeConfig->rcReadRawFn = srxlv2ReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = srxlv2FrameStatus;
    rxRuntimeConfig->rcProcessFrameFn = srxlv2ProcessFrame;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    portOptions_e options = SRXLv2_PORT_OPTIONS;
    if (rxConfig->serialrx_inverted) {
        options |= SERIAL_INVERTED;
    }
    if (rxConfig->halfDuplex) {
        options |= SERIAL_BIDIR;
    }

    serialPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, srxlv2DataReceive,
        NULL, SRXLv2_PORT_BAUDRATE_DEFAULT, SRXLv2_PORT_MODE, options);

    if (serialPort) {
        if (rssiSource == RSSI_SOURCE_NONE) {
            rssiSource = RSSI_SOURCE_RX_PROTOCOL;
        }
    }

    return serialPort;
}

bool srxlv2RxIsActive(void)
{
    return serialPort;
}

#endif
