#pragma once

#define PACKED __attribute__((packed))

typedef enum {
    Disabled,
    ListenOnStartup,
    SendHandshake,
    ListenForHandshake,
    Running
} srxlv2State;

typedef enum {
    Handshake = 0x21,
    BindInfo = 0x41,
    ParameterConfiguration = 0x50,
    SignalQuality = 0x55,
    TelemetrySensorData = 0x80,
    ControlData = 0xCD,
} srxlv2PacketType;

typedef struct {
    uint8_t id;
    uint8_t packet_type;
    uint8_t length;
} PACKED srxlv2Header;

typedef struct {
    uint8_t source_device_id;
    uint8_t destination_device_id;
    uint8_t priority;
    uint8_t baud_supported;
    uint8_t info;
    uint32_t unique_id;
} PACKED srxlv2HandshakeSubHeader;

typedef struct {
    uint8_t command;
    uint8_t reply_id;
} PACKED srxlv2ControlDataSubHeader;

typedef enum {
    ChannelData = 0x00,
    FailsafeChannelData = 0x01,
    VTXData = 0x02,
} srxlv2ControlDataCommand;

typedef struct {
    int8_t rssi;
    uint16_t frame_losses;
    union {
        struct {
            uint8_t channels_0_7;
            uint8_t channels_8_15;
            uint8_t channels_16_23;
            uint8_t channels_24_31;
        } u8;
        uint32_t u32;
    } channel_mask;
} PACKED srxlv2ChannelDataHeader;

typedef enum {
    NoDevice = 0,
    RemoteReceiver = 1,
    Receiver = 2,
    FlightController = 3,
    ESC = 4,
    Reserved = 5,
    SRXLServo = 6,
    SRXLServo_2 = 7,
    VTX = 8,
} srxlV2DeviceType;

typedef enum {
    FlightControllerDefault = 0x30,
    FlightControllerMax = 0x3F,
    Broadcast = 0xFF,
} srxlv2DeviceId;

typedef struct {
    srxlv2Header header;
    srxlv2HandshakeSubHeader payload;
    uint8_t crc_high;
    uint8_t crc_low;
} PACKED srxlv2HandshakeFrame;

#undef PACKED
