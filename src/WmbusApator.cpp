// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * CC1101 receiver for Apator 16-2 water meters.
 *
 * wM-Bus frame handling and Apator register sizes are based on
 * wmbusmeters (see https://github.com/wmbusmeters/wmbusmeters/blob/ff7c7d288cc866b881ffbbe8d23da005cf65e1c9/drivers/src/apator162.xmq)
 */
#include "WmbusApator.h"

#ifdef OPENDTU_WMBUS_APATOR

#include "MqttSettings.h"
#include <SpiManager.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <esp_heap_caps.h>
#include <inttypes.h>
#include <mbedtls/aes.h>
#include <memory>

#undef TAG
static const char* TAG = "wmbus-apator";

#ifndef WMBUS_CC1101_PIN_SCLK
#define WMBUS_CC1101_PIN_SCLK GPIO_NUM_22
#endif
#ifndef WMBUS_CC1101_PIN_MOSI
#define WMBUS_CC1101_PIN_MOSI GPIO_NUM_21
#endif
#ifndef WMBUS_CC1101_PIN_MISO
#define WMBUS_CC1101_PIN_MISO GPIO_NUM_27
#endif
#ifndef WMBUS_CC1101_PIN_CS
#define WMBUS_CC1101_PIN_CS GPIO_NUM_26
#endif
#ifndef WMBUS_CC1101_PIN_GDO0
#define WMBUS_CC1101_PIN_GDO0 GPIO_NUM_25
#endif

#ifndef WMBUS_APATOR_METER_ID
#define WMBUS_APATOR_METER_ID 0U
#endif
#ifndef WMBUS_APATOR_KEY
#define WMBUS_APATOR_KEY "00000000000000000000000000000000"
#endif
#ifndef WMBUS_APATOR_MQTT_TOPIC_PREFIX
#define WMBUS_APATOR_MQTT_TOPIC_PREFIX "water_meter"
#endif
#ifndef WMBUS_APATOR_MQTT_TOPIC_RSSI
#define WMBUS_APATOR_MQTT_TOPIC_RSSI WMBUS_APATOR_MQTT_TOPIC_PREFIX "/packet_rssi"
#endif
#ifndef WMBUS_APATOR_MQTT_TOPIC_TOTAL
#define WMBUS_APATOR_MQTT_TOPIC_TOTAL WMBUS_APATOR_MQTT_TOPIC_PREFIX "/total_m3"
#endif
#ifndef WMBUS_APATOR_MQTT_TOPIC_AVAILABILITY
#define WMBUS_APATOR_MQTT_TOPIC_AVAILABILITY WMBUS_APATOR_MQTT_TOPIC_PREFIX "/availability"
#endif

namespace {
constexpr uint32_t AvailabilityTimeoutMs = 30UL * 60UL * 1000UL;
constexpr uint32_t RadioFrequencyHz = 868950000UL;
constexpr uint32_t Cc1101CrystalHz = 26000000UL;
constexpr size_t MaxRawPacketSize = 384;
constexpr size_t Cc1101FifoBytes = 64;
constexpr uint32_t ReadTimeoutMs = 500;
constexpr size_t T1ProbeEncodedBytes = 3;
constexpr size_t SpiDmaAlignment = 4;
constexpr size_t Cc1101BurstBufferBytes = ((Cc1101FifoBytes + 1 + SpiDmaAlignment - 1) / SpiDmaAlignment) * SpiDmaAlignment;
constexpr uint16_t ManufacturerApa = ((('A' - 64) * 1024) + (('P' - 64) * 32) + ('A' - 64));
constexpr int16_t Cc1101RssiOffsetDb = 74;

constexpr int16_t cc1101RssiRegisterToDbm(uint8_t rawRssi)
{
    const int16_t signedRssi = rawRssi >= 128
        ? static_cast<int16_t>(rawRssi) - 256
        : static_cast<int16_t>(rawRssi);
    return (signedRssi / 2) - Cc1101RssiOffsetDb;
}

static_assert(cc1101RssiRegisterToDbm(0x00) == -74);
static_assert(cc1101RssiRegisterToDbm(0xFF) == -74);
static_assert(cc1101RssiRegisterToDbm(0xFE) == -75);
static_assert(cc1101RssiRegisterToDbm(0x80) == -138);

constexpr char TopicRssi[] = WMBUS_APATOR_MQTT_TOPIC_RSSI;
constexpr char TopicTotal[] = WMBUS_APATOR_MQTT_TOPIC_TOTAL;
constexpr char TopicAvailability[] = WMBUS_APATOR_MQTT_TOPIC_AVAILABILITY;

constexpr uint8_t CC1101_SRES = 0x30;
constexpr uint8_t CC1101_SCAL = 0x33;
constexpr uint8_t CC1101_SRX = 0x34;
constexpr uint8_t CC1101_SIDLE = 0x36;
constexpr uint8_t CC1101_SFRX = 0x3A;
constexpr uint8_t CC1101_IOCFG2 = 0x00;
constexpr uint8_t CC1101_IOCFG1 = 0x01;
constexpr uint8_t CC1101_IOCFG0 = 0x02;
constexpr uint8_t CC1101_FIFOTHR = 0x03;
constexpr uint8_t CC1101_SYNC1 = 0x04;
constexpr uint8_t CC1101_SYNC0 = 0x05;
constexpr uint8_t CC1101_PKTLEN = 0x06;
constexpr uint8_t CC1101_PKTCTRL1 = 0x07;
constexpr uint8_t CC1101_PKTCTRL0 = 0x08;
constexpr uint8_t CC1101_ADDR = 0x09;
constexpr uint8_t CC1101_CHANNR = 0x0A;
constexpr uint8_t CC1101_FSCTRL1 = 0x0B;
constexpr uint8_t CC1101_FSCTRL0 = 0x0C;
constexpr uint8_t CC1101_FREQ2 = 0x0D;
constexpr uint8_t CC1101_FREQ1 = 0x0E;
constexpr uint8_t CC1101_FREQ0 = 0x0F;
constexpr uint8_t CC1101_MDMCFG4 = 0x10;
constexpr uint8_t CC1101_MDMCFG3 = 0x11;
constexpr uint8_t CC1101_MDMCFG2 = 0x12;
constexpr uint8_t CC1101_MDMCFG1 = 0x13;
constexpr uint8_t CC1101_MDMCFG0 = 0x14;
constexpr uint8_t CC1101_DEVIATN = 0x15;
constexpr uint8_t CC1101_MCSM2 = 0x16;
constexpr uint8_t CC1101_MCSM1 = 0x17;
constexpr uint8_t CC1101_MCSM0 = 0x18;
constexpr uint8_t CC1101_FOCCFG = 0x19;
constexpr uint8_t CC1101_BSCFG = 0x1A;
constexpr uint8_t CC1101_AGCCTRL2 = 0x1B;
constexpr uint8_t CC1101_AGCCTRL1 = 0x1C;
constexpr uint8_t CC1101_AGCCTRL0 = 0x1D;
constexpr uint8_t CC1101_WOREVT1 = 0x1E;
constexpr uint8_t CC1101_WOREVT0 = 0x1F;
constexpr uint8_t CC1101_WORCTRL = 0x20;
constexpr uint8_t CC1101_FREND1 = 0x21;
constexpr uint8_t CC1101_FREND0 = 0x22;
constexpr uint8_t CC1101_FSCAL3 = 0x23;
constexpr uint8_t CC1101_FSCAL2 = 0x24;
constexpr uint8_t CC1101_FSCAL1 = 0x25;
constexpr uint8_t CC1101_FSCAL0 = 0x26;
constexpr uint8_t CC1101_RCCTRL1 = 0x27;
constexpr uint8_t CC1101_RCCTRL0 = 0x28;
constexpr uint8_t CC1101_FSTEST = 0x29;
constexpr uint8_t CC1101_PTEST = 0x2A;
constexpr uint8_t CC1101_AGCTEST = 0x2B;
constexpr uint8_t CC1101_TEST2 = 0x2C;
constexpr uint8_t CC1101_TEST1 = 0x2D;
constexpr uint8_t CC1101_TEST0 = 0x2E;

constexpr uint8_t CC1101_PARTNUM = 0x30;
constexpr uint8_t CC1101_VERSION = 0x31;
constexpr uint8_t CC1101_RSSI = 0x34;
constexpr uint8_t CC1101_MARCSTATE = 0x35;
constexpr uint8_t CC1101_RXBYTES = 0x3B;
constexpr uint8_t CC1101_RXFIFO = 0x3F;

constexpr uint8_t CC1101_WRITE_SINGLE = 0x00;
constexpr uint8_t CC1101_READ_SINGLE = 0x80;
constexpr uint8_t CC1101_READ_BURST = 0xC0;

constexpr uint8_t CC1101_MARCSTATE_IDLE = 0x01;
constexpr uint8_t CC1101_MARCSTATE_RX_END = 0x0E;

constexpr uint8_t CC1101_GDO_SYNC_WORD = 0x06;
constexpr uint8_t CC1101_GDO_HI_Z = 0x2E;

bool decode3of6(const std::vector<uint8_t>& coded, std::vector<uint8_t>& decoded)
{
    static constexpr uint8_t Invalid = 0xFF;
    static constexpr uint8_t lookup[64] = {
        Invalid, Invalid, Invalid, Invalid, Invalid, Invalid, Invalid, Invalid,
        Invalid, Invalid, Invalid, 0x03, Invalid, 0x01, 0x02, Invalid,
        Invalid, Invalid, Invalid, 0x07, Invalid, Invalid, 0x00, Invalid,
        Invalid, 0x05, 0x06, Invalid, 0x04, Invalid, Invalid, Invalid,
        Invalid, Invalid, Invalid, 0x0B, Invalid, 0x09, 0x0A, Invalid,
        Invalid, 0x0F, Invalid, Invalid, 0x08, Invalid, Invalid, Invalid,
        Invalid, 0x0D, 0x0E, Invalid, 0x0C, Invalid, Invalid, Invalid,
        Invalid, Invalid, Invalid, Invalid, Invalid, Invalid, Invalid, Invalid,
    };

    const size_t segments = (coded.size() * 8) / 6;
    if ((segments % 2) != 0) {
        return false;
    }

    decoded.clear();
    decoded.reserve(segments / 2);
    for (size_t i = 0; i < segments; ++i) {
        const size_t bitIdx = i * 6;
        const size_t byteIdx = bitIdx / 8;
        const uint8_t bitOffset = bitIdx % 8;
        if (byteIdx >= coded.size()) {
            return false;
        }

        uint16_t window = static_cast<uint16_t>(coded[byteIdx]) << 8;
        if (byteIdx + 1 < coded.size()) {
            window |= coded[byteIdx + 1];
        }

        const uint8_t code = static_cast<uint8_t>((window >> (10 - bitOffset)) & 0x3F);
        const uint8_t nibble = lookup[code];
        if (nibble == Invalid) {
            return false;
        }

        if ((i % 2) == 0) {
            decoded.push_back(static_cast<uint8_t>(nibble << 4));
        } else {
            decoded.back() |= nibble;
        }
    }

    return true;
}

size_t encoded3of6Size(size_t decodedSize)
{
    return (decodedSize * 3 + 1) / 2;
}

size_t frameFormatABlockCount(uint8_t lField)
{
    return lField < 26 ? 2 : (static_cast<size_t>(lField) - 26) / 16 + 3;
}

size_t frameFormatARawSize(uint8_t lField)
{
    return static_cast<size_t>(lField) + 1 + 2 * frameFormatABlockCount(lField);
}

uint16_t crc16En13757(const uint8_t* data, size_t len)
{
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; ++i) {
        uint8_t b = data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (((crc & 0x8000) >> 8) ^ (b & 0x80)) {
                crc = (crc << 1) ^ 0x3D65;
            } else {
                crc = (crc << 1);
            }
            b <<= 1;
        }
    }
    return ~crc;
}

bool trimCrcsFrameFormatA(std::vector<uint8_t>& payload)
{
    if (payload.size() < 12) {
        return false;
    }

    const size_t len = payload.size();
    std::vector<uint8_t> out;
    out.reserve(len);

    uint16_t calcCrc = crc16En13757(payload.data(), 10);
    uint16_t checkCrc = (static_cast<uint16_t>(payload[10]) << 8) | payload[11];
    if (calcCrc != checkCrc) {
        return false;
    }
    out.insert(out.end(), payload.begin(), payload.begin() + 10);

    size_t pos = 12;
    for (; pos + 18 <= len; pos += 18) {
        const size_t crcPos = pos + 16;
        calcCrc = crc16En13757(&payload[pos], 16);
        checkCrc = (static_cast<uint16_t>(payload[crcPos]) << 8) | payload[crcPos + 1];
        if (calcCrc != checkCrc) {
            return false;
        }
        out.insert(out.end(), payload.begin() + pos, payload.begin() + crcPos);
    }

    if (pos < len - 2) {
        const size_t crcPos = len - 2;
        calcCrc = crc16En13757(&payload[pos], crcPos - pos);
        checkCrc = (static_cast<uint16_t>(payload[crcPos]) << 8) | payload[crcPos + 1];
        if (calcCrc != checkCrc) {
            return false;
        }
        out.insert(out.end(), payload.begin() + pos, payload.begin() + crcPos);
    }

    out[0] = out.size() - 1;
    payload = std::move(out);
    return true;
}

bool hexNibble(char c, uint8_t& out)
{
    if (c >= '0' && c <= '9') {
        out = c - '0';
        return true;
    }
    if (c >= 'a' && c <= 'f') {
        out = c - 'a' + 10;
        return true;
    }
    if (c >= 'A' && c <= 'F') {
        out = c - 'A' + 10;
        return true;
    }
    return false;
}

bool configuredKey(std::vector<uint8_t>& key)
{
    const char* hex = WMBUS_APATOR_KEY;
    const size_t len = strlen(hex);
    if (len == 0 || strcmp(hex, "NOKEY") == 0) {
        key.clear();
        return true;
    }
    if (len != 32) {
        return false;
    }

    key.resize(16);
    for (size_t i = 0; i < key.size(); ++i) {
        uint8_t hi;
        uint8_t lo;
        if (!hexNibble(hex[i * 2], hi) || !hexNibble(hex[i * 2 + 1], lo)) {
            key.clear();
            return false;
        }
        key[i] = (hi << 4) | lo;
    }
    return true;
}

int apatorRegisterSize(uint8_t reg)
{
    switch (reg) {
    case 0x00:
        return 4;
    case 0x01:
        return 3;
    case 0x10:
    case 0xA1:
        return 4;
    case 0x11:
        return 2;
    case 0x40:
        return 6;
    case 0x41:
        return 2;
    case 0x42:
        return 4;
    case 0x43:
        return 2;
    case 0x44:
        return 3;
    case 0x71:
        return 1 + 2 * 4;
    case 0x72:
        return 1 + 3 * 4;
    case 0x73:
        return 1 + 4 * 4;
    case 0x74:
        return 1 + 5 * 4;
    case 0x75:
        return 1 + 6 * 4;
    case 0x76:
        return 1 + 7 * 4;
    case 0x77:
        return 1 + 8 * 4;
    case 0x78:
        return 1 + 9 * 4;
    case 0x79:
        return 1 + 10 * 4;
    case 0x7A:
        return 1 + 11 * 4;
    case 0x7B:
        return 1 + 12 * 4;
    case 0x7C:
        return 1 + 13 * 4;
    case 0x80:
    case 0x81:
    case 0x82:
    case 0x83:
    case 0x84:
    case 0x86:
    case 0x87:
        return 10;
    case 0x85:
    case 0x88:
    case 0x8F:
        return 11;
    case 0x8A:
        return 9;
    case 0x8B:
    case 0x8C:
        return 6;
    case 0x8E:
        return 7;
    case 0xA0:
        return 4;
    case 0xA2:
        return 1;
    case 0xA3:
        return 7;
    case 0xA4:
        return 4;
    case 0xA5:
    case 0xA9:
    case 0xAF:
        return 1;
    case 0xA6:
        return 3;
    case 0xA7:
    case 0xA8:
    case 0xAA:
    case 0xAB:
    case 0xAC:
    case 0xAD:
        return 2;
    case 0xB0:
        return 5;
    case 0xB1:
        return 8;
    case 0xB2:
        return 16;
    case 0xB3:
        return 8;
    case 0xB4:
        return 2;
    case 0xB5:
        return 16;
    case 0xB6:
    case 0xB7:
    case 0xB8:
    case 0xB9:
    case 0xBA:
    case 0xBB:
    case 0xBC:
    case 0xBD:
    case 0xBE:
    case 0xBF:
    case 0xC0:
    case 0xC1:
    case 0xC2:
    case 0xC3:
    case 0xC4:
    case 0xC5:
    case 0xC6:
    case 0xC7:
    case 0xD0:
    case 0xD3:
        return 3;
    case 0xF0:
        return 4;
    default:
        return -1;
    }
}

}

WmbusApatorClass WmbusApator;

WmbusApatorClass::WmbusApatorClass()
    : _loopTask(TASK_SECOND, TASK_FOREVER, std::bind(&WmbusApatorClass::loop, this))
{
}

void WmbusApatorClass::init(Scheduler& scheduler)
{
    ESP_LOGI(TAG, "Initializing CC1101 Apator");
    if (!setupRadio()) {
        ESP_LOGE(TAG, "CC1101 Apator disabled");
        return;
    }

    scheduler.addTask(_loopTask);
    _loopTask.enable();
}

bool WmbusApatorClass::setupRadio()
{
    _telegramQueue = xQueueCreate(3, sizeof(QueuedTelegram));
    if (_telegramQueue == nullptr) {
        ESP_LOGE(TAG, "Failed to create telegram queue");
        return false;
    }

    auto busConfig = std::make_shared<SpiBusConfig>(
        static_cast<gpio_num_t>(WMBUS_CC1101_PIN_MOSI),
        static_cast<gpio_num_t>(WMBUS_CC1101_PIN_MISO),
        static_cast<gpio_num_t>(WMBUS_CC1101_PIN_SCLK));

    spi_device_interface_config_t deviceConfig {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 1000000,
        .input_delay_ns = 0,
        .spics_io_num = static_cast<int>(WMBUS_CC1101_PIN_CS),
        .flags = 0,
        .queue_size = 1,
        .pre_cb = nullptr,
        .post_cb = nullptr,
    };

    _spi = SpiManagerInst.alloc_device("", busConfig, deviceConfig);
    if (_spi == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate SPI device");
        return false;
    }

    _spiBurstTx = static_cast<uint8_t*>(heap_caps_calloc(Cc1101BurstBufferBytes, 1, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    _spiBurstRx = static_cast<uint8_t*>(heap_caps_calloc(Cc1101BurstBufferBytes, 1, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (_spiBurstTx == nullptr || _spiBurstRx == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate CC1101 DMA buffers");
        if (_spiBurstTx != nullptr) {
            heap_caps_free(_spiBurstTx);
            _spiBurstTx = nullptr;
        }
        if (_spiBurstRx != nullptr) {
            heap_caps_free(_spiBurstRx);
            _spiBurstRx = nullptr;
        }
        return false;
    }

    pinMode(static_cast<uint8_t>(WMBUS_CC1101_PIN_GDO0), INPUT);

    if (!configureCc1101()) {
        return false;
    }

    attachInterruptArg(
        digitalPinToInterrupt(static_cast<uint8_t>(WMBUS_CC1101_PIN_GDO0)),
        &WmbusApatorClass::handleInterrupt,
        this,
        FALLING);

#if portNUM_PROCESSORS > 1
    BaseType_t taskOk = xTaskCreatePinnedToCore(&WmbusApatorClass::receiverTask, "wmbus_cc1101", 8192, this, 24, &_receiverTaskHandle, 1);
#else
    BaseType_t taskOk = xTaskCreate(&WmbusApatorClass::receiverTask, "wmbus_cc1101", 8192, this, 24, &_receiverTaskHandle);
#endif
    if (taskOk != pdPASS) {
        ESP_LOGE(TAG, "Failed to create receiver task");
        return false;
    }

    ESP_LOGI(TAG,
        "CC1101 pins: SCLK=%d MOSI=%d MISO=%d CS=%d GDO0=%d, meter filter=%08" PRIx32,
        static_cast<int>(WMBUS_CC1101_PIN_SCLK),
        static_cast<int>(WMBUS_CC1101_PIN_MOSI),
        static_cast<int>(WMBUS_CC1101_PIN_MISO),
        static_cast<int>(WMBUS_CC1101_PIN_CS),
        static_cast<int>(WMBUS_CC1101_PIN_GDO0),
        static_cast<uint32_t>(WMBUS_APATOR_METER_ID));
    return true;
}

bool WmbusApatorClass::configureCc1101()
{
    strobe(CC1101_SRES);
    delay(10);

    const uint8_t partnum = readStatusRegister(CC1101_PARTNUM);
    const uint8_t version = readStatusRegister(CC1101_VERSION);
    ESP_LOGI(TAG, "CC1101 part=%02x version=%02x", partnum, version);
    if (partnum != 0x00) {
        ESP_LOGE(TAG, "Invalid CC1101 part number");
        return false;
    }

    writeRegister(CC1101_IOCFG2, CC1101_GDO_HI_Z);
    writeRegister(CC1101_IOCFG1, CC1101_GDO_HI_Z);
    writeRegister(CC1101_IOCFG0, CC1101_GDO_SYNC_WORD | 0x40);
    writeRegister(CC1101_FIFOTHR, 0x07);
    writeRegister(CC1101_SYNC1, 0x54);
    writeRegister(CC1101_SYNC0, 0x3D);
    writeRegister(CC1101_PKTLEN, 0xFF);
    writeRegister(CC1101_PKTCTRL1, 0x00);
    writeRegister(CC1101_PKTCTRL0, 0x00);
    writeRegister(CC1101_ADDR, 0x00);
    writeRegister(CC1101_CHANNR, 0x00);
    writeRegister(CC1101_FSCTRL1, 0x08);
    writeRegister(CC1101_FSCTRL0, 0x00);

    const uint32_t freqReg = (static_cast<uint64_t>(RadioFrequencyHz) << 16) / Cc1101CrystalHz;
    writeRegister(CC1101_FREQ2, static_cast<uint8_t>(freqReg >> 16));
    writeRegister(CC1101_FREQ1, static_cast<uint8_t>(freqReg >> 8));
    writeRegister(CC1101_FREQ0, static_cast<uint8_t>(freqReg));

    writeRegister(CC1101_MDMCFG4, 0x5C);
    writeRegister(CC1101_MDMCFG3, 0x04);
    writeRegister(CC1101_MDMCFG2, 0x05);
    writeRegister(CC1101_MDMCFG1, 0x22);
    writeRegister(CC1101_MDMCFG0, 0xF8);
    writeRegister(CC1101_DEVIATN, 0x44);
    writeRegister(CC1101_MCSM2, 0x07);
    writeRegister(CC1101_MCSM1, 0x00);
    writeRegister(CC1101_MCSM0, 0x18);
    writeRegister(CC1101_FOCCFG, 0x2E);
    writeRegister(CC1101_BSCFG, 0xBF);
    writeRegister(CC1101_AGCCTRL2, 0x43);
    writeRegister(CC1101_AGCCTRL1, 0x09);
    writeRegister(CC1101_AGCCTRL0, 0xB5);
    writeRegister(CC1101_WOREVT1, 0x87);
    writeRegister(CC1101_WOREVT0, 0x6B);
    writeRegister(CC1101_WORCTRL, 0xFB);
    writeRegister(CC1101_FREND1, 0xB6);
    writeRegister(CC1101_FREND0, 0x10);
    writeRegister(CC1101_FSCAL3, 0xEA);
    writeRegister(CC1101_FSCAL2, 0x2A);
    writeRegister(CC1101_FSCAL1, 0x00);
    writeRegister(CC1101_FSCAL0, 0x1F);
    writeRegister(CC1101_RCCTRL1, 0x41);
    writeRegister(CC1101_RCCTRL0, 0x00);
    writeRegister(CC1101_FSTEST, 0x59);
    writeRegister(CC1101_PTEST, 0x7F);
    writeRegister(CC1101_AGCTEST, 0x3F);
    writeRegister(CC1101_TEST2, 0x81);
    writeRegister(CC1101_TEST1, 0x35);
    writeRegister(CC1101_TEST0, 0x09);

    strobe(CC1101_SCAL);
    delay(1);
    restartRx();
    return true;
}

void WmbusApatorClass::loop()
{
    if (MqttSettings.getConnected() && !_availabilityPublished) {
        publishAvailability(false);
    }

    QueuedTelegram telegram { };
    while (xQueueReceive(_telegramQueue, &telegram, 0) == pdPASS) {
        publishTelegram(telegram);
    }

    if (_availabilityOnline && millis() - _lastTelegramMillis > AvailabilityTimeoutMs) {
        publishAvailability(false);
    }
}

bool WmbusApatorClass::readFrame(std::vector<uint8_t>& frame, int16_t& rssiDbm)
{
    std::vector<uint8_t> encoded(T1ProbeEncodedBytes);
    if (!readBytes(encoded.data(), encoded.size(), ReadTimeoutMs)) {
        return false;
    }

    std::vector<uint8_t> decodedPrefix;
    if (!decode3of6(encoded, decodedPrefix) || decodedPrefix.empty()) {
        return false;
    }

    const uint8_t lField = decodedPrefix[0];
    const size_t expectedDecodedSize = frameFormatARawSize(lField);
    const size_t expectedEncodedSize = encoded3of6Size(expectedDecodedSize);
    if (expectedDecodedSize < 12 || expectedDecodedSize > MaxRawPacketSize || expectedEncodedSize < encoded.size() || expectedEncodedSize > MaxRawPacketSize) {
        return false;
    }

    encoded.resize(expectedEncodedSize);
    if (!readBytes(encoded.data() + T1ProbeEncodedBytes, expectedEncodedSize - T1ProbeEncodedBytes, ReadTimeoutMs)) {
        return false;
    }

    std::vector<uint8_t> decoded;
    if (!decode3of6(encoded, decoded) || decoded.size() != expectedDecodedSize) {
        return false;
    }

    rssiDbm = cc1101RssiRegisterToDbm(_lastRssiRegister);

    frame = std::move(decoded);
    return trimCrcsFrameFormatA(frame);
}

bool WmbusApatorClass::readBytes(uint8_t* buffer, size_t length, uint32_t timeoutMs)
{
    size_t total = 0;
    uint32_t lastProgress = millis();

    while (total < length) {
        const uint8_t rxBytesRaw = readStatusRegister(CC1101_RXBYTES);
        if (rxBytesRaw & 0x80) {
            ESP_LOGW(TAG, "CC1101 RX FIFO overflow");
            return false;
        }

        const uint8_t available = rxBytesRaw & 0x7F;
        const size_t remaining = length - total;
        if (available > 0) {
            const size_t toRead = (remaining > 1 && available > 1)
                ? std::min(static_cast<size_t>(available - 1), remaining)
                : std::min(static_cast<size_t>(available), remaining);

            if (toRead > 0) {
                if (total == 0) {
                    _lastRssiRegister = readStatusRegister(CC1101_RSSI);
                }
                if (!readBurst(CC1101_RXFIFO, buffer + total, toRead)) {
                    return false;
                }
                total += toRead;
                lastProgress = millis();
            }
        }

        if (total > 0 && total < length) {
            const uint8_t marcstate = readStatusRegister(CC1101_MARCSTATE) & 0x1F;
            if (marcstate == CC1101_MARCSTATE_IDLE || marcstate == CC1101_MARCSTATE_RX_END) {
                const uint8_t finalBytes = readStatusRegister(CC1101_RXBYTES) & 0x7F;
                if (finalBytes > 0 && total + finalBytes <= length) {
                    if (!readBurst(CC1101_RXFIFO, buffer + total, finalBytes)) {
                        return false;
                    }
                    total += finalBytes;
                }
                break;
            }
        }

        if (millis() - lastProgress > timeoutMs) {
            return false;
        }

        delayMicroseconds(200);
    }

    return total == length;
}

bool WmbusApatorClass::decodeFrame(std::vector<uint8_t>& frame, int16_t rssiDbm, DecodedTelegram& telegram)
{
    if (frame.size() < 15) {
        return false;
    }

    const uint16_t manufacturer = (static_cast<uint16_t>(frame[3]) << 8) | frame[2];
    const uint8_t version = frame[8];
    const uint8_t media = frame[9];
    if (manufacturer != ManufacturerApa || version != 0x05 || (media != 0x06 && media != 0x07)) {
        return false;
    }

    const uint32_t meterId = static_cast<uint32_t>(frame[4])
        | (static_cast<uint32_t>(frame[5]) << 8)
        | (static_cast<uint32_t>(frame[6]) << 16)
        | (static_cast<uint32_t>(frame[7]) << 24);

    if (static_cast<uint32_t>(WMBUS_APATOR_METER_ID) != 0 && meterId != static_cast<uint32_t>(WMBUS_APATOR_METER_ID)) {
        return false;
    }

    size_t payloadOffset = 0;
    uint8_t accessNumber = 0;
    if (!extractPayload(frame, payloadOffset, accessNumber)) {
        return false;
    }

    std::vector<uint8_t> payload(frame.begin() + payloadOffset, frame.end());
    double totalM3 = 0;
    if (!decodeApatorPayload(payload, totalM3)) {
        return false;
    }

    telegram.meterId = meterId;
    telegram.totalM3 = totalM3;
    telegram.rssiDbm = rssiDbm;
    return true;
}

bool WmbusApatorClass::extractPayload(std::vector<uint8_t>& frame, size_t& payloadOffset, uint8_t& accessNumber)
{
    size_t pos = 10;
    if (pos >= frame.size()) {
        return false;
    }

    const uint8_t ci = frame[pos++];
    uint8_t tplNumEncryptedBlocks = 0;

    if (ci == 0x72) {
        if (pos + 8 > frame.size()) {
            return false;
        }
        pos += 8;
    } else if (ci != 0x7A) {
        return false;
    }

    if (pos + 4 > frame.size()) {
        return false;
    }

    accessNumber = frame[pos++];
    pos++; // STS
    const uint16_t cfg = static_cast<uint16_t>(frame[pos]) | (static_cast<uint16_t>(frame[pos + 1]) << 8);
    pos += 2;

    const uint8_t securityMode = (cfg >> 8) & 0x1F;
    if (securityMode == 5) {
        tplNumEncryptedBlocks = (cfg >> 4) & 0x0F;

        if (pos + 2 <= frame.size() && frame[pos] == 0x2F && frame[pos + 1] == 0x2F) {
            payloadOffset = pos + 2;
            return true;
        }

        if (!decryptAesCbcIvPayload(frame, pos, accessNumber, tplNumEncryptedBlocks)) {
            return false;
        }
        if (pos + 2 > frame.size() || frame[pos] != 0x2F || frame[pos + 1] != 0x2F) {
            ESP_LOGW(TAG, "AES-CBC-IV decrypt check bytes missing");
            return false;
        }
        payloadOffset = pos + 2;
        return true;
    }

    if (securityMode == 0) {
        payloadOffset = pos;
        return true;
    }

    ESP_LOGW(TAG, "Unsupported TPL security mode %u", securityMode);
    return false;
}

bool WmbusApatorClass::decryptAesCbcIvPayload(std::vector<uint8_t>& frame, size_t payloadOffset, uint8_t accessNumber, uint8_t tplNumEncryptedBlocks)
{
    std::vector<uint8_t> key;
    if (!configuredKey(key) || key.size() != 16) {
        ESP_LOGW(TAG, "Invalid or missing WMBUS_APATOR_KEY for encrypted telegram");
        return false;
    }

    const size_t encryptedAvailable = frame.size() - payloadOffset;
    size_t encryptedSize = tplNumEncryptedBlocks != 0 ? static_cast<size_t>(tplNumEncryptedBlocks) * 16 : encryptedAvailable;
    if (encryptedAvailable < encryptedSize) {
        encryptedSize = encryptedAvailable;
        if (encryptedSize < 16) {
            return false;
        }
    }
    if (encryptedSize % 16 != 0) {
        encryptedSize -= encryptedSize % 16;
        if (encryptedSize < 16) {
            return false;
        }
    }

    uint8_t iv[16] { };
    size_t ivPos = 0;
    iv[ivPos++] = frame[2];
    iv[ivPos++] = frame[3];
    for (size_t i = 4; i < 10; ++i) {
        iv[ivPos++] = frame[i];
    }
    for (uint8_t i = 0; i < 8; ++i) {
        iv[ivPos++] = accessNumber;
    }

    std::vector<uint8_t> decrypted(encryptedSize);
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    int rc = mbedtls_aes_setkey_dec(&aes, key.data(), 128);
    if (rc == 0) {
        rc = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, encryptedSize, iv, frame.data() + payloadOffset, decrypted.data());
    }
    mbedtls_aes_free(&aes);
    if (rc != 0) {
        ESP_LOGW(TAG, "AES decrypt failed: %d", rc);
        return false;
    }

    std::vector<uint8_t> out;
    out.reserve(payloadOffset + decrypted.size() + (encryptedAvailable - encryptedSize));
    out.insert(out.end(), frame.begin(), frame.begin() + payloadOffset);
    out.insert(out.end(), decrypted.begin(), decrypted.end());
    out.insert(out.end(), frame.begin() + payloadOffset + encryptedSize, frame.end());
    frame = std::move(out);
    return true;
}

bool WmbusApatorClass::decodeApatorPayload(const std::vector<uint8_t>& payload, double& totalM3) const
{
    if (payload.size() < 12) {
        return false;
    }

    size_t pos = 8;
    while (pos < payload.size()) {
        const uint8_t reg = payload[pos++];
        if (reg == 0xFF) {
            break;
        }

        const int size = apatorRegisterSize(reg);
        if (size < 0 || pos + static_cast<size_t>(size) > payload.size()) {
            return false;
        }

        if (reg == 0x10 && size == 4) {
            const uint32_t liters = static_cast<uint32_t>(payload[pos])
                | (static_cast<uint32_t>(payload[pos + 1]) << 8)
                | (static_cast<uint32_t>(payload[pos + 2]) << 16)
                | (static_cast<uint32_t>(payload[pos + 3]) << 24);
            totalM3 = static_cast<double>(liters) / 1000.0;
            return true;
        }

        pos += size;
    }

    return false;
}

void WmbusApatorClass::publishTelegram(const QueuedTelegram& telegram)
{
    if (!MqttSettings.getConnected()) {
        return;
    }

    MqttSettings.publishGeneric(TopicRssi, String(telegram.rssiDbm), true, 0);
    MqttSettings.publishGeneric(TopicTotal, String(static_cast<double>(telegram.totalMilliM3) / 1000.0, 3), true, 0);
    publishAvailability(true);

    ESP_LOGI(TAG, "Apator %08" PRIx32 ": %.3f m3 RSSI=%d dBm",
        telegram.meterId,
        static_cast<double>(telegram.totalMilliM3) / 1000.0,
        telegram.rssiDbm);
}

void WmbusApatorClass::publishAvailability(bool online)
{
    if (!MqttSettings.getConnected()) {
        return;
    }
    MqttSettings.publishGeneric(TopicAvailability, online ? "online" : "offline", true, 0);
    _availabilityOnline = online;
    _availabilityPublished = true;
}

void WmbusApatorClass::receiverTask(void* arg)
{
    static_cast<WmbusApatorClass*>(arg)->receiverLoop();
}

void WmbusApatorClass::receiverLoop()
{
    while (true) {
        if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(60000))) {
            restartRx();
            continue;
        }

        std::vector<uint8_t> frame;
        int16_t rssiDbm = 0;
        const bool ok = readFrame(frame, rssiDbm);
        restartRx();
        if (!ok) {
            continue;
        }

        DecodedTelegram decoded;
        if (!decodeFrame(frame, rssiDbm, decoded)) {
            continue;
        }

        QueuedTelegram queued {
            .meterId = decoded.meterId,
            .totalMilliM3 = static_cast<int32_t>(std::lround(decoded.totalM3 * 1000.0)),
            .rssiDbm = decoded.rssiDbm,
        };

        if (xQueueSend(_telegramQueue, &queued, 0) == pdTRUE) {
            _lastTelegramMillis = millis();
        }
    }
}

void IRAM_ATTR WmbusApatorClass::handleInterrupt(void* arg)
{
    auto* instance = static_cast<WmbusApatorClass*>(arg);
    if (instance->_receiverTaskHandle == nullptr) {
        return;
    }

    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(instance->_receiverTaskHandle, &higherPriorityTaskWoken);
    if (higherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void WmbusApatorClass::restartRx()
{
    strobe(CC1101_SIDLE);
    delay(1);
    strobe(CC1101_SFRX);
    strobe(CC1101_SRX);
    delay(1);
}

uint8_t WmbusApatorClass::strobe(uint8_t command)
{
    uint8_t rx = 0;
    spi_transaction_t trans {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .cmd = 0,
        .addr = 0,
        .length = 8,
        .rxlength = 8,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer = nullptr,
    };
    trans.tx_data[0] = command;
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &trans));
    rx = trans.rx_data[0];
    return rx;
}

uint8_t WmbusApatorClass::readRegister(uint8_t address)
{
    spi_transaction_t trans {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .cmd = 0,
        .addr = 0,
        .length = 16,
        .rxlength = 16,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer = nullptr,
    };
    trans.tx_data[0] = static_cast<uint8_t>(address | CC1101_READ_SINGLE);
    trans.tx_data[1] = 0x00;
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &trans));
    return trans.rx_data[1];
}

uint8_t WmbusApatorClass::readStatusRegister(uint8_t address)
{
    spi_transaction_t trans {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .cmd = 0,
        .addr = 0,
        .length = 16,
        .rxlength = 16,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer = nullptr,
    };
    trans.tx_data[0] = static_cast<uint8_t>(address | CC1101_READ_BURST);
    trans.tx_data[1] = 0x00;
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &trans));
    return trans.rx_data[1];
}

void WmbusApatorClass::writeRegister(uint8_t address, uint8_t value)
{
    spi_transaction_t trans {
        .flags = SPI_TRANS_USE_TXDATA,
        .cmd = 0,
        .addr = 0,
        .length = 16,
        .rxlength = 0,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer = nullptr,
    };
    trans.tx_data[0] = static_cast<uint8_t>(address | CC1101_WRITE_SINGLE);
    trans.tx_data[1] = value;
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &trans));
}

bool WmbusApatorClass::readBurst(uint8_t address, uint8_t* data, size_t length)
{
    if (length == 0) {
        return true;
    }
    if (_spiBurstTx == nullptr || _spiBurstRx == nullptr) {
        return false;
    }

    size_t offset = 0;
    while (offset < length) {
        const size_t chunk = std::min(Cc1101FifoBytes, length - offset);
        const size_t transferBytes = chunk + 1;
        const size_t safeBytes = ((transferBytes + SpiDmaAlignment - 1) / SpiDmaAlignment) * SpiDmaAlignment;

        memset(_spiBurstTx, 0, safeBytes);
        memset(_spiBurstRx, 0, safeBytes);
        _spiBurstTx[0] = address | CC1101_READ_BURST;

        spi_transaction_t trans {
            .flags = 0,
            .cmd = 0,
            .addr = 0,
            .length = static_cast<size_t>(transferBytes * 8),
            .rxlength = static_cast<size_t>(transferBytes * 8),
            .user = nullptr,
            .tx_buffer = _spiBurstTx,
            .rx_buffer = _spiBurstRx,
        };
        ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &trans));
        memcpy(data + offset, _spiBurstRx + 1, chunk);
        offset += chunk;
    }

    return true;
}

#endif
