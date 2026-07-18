// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>

#ifdef OPENDTU_WMBUS_APATOR

#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <stdint.h>
#include <vector>

class WmbusApatorClass {
public:
    WmbusApatorClass();
    void init(Scheduler& scheduler);

private:
    struct DecodedTelegram {
        uint32_t meterId = 0;
        double totalM3 = 0;
        int16_t rssiDbm = 0;
    };

    struct QueuedTelegram {
        uint32_t meterId;
        int32_t totalMilliM3;
        int16_t rssiDbm;
    };

    void loop();
    bool setupRadio();
    bool configureCc1101();
    void restartRx();
    bool readFrame(std::vector<uint8_t>& frame, int16_t& rssiDbm);
    bool readBytes(uint8_t* buffer, size_t length, uint32_t timeoutMs);
    bool decodeFrame(std::vector<uint8_t>& frame, int16_t rssiDbm, DecodedTelegram& telegram);
    bool extractPayload(std::vector<uint8_t>& frame, size_t& payloadOffset, uint8_t& accessNumber);
    bool decryptAesCbcIvPayload(std::vector<uint8_t>& frame, size_t payloadOffset, uint8_t accessNumber, uint8_t tplNumEncryptedBlocks);
    bool decodeApatorPayload(const std::vector<uint8_t>& payload, double& totalM3) const;
    void publishTelegram(const QueuedTelegram& telegram);
    void publishAvailability(bool online);

    uint8_t strobe(uint8_t command);
    uint8_t readRegister(uint8_t address);
    uint8_t readStatusRegister(uint8_t address);
    void writeRegister(uint8_t address, uint8_t value);
    bool readBurst(uint8_t address, uint8_t* data, size_t length);

    static void receiverTask(void* arg);
    static void IRAM_ATTR handleInterrupt(void* arg);
    void receiverLoop();

    spi_device_handle_t _spi = nullptr;
    uint8_t* _spiBurstTx = nullptr;
    uint8_t* _spiBurstRx = nullptr;
    QueueHandle_t _telegramQueue = nullptr;
    TaskHandle_t _receiverTaskHandle = nullptr;
    Task _loopTask;
    uint32_t _lastTelegramMillis = 0;
    bool _availabilityOnline = false;
    bool _availabilityPublished = false;
    uint8_t _lastRssiRegister = 0;
};

extern WmbusApatorClass WmbusApator;

#endif
