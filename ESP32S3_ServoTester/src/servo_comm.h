#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <driver/i2c_master.h>
#include "esp32-hal-i2c.h"

// I2C Opcodes matching ElectronBot ServoDrive-fw / ServoDrive-fw-ll
#define CMD_SET_ANGLE        0x01
#define CMD_SET_VELOCITY     0x02
#define CMD_SET_TORQUE       0x03
#define CMD_GET_ANGLE        0x11
#define CMD_GET_VELOCITY     0x12
#define CMD_GET_RAW_ADC      0x13
#define CMD_SET_NODE_ID      0x21
#define CMD_SET_KP           0x22
#define CMD_SET_KI           0x23
#define CMD_SET_KV           0x24
#define CMD_SET_KD           0x25
#define CMD_SET_TORQUE_LIMIT 0x26
#define CMD_SET_INIT_POS     0x27
#define CMD_SET_ADC_MIN      0x28
#define CMD_SET_ADC_MAX      0x29
#define CMD_SET_MAX_ANGLE    0x2A
#define CMD_SET_INVERT       0x30
#define CMD_GET_INVERT       0x14
#define CMD_GET_MAX_ANGLE    0x15
#define CMD_GET_KP           0x16
#define CMD_GET_KI           0x17
#define CMD_GET_KV           0x18
#define CMD_GET_KD           0x19
#define CMD_SET_ENABLE       0xFF

#define DEFAULT_XIAO_SDA     5
#define DEFAULT_XIAO_SCL     6

class SmartServoComm {
public:
    bool begin(int sda = DEFAULT_XIAO_SDA, int scl = DEFAULT_XIAO_SCL, uint32_t freq = 100000) {
        Wire.setTimeOut(30);
        return Wire.begin(sda, scl, freq);
    }

    // Standard 5-byte TX -> 5-byte RX transaction with glitch-tolerant retry
    uint8_t lastErr = 0;
    size_t lastN = 0;
    uint8_t lastRx0 = 0;

    void recoverBus(uint8_t devAddr) {
        // Step 1: Send quick 0-byte frame with short timeout to address 
        // to reset slave DMA and state machine
        Wire.setTimeOut(3);
        Wire.beginTransmission(devAddr);
        Wire.endTransmission(true);
        Wire.setTimeOut(30);

        // Step 2: If SDA or SCL is stuck LOW, perform 9-clock hardware unwedge
        if (digitalRead(DEFAULT_XIAO_SDA) == LOW || digitalRead(DEFAULT_XIAO_SCL) == LOW) {
            Wire.end();
            pinMode(DEFAULT_XIAO_SCL, OUTPUT_OPEN_DRAIN);
            pinMode(DEFAULT_XIAO_SDA, INPUT_PULLUP);
            digitalWrite(DEFAULT_XIAO_SCL, HIGH);
            delayMicroseconds(5);

            for (int i = 0; i < 9; i++) {
                if (digitalRead(DEFAULT_XIAO_SDA) == HIGH) {
                    break;
                }
                digitalWrite(DEFAULT_XIAO_SCL, LOW);
                delayMicroseconds(5);
                digitalWrite(DEFAULT_XIAO_SCL, HIGH);
                delayMicroseconds(5);
            }

            // Generate STOP condition
            pinMode(DEFAULT_XIAO_SDA, OUTPUT_OPEN_DRAIN);
            digitalWrite(DEFAULT_XIAO_SDA, LOW);
            delayMicroseconds(5);
            digitalWrite(DEFAULT_XIAO_SCL, HIGH);
            delayMicroseconds(5);
            digitalWrite(DEFAULT_XIAO_SDA, HIGH);
            delayMicroseconds(5);

            // Re-initialize Wire peripheral
            Wire.begin(DEFAULT_XIAO_SDA, DEFAULT_XIAO_SCL, 100000);
            Wire.setTimeOut(30);
        }
    }

    bool transfer(uint8_t devAddr, uint8_t opcode, float val, float &outVal) {
        uint8_t txBuf[5];
        txBuf[0] = opcode;
        memcpy(txBuf + 1, &val, 4);

        for (int attempt = 0; attempt < 3; attempt++) {
            if (attempt > 0) {
                recoverBus(devAddr);
                delay(2);
            }

            Wire.beginTransmission(devAddr);
            Wire.write(txBuf, 5);
            uint8_t err = Wire.endTransmission(true);
            lastErr = err;
            if (err != 0) {
                continue;
            }

            // 800us ensures STM32 DMA callback and any TIM14 PID interrupt complete
            delayMicroseconds(800);

            size_t n = Wire.requestFrom((uint8_t)devAddr, (size_t)5);
            lastN = n;
            if (n != 5) {
                continue;
            }

            uint8_t rxBuf[5];
            Wire.readBytes(rxBuf, 5);
            lastRx0 = rxBuf[0];

            // Sanity validation: response must echo the requested opcode
            if (rxBuf[0] != opcode) {
                continue;
            }

            memcpy(&outVal, rxBuf + 1, 4);
            return true;
        }
        recoverBus(devAddr);
        return false;
    }

    bool setAngle(uint8_t id, float angle, float &actualAngle) {
        return transfer(id, CMD_SET_ANGLE, angle, actualAngle);
    }

    bool setVelocity(uint8_t id, float vel, float &actualVel) {
        return transfer(id, CMD_SET_VELOCITY, vel, actualVel);
    }

    bool setTorque(uint8_t id, float torque, float &actualAngle) {
        return transfer(id, CMD_SET_TORQUE, torque, actualAngle);
    }

    bool getAngle(uint8_t id, float &actualAngle) {
        return transfer(id, CMD_GET_ANGLE, 0.0f, actualAngle);
    }

    bool getVelocity(uint8_t id, float &actualVel) {
        return transfer(id, CMD_GET_VELOCITY, 0.0f, actualVel);
    }

    bool setEnable(uint8_t id, bool enable, float &actualAngle) {
        return transfer(id, CMD_SET_ENABLE, enable ? 1.0f : 0.0f, actualAngle);
    }

    bool setNodeId(uint8_t currentId, uint8_t newId, float &actualAngle) {
        return transfer(currentId, CMD_SET_NODE_ID, (float)newId, actualAngle);
    }

    bool setKp(uint8_t id, float kp) { float d; return transfer(id, CMD_SET_KP, kp, d); }
    bool setKi(uint8_t id, float ki) { float d; return transfer(id, CMD_SET_KI, ki, d); }
    bool setKv(uint8_t id, float kv) { float d; return transfer(id, CMD_SET_KV, kv, d); }
    bool setKd(uint8_t id, float kd) { float d; return transfer(id, CMD_SET_KD, kd, d); }
    bool getKp(uint8_t id, float &kp) { return transfer(id, CMD_GET_KP, 0.0f, kp); }
    bool getKi(uint8_t id, float &ki) { return transfer(id, CMD_GET_KI, 0.0f, ki); }
    bool getKv(uint8_t id, float &kv) { return transfer(id, CMD_GET_KV, 0.0f, kv); }
    bool getKd(uint8_t id, float &kd) { return transfer(id, CMD_GET_KD, 0.0f, kd); }
    bool setTorqueLimit(uint8_t id, float limit) { float d; return transfer(id, CMD_SET_TORQUE_LIMIT, limit, d); }
    bool setInitPos(uint8_t id, float pos) { float d; return transfer(id, CMD_SET_INIT_POS, pos, d); }
    bool setInvert(uint8_t id, bool invert, float &actualAngle) { return transfer(id, CMD_SET_INVERT, invert ? 1.0f : 0.0f, actualAngle); }
    bool getInvert(uint8_t id, float &actualInvert) { return transfer(id, CMD_GET_INVERT, 0.0f, actualInvert); }
    bool setMaxAngle(uint8_t id, float maxAngle, float &actualAngle) { return transfer(id, CMD_SET_MAX_ANGLE, maxAngle, actualAngle); }
    bool getMaxAngle(uint8_t id, float &maxAngle) { return transfer(id, CMD_GET_MAX_ANGLE, 0.0f, maxAngle); }
    bool getRawAdc(uint8_t id, float &rawAdc) { return transfer(id, CMD_GET_RAW_ADC, 0.0f, rawAdc); }
    bool setAdcMin(uint8_t id, float minAdc) { float d; return transfer(id, CMD_SET_ADC_MIN, minAdc, d); }
    bool setAdcMax(uint8_t id, float maxAdc) { float d; return transfer(id, CMD_SET_ADC_MAX, maxAdc, d); }

    std::vector<uint8_t> scanBus() {
        std::vector<uint8_t> found;
        Wire.setTimeOut(3);
        for (uint8_t addr = 1; addr <= 126; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                found.push_back(addr);
            }
        }
        Wire.setTimeOut(30);
        return found;
    }

    bool rawTransfer(uint8_t id, const uint8_t* tx, uint8_t* rx) {
        Wire.beginTransmission(id);
        Wire.write(tx, 5);
        if (Wire.endTransmission() != 0) return false;
        if (Wire.requestFrom(id, (size_t)5) != 5) return false;
        Wire.readBytes(rx, 5);
        return true;
    }
};
