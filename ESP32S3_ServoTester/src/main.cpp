#include <Arduino.h>
#include "servo_comm.h"

#define I2C_SDA_PIN 5
#define I2C_SCL_PIN 6

SmartServoComm servo;

void nak() {
    Serial.println("ERR NAK");
}

void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    // PING
    if (cmd == "PING") {
        Serial.println("PONG");
        return;
    }

    if (cmd == "PIN_STATUS") {
        Serial.printf("SDA(5)=%d SCL(6)=%d\n", digitalRead(I2C_SDA_PIN), digitalRead(I2C_SCL_PIN));
        return;
    }

    if (cmd == "PROBE") {
        void* busHandle = i2cBusHandle(0);
        Serial.printf("BusHandle=%p\n", busHandle);
        if (!busHandle) {
            Serial.println("NO_BUS");
            return;
        }
        for (uint16_t addr = 1; addr <= 126; addr++) {
            esp_err_t err = i2c_master_probe((i2c_master_bus_handle_t)busHandle, addr, 20);
            if (err == ESP_OK) {
                Serial.printf("FOUND 0x%02X (%d)\n", addr, addr);
            }
        }
        Serial.println("PROBE_DONE");
        return;
    }

    // SCAN
    if (cmd == "SCAN") {
        auto devs = servo.scanBus();
        Serial.print("DEVICES ");
        for (size_t i = 0; i < devs.size(); i++) {
            Serial.print(devs[i]);
            if (i < devs.size() - 1) Serial.print(",");
        }
        Serial.println();
        return;
    }

    // Commands with arguments: OPCODE ID [ARG]
    int firstSpace = cmd.indexOf(' ');
    if (firstSpace == -1) {
        Serial.println("ERR UNKNOWN_CMD");
        return;
    }

    String op = cmd.substring(0, firstSpace);
    op.toUpperCase();
    String rest = cmd.substring(firstSpace + 1);
    rest.trim();

    int secondSpace = rest.indexOf(' ');
    uint8_t id = 0;
    String argStr = "";
    if (secondSpace == -1) {
        id = (uint8_t)rest.toInt();
    } else {
        id = (uint8_t)rest.substring(0, secondSpace).toInt();
        argStr = rest.substring(secondSpace + 1);
        argStr.trim();
    }

    float floatArg = argStr.toFloat();
    float retVal = 0.0f;

    if (op == "ANGLE") {
        if (servo.setAngle(id, floatArg, retVal)) {
            Serial.printf("OK %.2f\n", retVal);
        } else {
            nak();
        }
    } else if (op == "VEL") {
        if (servo.setVelocity(id, floatArg, retVal)) {
            Serial.printf("OK %.2f\n", retVal);
        } else {
            nak();
        }
    } else if (op == "TORQUE") {
        if (servo.setTorque(id, floatArg, retVal)) {
            Serial.printf("OK %.2f\n", retVal);
        } else {
            nak();
        }
    } else if (op == "GET_ANGLE") {
        if (servo.getAngle(id, retVal)) {
            Serial.printf("OK %.2f\n", retVal);
        } else {
            nak();
        }
    } else if (op == "GET_VEL") {
        if (servo.getVelocity(id, retVal)) {
            Serial.printf("OK %.2f\n", retVal);
        } else {
            nak();
        }
    } else if (op == "ENABLE") {
        bool en = argStr.toInt() > 0;
        if (servo.setEnable(id, en, retVal)) {
            Serial.printf("OK %.2f\n", retVal);
        } else {
            nak();
        }
    } else if (op == "SET_ID") {
        uint8_t newId = (uint8_t)argStr.toInt();
        if (servo.setNodeId(id, newId, retVal)) {
            Serial.printf("OK %d\n", newId);
        } else {
            Serial.println("ERR NAK");
        }
    } else if (op == "SET_KP") {
        if (servo.setKp(id, floatArg)) Serial.println("OK");
        else Serial.println("ERR NAK");
    } else if (op == "GET_KP") {
        if (servo.getKp(id, retVal)) Serial.printf("OK %.2f\n", retVal);
        else Serial.println("ERR NAK");
    } else if (op == "SET_KI") {
        if (servo.setKi(id, floatArg)) Serial.println("OK");
        else Serial.println("ERR NAK");
    } else if (op == "GET_KI") {
        if (servo.getKi(id, retVal)) Serial.printf("OK %.2f\n", retVal);
        else Serial.println("ERR NAK");
    } else if (op == "SET_KV") {
        if (servo.setKv(id, floatArg)) Serial.println("OK");
        else Serial.println("ERR NAK");
    } else if (op == "GET_KV") {
        if (servo.getKv(id, retVal)) Serial.printf("OK %.2f\n", retVal);
        else Serial.println("ERR NAK");
    } else if (op == "SET_KD") {
        if (servo.setKd(id, floatArg)) Serial.println("OK");
        else Serial.println("ERR NAK");
    } else if (op == "GET_KD") {
        if (servo.getKd(id, retVal)) Serial.printf("OK %.2f\n", retVal);
        else Serial.println("ERR NAK");
    } else if (op == "SET_TORQUE_LIMIT") {
        if (servo.setTorqueLimit(id, floatArg)) Serial.println("OK");
        else Serial.println("ERR NAK");
    } else if (op == "SET_INIT_POS") {
        if (servo.setInitPos(id, floatArg)) Serial.println("OK");
        else Serial.println("ERR NAK");
    } else if (op == "SET_INVERT") {
        bool inv = (floatArg > 0.5f || argStr.toInt() > 0);
        if (servo.setInvert(id, inv, retVal)) {
            Serial.printf("OK %d\n", inv ? 1 : 0);
        } else {
            Serial.println("ERR NAK");
        }
    } else if (op == "GET_INVERT") {
        if (servo.getInvert(id, retVal)) {
            Serial.printf("OK %d\n", (int)retVal);
        } else {
            Serial.println("ERR NAK");
        }
    } else if (op == "SET_MAX_ANGLE") {
        if (servo.setMaxAngle(id, floatArg, retVal)) {
            Serial.printf("OK %.1f\n", floatArg);
        } else {
            Serial.println("ERR NAK");
        }
    } else if (op == "GET_MAX_ANGLE") {
        if (servo.getMaxAngle(id, retVal)) {
            Serial.printf("OK %.1f\n", retVal);
        } else {
            Serial.println("ERR NAK");
        }
    } else if (op == "GET_RAW_ADC") {
        if (servo.getRawAdc(id, retVal)) {
            Serial.printf("OK %.0f\n", retVal);
        } else {
            Serial.println("ERR NAK");
        }
    } else if (op == "SET_ADC_MIN") {
        if (servo.setAdcMin(id, floatArg)) Serial.println("OK");
        else Serial.println("ERR NAK");
    } else if (op == "SET_ADC_MAX") {
        if (servo.setAdcMax(id, floatArg)) Serial.println("OK");
        else Serial.println("ERR NAK");
    } else {
        Serial.println("ERR INVALID_OP");
    }
}

void setup() {
    Serial.begin(115200);
    // Wire clock frequency: 100kHz standard mode for maximum reliability over jumper cables
    servo.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
}

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        processCommand(line);
    }
}
