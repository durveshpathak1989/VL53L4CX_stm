#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>

/**
 * Lightweight, non-blocking VL53L4CX wrapper for flight-control projects.
 *
 * This class intentionally keeps the heavy ST VL53L4CX driver hidden behind
 * a small interface. Call update() from a slow sensor task, then read the
 * latest cached value from the control loop.
 */
struct ToF_VL53L4CX_Reading {
    bool valid = false;
    bool newData = false;

    uint16_t distanceMm = 0;
    uint8_t rangeStatus = 255;
    uint8_t objectCount = 0;
    uint8_t streamCount = 0;

    float signalMcps = 0.0f;
    float ambientMcps = 0.0f;

    uint32_t lastUpdateMs = 0;
};

class ToF_VL53L4CX {
public:
    enum DistanceMode : uint8_t {
        DISTANCE_SHORT = 1,
        DISTANCE_MEDIUM = 2,
        DISTANCE_LONG = 3
    };

    ToF_VL53L4CX(TwoWire& wire = Wire, int xshutPin = -1);

    /**
     * Start the sensor.
     *
     * stAddress is intentionally the ST-driver style address used by
     * InitSensor(). The STM32duino examples use 0x12.
     */
    bool begin(uint8_t stAddress = 0x12,
               DistanceMode distanceMode = DISTANCE_MEDIUM,
               uint32_t timingBudgetUs = 20000,
               uint32_t interMeasurementMs = 25);

    /**
     * Non-blocking update.
     * Returns true only when a fresh measurement was copied into _reading.
     */
    bool update();

    bool isReady() const { return _initialized; }
    bool isValid() const { return _reading.valid; }
    bool hasNewData() const { return _reading.newData; }

    uint16_t distanceMm() const { return _reading.distanceMm; }
    float distanceM() const { return _reading.distanceMm * 0.001f; }
    uint32_t ageMs() const { return millis() - _reading.lastUpdateMs; }

    const ToF_VL53L4CX_Reading& reading() const { return _reading; }

    void stop();
    void powerOff();
    void powerOn();

private:
    TwoWire* _wire;
    int _xshutPin;
    VL53L4CX _sensor;
    ToF_VL53L4CX_Reading _reading;
    bool _initialized = false;

    static bool isGoodRangeStatus(uint8_t status);
};
