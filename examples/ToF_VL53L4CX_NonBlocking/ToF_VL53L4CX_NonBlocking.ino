#include <Arduino.h>
#include <Wire.h>
#include <ToF_VL53L4CX.h>

// ESP32-WROOM common default I2C pins. Change to match your board.
#define I2C_SDA 21
#define I2C_SCL 22

// Use -1 if XSHUT is not connected.
#define TOF_XSHUT_PIN -1

ToF_VL53L4CX tof(Wire, TOF_XSHUT_PIN);

void setup()
{
    Serial.begin(115200);
    delay(1000);

#if defined(ESP32)
    Wire.begin(I2C_SDA, I2C_SCL);
#else
    Wire.begin();
#endif

    Wire.setClock(400000);

    Serial.println("Starting VL53L4CX non-blocking ToF wrapper...");

    if (!tof.begin()) {
        Serial.println("VL53L4CX init failed");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("VL53L4CX ready");
}

void loop()
{
    if (tof.update()) {
        const ToF_VL53L4CX_Reading& r = tof.reading();

        Serial.print("tof_mm=");
        Serial.print(r.distanceMm);
        Serial.print(", valid=");
        Serial.print(r.valid);
        Serial.print(", status=");
        Serial.print(r.rangeStatus);
        Serial.print(", objects=");
        Serial.print(r.objectCount);
        Serial.print(", signal=");
        Serial.print(r.signalMcps, 3);
        Serial.print(", ambient=");
        Serial.println(r.ambientMcps, 3);
    }

    // This is not blocking the sensor. It only controls serial print/update rate.
    delay(5);
}
