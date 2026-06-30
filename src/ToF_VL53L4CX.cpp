#include "ToF_VL53L4CX.h"

ToF_VL53L4CX::ToF_VL53L4CX(TwoWire& wire, int xshutPin)
    : _wire(&wire),
      _xshutPin(xshutPin),
      _sensor(&wire, xshutPin)
{
}

bool ToF_VL53L4CX::begin(uint8_t stAddress,
                         DistanceMode distanceMode,
                         uint32_t timingBudgetUs,
                         uint32_t interMeasurementMs)
{
    _reading = ToF_VL53L4CX_Reading();
    _initialized = false;

    if (_wire == nullptr) {
        return false;
    }

    _sensor.begin();

    // Match the STM32duino example sequence: Off -> InitSensor(address) -> StartMeasurement.
    _sensor.VL53L4CX_Off();

    int status = _sensor.InitSensor(stAddress);
    if (status != 0) {
        return false;
    }

    VL53L4CX_DistanceModes stDistanceMode = VL53L4CX_DISTANCEMODE_MEDIUM;

    switch (distanceMode) {
        case DISTANCE_SHORT:
            stDistanceMode = VL53L4CX_DISTANCEMODE_SHORT;
            break;

        case DISTANCE_LONG:
            stDistanceMode = VL53L4CX_DISTANCEMODE_LONG;
            break;

        case DISTANCE_MEDIUM:
        default:
            stDistanceMode = VL53L4CX_DISTANCEMODE_MEDIUM;
            break;
    }

    status = _sensor.VL53L4CX_SetDistanceMode(stDistanceMode);
    if (status != 0) {
        return false;
    }

    status = _sensor.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(timingBudgetUs);
    if (status != 0) {
        return false;
    }

    status = _sensor.VL53L4CX_SetInterMeasurementPeriodMilliSeconds(interMeasurementMs);
    if (status != 0) {
        return false;
    }

    status = _sensor.VL53L4CX_StartMeasurement();
    if (status != 0) {
        return false;
    }

    _initialized = true;
    return true;
}

bool ToF_VL53L4CX::update()
{
    _reading.newData = false;

    if (!_initialized) {
        _reading.valid = false;
        return false;
    }

    uint8_t newDataReady = 0;
    int status = _sensor.VL53L4CX_GetMeasurementDataReady(&newDataReady);

    if (status != 0 || newDataReady == 0) {
        return false;
    }

    VL53L4CX_MultiRangingData_t rangingData;
    status = _sensor.VL53L4CX_GetMultiRangingData(&rangingData);

    if (status != 0) {
        _reading.valid = false;
        _sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
        return false;
    }

    _reading.streamCount = rangingData.StreamCount;
    _reading.objectCount = rangingData.NumberOfObjectsFound;

    if (_reading.objectCount > 0) {
        const auto& target = rangingData.RangeData[0];

        _reading.distanceMm = target.RangeMilliMeter;
        _reading.rangeStatus = target.RangeStatus;
        _reading.signalMcps = (float)target.SignalRateRtnMegaCps / 65536.0f;
        _reading.ambientMcps = (float)target.AmbientRateRtnMegaCps / 65536.0f;
        _reading.valid = isGoodRangeStatus(_reading.rangeStatus);
        _reading.newData = true;
        _reading.lastUpdateMs = millis();
    } else {
        _reading.valid = false;
    }

    _sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
    return _reading.newData;
}

void ToF_VL53L4CX::stop()
{
    if (_initialized) {
        _sensor.VL53L4CX_StopMeasurement();
    }
    _initialized = false;
}

void ToF_VL53L4CX::powerOff()
{
    stop();
    _sensor.VL53L4CX_Off();
}

void ToF_VL53L4CX::powerOn()
{
    _sensor.VL53L4CX_On();
}

bool ToF_VL53L4CX::isGoodRangeStatus(uint8_t status)
{
    // In the ST examples, RangeStatus is printed directly. For altitude assist,
    // use only status 0 as the clean/valid measurement.
    return status == 0;
}
