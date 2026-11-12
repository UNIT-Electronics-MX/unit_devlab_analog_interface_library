/**
 * @file DevLab_AnalogInterface.cpp
 * @brief DevLab UE0177 Analog-Digital to I2C Sensor Interface driver over DDP/I2C.
 *
 * @author Jonathan Mejorado
 * @organization UNIT Electronics MX
 */

#include "DevLab_AnalogInterface.h"

bool DevLab_AnalogInterface::finishBegin(DevLab_AnalogGpioMode gpioMode) {
    _verified = _ddp.matchesExpectedDevice(_address, &_info);
    if (!_verified) return false;
    /* Always send the mode: the node keeps PB5 in its last mode when only the
     * master restarts, and the master must not assume the reset default. */
    return setGpioMode(gpioMode);
}

bool DevLab_AnalogInterface::begin(DevLab_AnalogGpioMode gpioMode) {
    _verified = false;
    _bus.setClock(_clock);
    _busReady = _bus.begin();
    if (!_busReady) return false;
    return finishBegin(gpioMode);
}

bool DevLab_AnalogInterface::begin(uint8_t sdaPin, uint8_t sclPin, DevLab_AnalogGpioMode gpioMode) {
    _verified = false;
    _bus.setClock(_clock);
    _busReady = _bus.begin(sdaPin, sclPin);
    if (!_busReady) return false;
    return finishBegin(gpioMode);
}

bool DevLab_AnalogInterface::beginRecovered(uint8_t sdaPin, uint8_t sclPin, DevLab_AnalogGpioMode gpioMode,
                                            uint32_t timeoutUs, bool restart) {
    _verified = false;
    _bus.setClock(_clock);
    _busReady = _bus.beginRecovered(sdaPin, sclPin, timeoutUs, restart);
    if (!_busReady) return false;
    return finishBegin(gpioMode);
}

void DevLab_AnalogInterface::printInfo(Print &out) const {
    DevLabDDP::printDeviceInfo(out, _address, _info, _ddp.expectedDeviceId());
}

bool DevLab_AnalogInterface::readU16(uint8_t command, uint16_t &value) {
    if (!_verified) return false;
    uint8_t bytes[2];
    if (!_ddp.readCommand(_address, command, bytes, 2U, _kResponseDelayMs)) return false;
    value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    return true;
}

bool DevLab_AnalogInterface::readRaw(uint16_t &rawValue) {
    uint16_t value;
    if (!readU16(kCmdGetRaw, value)) return false;
    rawValue = value > ADC_MAX ? ADC_MAX : value;
    return true;
}

bool DevLab_AnalogInterface::readMillivolts(uint16_t &millivolts) {
    uint16_t raw;
    if (!readRaw(raw)) return false;
    millivolts = (uint16_t)(((uint32_t)raw * _supplyMv + ADC_MAX / 2U) / ADC_MAX);
    return true;
}

bool DevLab_AnalogInterface::readVolts(float &volts) {
    uint16_t millivolts;
    if (!readMillivolts(millivolts)) return false;
    volts = millivolts / 1000.0f;
    return true;
}

bool DevLab_AnalogInterface::readPercentage(float &percentage) {
    uint16_t raw;
    if (!readRaw(raw)) return false;
    percentage = (raw * 100.0f) / (float)ADC_MAX;
    return true;
}

bool DevLab_AnalogInterface::readAboveThreshold(bool &above) {
    if (!_verified) return false;
    uint8_t raw = 0U;
    if (!_ddp.readCommand(_address, kCmdGetState, &raw, 1U, _kResponseDelayMs)) return false;
    above = (raw & 0x01U) != 0U;
    return true;
}

bool DevLab_AnalogInterface::readThresholds(uint16_t &high, uint16_t &low) {
    if (!_verified) return false;
    uint8_t bytes[4];
    if (!_ddp.readCommand(_address, kCmdGetThresholds, bytes, 4U, _kResponseDelayMs)) return false;
    high = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    low = (uint16_t)bytes[2] | ((uint16_t)bytes[3] << 8);
    return true;
}

bool DevLab_AnalogInterface::sendCommand(uint8_t command) {
    uint8_t response = 0xFFU;
    if (!_ddp.readCommand(_address, command, &response, 1U, _kResponseDelayMs)) return false;
    /* Firmware older than 0.3 answers DDP_STATUS_UNKNOWN_CMD to these commands. */
    return response == DDP_STATUS_OK;
}

/* The firmware takes one command byte per transaction, so a 12-bit level goes
 * out as three nibble commands (most significant first) and a store command. */
bool DevLab_AnalogInterface::sendLevel(uint16_t level, uint8_t storeCommand) {
    for (int8_t shift = 8; shift >= 0; shift -= 4) {
        if (!sendCommand((uint8_t)(kCmdThresholdNibble + ((level >> shift) & 0x0FU)))) return false;
    }
    return sendCommand(storeCommand);
}

bool DevLab_AnalogInterface::setThresholds(uint16_t high, uint16_t low) {
    if (!_verified) return false;
    if (high > ADC_MAX || low >= high) return false;
    return sendLevel(high, kCmdThresholdStoreHigh) &&
           sendLevel(low, kCmdThresholdStoreLow) &&
           sendCommand(kCmdThresholdApply);
}

bool DevLab_AnalogInterface::setThresholdsMillivolts(uint16_t highMv, uint16_t lowMv) {
    if (_supplyMv == 0U || highMv > _supplyMv || lowMv >= highMv) return false;
    uint16_t high = (uint16_t)(((uint32_t)highMv * ADC_MAX + _supplyMv / 2U) / _supplyMv);
    uint16_t low = (uint16_t)(((uint32_t)lowMv * ADC_MAX + _supplyMv / 2U) / _supplyMv);
    return setThresholds(high, low);
}

bool DevLab_AnalogInterface::resetThresholds() {
    if (!_verified) return false;
    return sendCommand(kCmdThresholdReset);
}

bool DevLab_AnalogInterface::readStatus(uint8_t &status) {
    if (!_verified) return false;
    return _ddp.readCommand(_address, kCmdGetStatus, &status, 1U, _kResponseDelayMs);
}

bool DevLab_AnalogInterface::poll(DevLab_AnalogInterface_State &out) {
    out = DevLab_AnalogInterface_State();
    uint8_t status = 0U;
    if (!readStatus(status)) return false;
    out.above = (status & STATUS_ABOVE) != 0U;
    out.changed = (status & STATUS_EDGE) != 0U;
    out.adcError = (status & STATUS_ADC_ERROR) != 0U;
    out.gpioHigh = (status & STATUS_GPIO_HIGH) != 0U;
    return true;
}

bool DevLab_AnalogInterface::setGpioMode(DevLab_AnalogGpioMode mode) {
    if (!_verified) return false;
    uint8_t number = (uint8_t)mode;
    if (number >= GPIO_MODE_COUNT) return false;
    uint8_t response = 0xFFU;
    if (!_ddp.readCommand(_address, (uint8_t)(kCmdGpioSetInput + number), &response, 1U, _kResponseDelayMs)) return false;
    /* Firmware older than 0.2 answers DDP_STATUS_UNKNOWN_CMD here. */
    return response == DDP_STATUS_OK;
}

bool DevLab_AnalogInterface::readGpioMode(DevLab_AnalogGpioMode &mode) {
    if (!_verified) return false;
    uint8_t number = 0U;
    if (!_ddp.readCommand(_address, kCmdGpioGetMode, &number, 1U, _kResponseDelayMs)) return false;
    if (number >= GPIO_MODE_COUNT) return false;
    mode = (DevLab_AnalogGpioMode)number;
    return true;
}

bool DevLab_AnalogInterface::readGpio(bool &high) {
    if (!_verified) return false;
    uint8_t raw = 0U;
    if (!_ddp.readCommand(_address, kCmdReadGpio1, &raw, 1U, _kResponseDelayMs)) return false;
    high = (raw & 0x01U) != 0U;
    return true;
}
