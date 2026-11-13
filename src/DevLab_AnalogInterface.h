/**
 * @file DevLab_AnalogInterface.h
 * @brief DevLab UE0177 Analog-Digital to I2C Sensor Interface driver over DDP/I2C.
 *
 * The node buffers the analog signal on SIGIN, reads it with the 12-bit ADC of
 * a PY32F003 (PA2) and exposes the count, a software threshold with
 * hysteresis and the optional PB5 GPIO line (JP2 pin 3). The U1A hardware
 * comparator is on the board (output OUT on J2) and is not read over I2C.
 *
 * begin() identifies the node and then configures PB5. The node forgets its
 * PB5 mode on every reset, so call begin() again whenever the master restarts.
 *
 * @author Jonathan Mejorado
 * @organization UNIT Electronics MX
 */

#ifndef DEVLAB_ANALOGINTERFACE_H
#define DEVLAB_ANALOGINTERFACE_H

#pragma once

#include "DevLabDDP.h"
#include "DevLab_I2C_Orchestrator.h"

/* PB5 configuration. The numeric values are the firmware mode numbers. */
enum class DevLab_AnalogGpioMode : uint8_t
{
    Input = 0,                   /* input with pull-up (reset default) */
    OutputLow = 1,               /* push-pull output, low */
    OutputHigh = 2,              /* push-pull output, high */
    FollowThreshold = 3,         /* output = above the threshold */
    FollowThresholdInverted = 4  /* output = below the threshold */
};

/* Result of DevLab_AnalogInterface::poll(). */
struct DevLab_AnalogInterface_State
{
    bool above = false;     /* signal above the software threshold */
    bool changed = false;   /* the state changed since the previous status read */
    bool adcError = false;  /* the last ADC conversion failed */
    bool gpioHigh = false;  /* PB5 reads high */
};

class DevLab_AnalogInterface
{
public:
    /* DDP Device ID of the UE0177 module (DDP_DEVICE_ID in firmware). */
    static constexpr uint16_t DEVICE_ID = 0x0119U;

    /* Factory I2C address (STARTUP_I2C_ADDRESS). Firmware 0.4 and newer can store
     * another one (see examples/i2c/changeAddress); pass it to the constructor. */
    static constexpr uint8_t DEFAULT_ADDRESS = 0x39U;

    /* The firmware runs its I2C slave at 100 kHz. */
    static constexpr uint32_t DEFAULT_CLOCK = 100000UL;

    /* 12-bit ADC full scale. */
    static constexpr uint16_t ADC_MAX = 4095U;

    /* Supply used by readMillivolts()/readVolts() until setSupplyMillivolts(). */
    static constexpr uint16_t DEFAULT_SUPPLY_MV = 3300U;

    /* Bits returned by readStatus() (command 0x83). */
    static constexpr uint8_t STATUS_ABOVE = 0x01U;
    static constexpr uint8_t STATUS_EDGE = 0x02U;
    static constexpr uint8_t STATUS_ADC_ERROR = 0x04U;
    static constexpr uint8_t STATUS_GPIO_HIGH = 0x08U;

    explicit DevLab_AnalogInterface(TwoWire &wire = Wire, uint8_t address = DEFAULT_ADDRESS, uint32_t clock = DEFAULT_CLOCK)
    : _bus(wire, clock), _ddp(_bus, DEVICE_ID), _address(address), _clock(clock) {}

    /* Start the bus, identify the node and configure PB5. Returns false when
     * the node is not found or the PB5 mode could not be set; isConnected()
     * tells the two cases apart. */
    bool begin(DevLab_AnalogGpioMode gpioMode = DevLab_AnalogGpioMode::Input);
    bool begin(uint8_t sdaPin, uint8_t sclPin, DevLab_AnalogGpioMode gpioMode = DevLab_AnalogGpioMode::Input);
    bool beginRecovered(uint8_t sdaPin, uint8_t sclPin,
                        DevLab_AnalogGpioMode gpioMode = DevLab_AnalogGpioMode::Input,
                        uint32_t timeoutUs = 20000, bool restart = false);

    /* ---- Analog input (PA2 / ADC_IN2) ---- */

    /* 12-bit ADC count, 0..4095 (command 0x81, same as standard ADC0 0x60).
     * The count is a fraction of the supply voltage. */
    bool readRaw(uint16_t &rawValue);

    /* Voltage at the input, scaled by the supply set with setSupplyMillivolts().
     * The scaling happens here, so a 5 V board needs no firmware change. */
    bool readMillivolts(uint16_t &millivolts);
    bool readVolts(float &volts);

    /* Count as a percentage of full scale. */
    bool readPercentage(float &percentage);

    /* Supply voltage of the module (VSYS) in millivolts, used for the mV/V
     * conversions. Default 3300. */
    void setSupplyMillivolts(uint16_t millivolts) { _supplyMv = millivolts; }
    uint16_t supplyMillivolts() const { return _supplyMv; }

    /* ---- Software threshold (hysteresis and debounce run in the node) ---- */

    /* true while the signal is above the threshold (command 0x80, same as the
     * standard GPIO0 command 0x40). */
    bool readAboveThreshold(bool &above);

    /* Count levels of the hysteresis: the state turns on at `high` and off at
     * `low` (command 0x84). Firmware default: 2300 and 1800. */
    bool readThresholds(uint16_t &high, uint16_t &low);

    /* Changes the levels, in 12-bit counts (0..4095, low < high). The signal
     * turns the state on at count >= high and off at count <= low. They are
     * kept in node RAM only: after a reset of the node they return to the
     * firmware defaults, so call this from setup() after begin(). Needs
     * firmware 0.3 or newer; returns false with older firmware or when the pair
     * is not valid, and the active levels are kept. */
    bool setThresholds(uint16_t high, uint16_t low);

    /* Same in millivolts at the pin, using setSupplyMillivolts() for the scale. */
    bool setThresholdsMillivolts(uint16_t highMv, uint16_t lowMv);

    /* Back to the firmware default levels. */
    bool resetThresholds();

    /* Reads the status byte (command 0x83). Reading CLEARS the edge event. */
    bool readStatus(uint8_t &status);

    /* One-call helper over readStatus(). */
    bool poll(DevLab_AnalogInterface_State &out);

    /* ---- PB5 GPIO (JP2 pin 3) ---- */

    /* Sets the PB5 mode. The mode is kept in node RAM only. */
    bool setGpioMode(DevLab_AnalogGpioMode mode);
    bool readGpioMode(DevLab_AnalogGpioMode &mode);

    /* Level of PB5 in any mode (command 0x41). */
    bool readGpio(bool &high);

    /* Shortcuts over setGpioMode(). */
    bool gpioInput() { return setGpioMode(DevLab_AnalogGpioMode::Input); }
    bool gpioWrite(bool high) { return setGpioMode(high ? DevLab_AnalogGpioMode::OutputHigh : DevLab_AnalogGpioMode::OutputLow); }
    bool gpioFollowThreshold(bool inverted = false) { return setGpioMode(inverted ? DevLab_AnalogGpioMode::FollowThresholdInverted : DevLab_AnalogGpioMode::FollowThreshold); }

    /* ---- Bus and device information ---- */

    bool busReady() const { return _busReady; }
    bool isConnected() const { return _verified; }
    uint8_t address() const { return _address; }
    const DevLabDDP::DeviceInfo &deviceInfo() const { return _info; }
    void printInfo(Print &out = Serial) const;

    DevLabDDP::Master &protocol() { return _ddp; }
    DevLab_I2C_Orchestrator &bus() { return _bus; }

private:
    bool sendCommand(uint8_t command);
    bool sendLevel(uint16_t level, uint8_t storeCommand);
    static constexpr uint8_t kCmdGetState = 0x80U;
    static constexpr uint8_t kCmdGetRaw = 0x81U;
    static constexpr uint8_t kCmdGetStatus = 0x83U;
    static constexpr uint8_t kCmdGetThresholds = 0x84U;
    static constexpr uint8_t kCmdThresholdStoreHigh = 0x8BU;
    static constexpr uint8_t kCmdThresholdStoreLow = 0x8CU;
    static constexpr uint8_t kCmdThresholdApply = 0x8DU;
    static constexpr uint8_t kCmdThresholdReset = 0x8EU;
    static constexpr uint8_t kCmdThresholdNibble = 0x90U; /* + 4 bits */
    static constexpr uint8_t kCmdGpioSetInput = 0x85U; /* + mode number for the other modes */
    static constexpr uint8_t kCmdGpioGetMode = 0x8AU;
    static constexpr uint8_t kCmdReadGpio1 = 0x41U;
    static constexpr uint8_t GPIO_MODE_COUNT = 5U;
    static constexpr uint16_t _kResponseDelayMs = 2U;

    bool finishBegin(DevLab_AnalogGpioMode gpioMode);
    bool readU16(uint8_t command, uint16_t &value);

    DevLab_I2C_Orchestrator _bus;
    DevLabDDP::Master _ddp;
    uint8_t _address;
    uint32_t _clock;
    uint16_t _supplyMv = DEFAULT_SUPPLY_MV;
    bool _busReady = false;
    bool _verified = false;
    DevLabDDP::DeviceInfo _info;
};

#endif
