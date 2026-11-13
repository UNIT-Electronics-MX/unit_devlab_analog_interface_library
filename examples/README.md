# DevLab_AnalogInterface examples

| Example | Purpose |
|---|---|
| `adc/readAnalog` | Reads the sensor on SIGIN: ADC count, voltage, percentage and threshold state, in Serial Plotter format. |
| `adc/readVoltage` | Voltage with host-side averaging; set the 3.3 V or 5 V supply. |
| `adc/linearSensor` | Converts the voltage of a linear sensor (LM35 by default) to physical units with a gain and an offset. |
| `adc/readKY013` | Reads a KY-013 NTC temperature sensor: resistance from the divider, then Steinhart-Hart (or Beta) to degrees C and F. |
| `adc/readKY039` | Heartbeat monitor for a KY-039 finger sensor: removes the DC level, detects the pulse peaks and prints BPM, plus the 15 s beat count of the classic sketch. Has a Serial Plotter mode to tune it. |
| `adc/thresholdEvents` | Prints each crossing of the software threshold (hysteresis and debounce run in the node). |
| `gpio/readKY021` | KY-021 magnetic reed switch on PB5: debounced "magnet detected / removed" events and a counter. Digital input with the node's pull-up, no ADC needed. |
| `gpio/readKY027` | KY-027 mercury tilt switch on PB5: debounced "tilt detected / upright again" events and a counter. Digital input with the node's pull-up; the module LED is optional (master pin). |
| `gpio/laserKY008` | Switches a KY-008 laser module with PB5 (output, no ADC): serial commands for on, off and blink. Notes on the pin current limit and laser safety in the header. |
| `gpio/gpioInput` | PB5 as a digital input: prints every change. |
| `gpio/gpioOutput` | PB5 as a digital output: blinks an LED or drives an enable line. |
| `gpio/gpioFollowThreshold` | PB5 follows the analog threshold inside the node; `i`/`n` choose inverted or normal. |
| `gpio/gpioHostAlarm` | The sketch drives PB5 as an alarm with its own limits in millivolts. |
| `gpio/gpioModeConsole` | Serial console to try every PB5 mode and read the pin. |
| `i2c/i2c_scanner` | Lists every address on the bus and marks the UE0177. |
| `i2c/changeAddress` | Serial console to change the I2C address of the module (`change 39 40`) or restore the factory address (`factory 40`). Needs firmware 0.4. |

Device ID `0x0119`, factory I2C address `0x39`, I2C at 100 kHz. Firmware 0.4
and newer can change the address (`i2c/changeAddress`): it is stored in the
Flash of the module, so it survives a power cycle, and `factory` restores
`0x39`. The other examples use the factory address; if you moved the module,
replace `DevLab_AnalogInterface::DEFAULT_ADDRESS` with the new address in their
`DevLab_AnalogInterface analog(...)` line.

`begin()` always sends the PB5 mode, so each sketch starts from a known pin
state even if only the master was restarted. The signal enters through JP2
(`SIGIN`); PB5 is JP2 pin 3 (`GPIO`).

Pins: ESP32 SDA GPIO6 / SCL GPIO7; RP2040/RP2350 GPIO12/13 (GPIO24/25 on
Pulsar); STM32 and AVR use the board default pins. On RP2040/RP2350 the
examples pick `Wire` or `Wire1` from the SDA pin, because each GPIO belongs to
a fixed I2C controller (GPIO 24 and 12 are I2C0).
