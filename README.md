# Anyleaf

## For use with the AnyLeaf pH and RTD sensors in C++, or Arduino
[Homepage](https://anyleaf.org)

To install using Arduino IDE, go to `Sketch` menu → `Include Library` → `Manage Libraries` →
search for `anyleaf` → select and click `Install`.

### Install Depdendencies

* [Adafruit ADS1X15](https://github.com/adafruit/Adafruit_ADS1X15)
* [SimpleKalmanFilter](https://github.com/denyssene/SimpleKalmanFilter)
* [Adafruit MAX31865](https://github.com/adafruit/Adafruit_MAX31865)

### Example for Arduino:
```c++
#include <Anyleaf.h>

PhSensor ph_sensor;

void setup(void) {
    Serial.begin(9600);

    ph_sensor = PhSensor();

    // 2 or 3 pt calibration both give acceptable results.ca
    // Calibrate with known values. (voltage, pH, temp in °C).
    // You can find voltage and temperature with `ph_sensor.read_voltage()` and
    // `ph_sensor.read_temp()` respectively.
    // For 3 pt calibration, pass a third argument to `calibrate_all`.
    ph_sensor.calibrate_all(
        CalPt(0., 7., 25.), CalPt(0.17, 4., 25.)
    );

    // Or, call these with the sensor in the appropriate buffer solution.
    // This will automatically use voltage and temperature.
    // Voltage and Temp are returned, but calibration occurs
    // without using the return values.

    // 20. here is externally-sourced Temp, in °C.
    // (V, T) = ph_sensor.calibrate(CalSlot::One, 7., 20.)
    // ph_sensor.calibrate(CalSlot::Two, 4., temp_sensor.read());

    // Store the calibration parameters somewhere, so they persist
    // between program runs.

}

void loop(void) {
    double pH = ph_sensor.read();
    // To use an offboard temperature measurement: `double pH = ph_sensor.read(temp_sensor.read());
    Serial.print("pH: "); Serial.println(pH);

    delay(1000);
}
```

If the above code isn't suitable for your microcontroller, the key takeaway
 is that the module's digital output comes from
a [Texas Instruments ADS1115](http://www.ti.com/lit/ds/symlink/ads1114.pdf)
ADC, addressed on I²C port `0x48` or `0x49`. The pH or ORP voltage is mapped to a 
differential
from inputs `A0` and `A1`. The onboard temperature voltage is from input `A2`.
The `RTD` uses a `max31865` chip.