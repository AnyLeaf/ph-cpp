#include <Anyleaf.h>

PhSensor phSensor;

void setup(void) {
    Serial.begin(9600);

    phSensor = PhSensor();

    // 2 or 3 pt calibration both give acceptable results.ca
    // Calibrate with known values. (voltage, pH, temp in °C).
    // You can find voltage and temperature with `ph_sensor.read_voltage()` and
    // `ph_sensor.read_temp()` respectively.
    // For 3 pt calibration, pass a third argument to `calibrate_all`.
    phSensor.calibrate_all(
        CalPt(0., 7., 25.), CalPt(0.17, 4., 25.)
    );

    // Or, call these with the sensor in the appropriate buffer solution.
    // This will automatically use voltage and temperature.
    // Voltage and Temp are returned, but calibration occurs
    // without using the return values.

    // 20. here is externally-sourced Temp, in °C.
    // (V, T) = ph_sensor.calibrate(CalSlot::One, 7., 20.)
    // ph_sensor.calibrate(CalSlot::Two, 4.)

    // Store the calibration parameters somewhere, so they persist
    // between program runs.

}

void loop(void) {
    double pH = phSensor.read();
    // To use an offboard temperature measurement: `double pH = ph_sensor.read(temp_sensor.read());
    Serial.print("pH: "); Serial.println(pH);

    delay(1000);
}