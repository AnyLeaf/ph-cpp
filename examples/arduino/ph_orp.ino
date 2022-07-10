#include <Anyleaf.h>

PhSensor phSensor;
// OrpSensor orpSensor;

void setup(void) {
    Serial.begin(9600);

    phSensor = PhSensor();

    // If you connect multiple AnyLeaf modules on the same I²C bus, set one's
    // jumper to the `0x49` position, and specify this as below:
    // PhSensor phSensor = PhSensor::new_alt_addr();

    // If you're using an ORP sensor:
    // orpSensor = OrpSensor();
    // Or:
    // OrpSensor orpSensor = OrpSensor::new_alt_addr();

    // 2 or 3 point calibration both give acceptable results.
    // Calibrate with known values. (voltage, pH, temp in °C).
    // You can find voltage and temperature with `ph_sensor.read_voltage()` and
    // `ph_sensor.read_temp()` respectively.
    // For 3 point calibration, pass a third argument to `calibrate_all`.

    // `calibrate_all` stores 2 or 3 calibration values, from known previous values.
    // The first value is measured voltage; the second is nominal pH; the third is temperature
    // at which the calibration was performed.
    phSensor.calibrate_all(
        CalPt(0., 7., 25.), CalPt(0.17, 4., 25.)
    );

    // Or, call `calibrate` with the sensor in the appropriate buffer solution.
    // This will automatically use voltage and temperature.
    // Voltage and Temp are returned, but calibration occurs
    // without using the return values:

    // ph_sensor.calibrate(CalSlot::One, 7.);
    // ph_sensor.calibrate(CalSlot::Two, 4.);

    // (V1, T1) = ph_sensor.calibrate(CalSlot::One, 7.)
    // (V2, T2) = ph_sensor.calibrate(CalSlot::Two, 4.)

    // ORP setup is simpler: There's only 1 calibration point, and no
    // temperature compensation. Use these as equivalents to the above:
    // orpSensor.calibrate_all(CalPtOrp(0.4, 400.0))
    // V = orp_sensor.calibrate(400.)

    // Ideally, store the calibration parameters somewhere, so they persist
    // between program runs. You could store them to nonvolatile memory,
    // or just replace the `ph_sensor.calibrate_all`(
    // line above with the new calibration values. (V1, T1, V2, and T2).
}

void loop(void) {
    double pH = phSensor.read();
    // double ORP = orpSensor.read();

    // To use an offboard temperature measurement: `double pH = ph_sensor.read(temp_sensor.read());
    Serial.print("pH: "); Serial.println(pH);

    // Serial.print("ORP: "); Serial.println(ORP);

    delay(1000);
}