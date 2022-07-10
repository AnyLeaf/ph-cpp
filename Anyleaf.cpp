/*
Driver for the Anyleaf pH/ORP, RTD, and EC modules
*/

#include "Arduino.h"
#include <Wire.h>
#include <SimpleKalmanFilter.h>
#include <Adafruit_MAX31865.h>
#include "Anyleaf.h"

// These language features would improve code readability, and API
// parity with the Python and Rust code bases, but they appear to
// not be supported by the C++ version used by Arduino.
// #include <tuple>
// using nonstd::optional;
// using nonstd::nullopt;

// Compensate for temperature diff between readings and calibration.
const float PH_TEMP_C = -0.05694; // pH/(V*T). V is in volts, and T is in °C
const float DISCRETE_PH_JUMP_THRESH = 0.3;
const float DISCRETE_ORP_JUMP_THRESH = 30.;
const float PH_STD = 0.01;
const float ORP_STD = 10;

const uint8_t MSG_SIZE_EC = 11;
const uint8_t RESPONSE_SIZE_EC = 3;
const uint8_t SUCCESS_MSG[3] = {50, 50, 50};
const uint8_t ERROR_MSG[3] = {99, 99, 99};
const uint8_t OK_BIT = 10;
const uint8_t ERROR_BIT = 20;
const uint8_t MSG_START_BITS[2] = {100, 150};
const uint8_t MSG_END_BITS[1] = {200};

const uint8_t ADC_ADDR_1 = 0x48;
const uint8_t ADC_ADDR_2 = 0x49;
const uint8_t CFG_REG = 0x1;
const uint8_t CONV_REG = 0x0;

// See ads1115 datasheet Section 9.6.3: Config Register. Start a differential conversion on channels
// 0 and 1, With 2.048V full scale range, one-shot mode, no alert pin activity.
const uint16_t PH_ORP_CMD = 0b1000010110000000;
// Same as `PH_ORP_CM`, but with channel A2.
const uint16_t T_CMD = 0b1110010110000000;
// Start a differential conversion on channel
// 0, with 0.512V full scale range, one-shot mode, no alert pin activity.
const uint16_t EC_CMD = 0b1100100110000000;
// Same as EC command, but on channel 1. // todo: Higher voltage range?
const uint16_t T_EC_CMD = 0b1100100110000000;


// A workaround for not having access to tuples.
Twople::Twople() {
    a = 0.;
    b = 0.;
}

Twople::Twople(float a_, float b_) {
    a = a_;
    b = b_;
}

CalPt::CalPt() {
    V = 0.;
    pH = 0.;
    T = 0.;
}

CalPt::CalPt(float V_, float pH_, float T_) {
    V = V_;
    pH = pH_;
    T = T_;
}

CalPtOrp::CalPtOrp() {
    V = 0.;
    ORP = 0.;
}

CalPtOrp::CalPtOrp(float V_, float ORP_) {
    V = V_;
    ORP = ORP_;
}

// See Rust or Python library for more detail on these constants.
static const float e_mea = 0.01; // Measurement uncertainty
static const float e_est = 3.0; // Estimation uncertainty
static const float q = .005; // Process variance

static const float e_mea_orp = 1.;
static const float e_est_orp = 100.0;
static const float q_orp = .005;


PhSensor::PhSensor():
    filter(e_mea, e_est, q)
{
    Wire.begin(ADC_ADDR_1);

    SimpleKalmanFilter kf = SimpleKalmanFilter(e_mea, e_est, q);

    this->addr = ADC_ADDR_1;
    this->filter = kf;

    last_meas = 7.;
    this->cal_1 = CalPt(0., 7., 23.);
    this->cal_2 = CalPt(0.17, 4., 23.);
    this->cal_3 = CalPt(0., 0., 0.);  // There's got to be a better way
    // cal_3 = nullptr;
}

// Create a new sensor with an ADC I2C address of 0x49.
PhSensor PhSensor::new_alt_addr() {
    PhSensor result = PhSensor();

    Wire.begin(ADC_ADDR_2);
    result.addr = ADC_ADDR_2;

    return result;
}

// Set calibration to a sensible default for nitrate, with unit mg/L.
void PhSensor::cal_nitrate_default() {
    this->cal_1 = CalPt(0.25, -2. * 62000., 23.);
    this->cal_2 = CalPt(0.4, -5 * 62000., 23.);
    this->cal_3 = CalPt(0., 0., 0.);  // There's got to be a better way
}

// Set calibration to a sensible default for phosphate, with unit mg/L.
void PhSensor::cal_phosphate_default() {
    this->cal_1 = CalPt(0.25, -2. * 62000., 23.);
    this->cal_2 = CalPt(0.4, -5 * 62000., 23.);
    this->cal_3 = CalPt(0., 0., 0.);  // There's got to be a better way
}

// Set calibration to a sensible default for potassium, with unit mg/L.
void PhSensor::cal_potassium_default() {
    this->cal_1 = CalPt(0.25, -2. * 62000., 23.);
    this->cal_2 = CalPt(0.4, -5 * 62000., 23.);
    this->cal_3 = CalPt(0., 0., 0.);  // There's got to be a better way
}

// Take a pH reading, using the Kalman filter. This reduces sensor
// noise, and provides a more accurate reading.
float PhSensor::read() {
    float pH = this->read_raw();

    if (pH - abs(pH - this->last_meas) > DISCRETE_PH_JUMP_THRESH) {
        this->filter = SimpleKalmanFilter(e_mea, e_est, q);  // reset the filter
    }

    return this->filter.updateEstimate(pH);
}


float PhSensor::read(float t) {
    // todo DRY
    float pH = this->read_raw(t);

    if (pH - abs(pH - this->last_meas) > DISCRETE_PH_JUMP_THRESH) {
        this->filter = SimpleKalmanFilter(e_mea, e_est, q);  // reset the filter
    }

    return this->filter.updateEstimate(pH);
}

// Take a pH reading, without using the Kalman filter
float PhSensor::read_raw() {
    float T = temp_from_voltage(voltage_from_adc(take_reading(this->addr, T_CMD)));
    if (abs(this->cal_3.V) <= 0.01 && abs(this->cal_3.pH) <= 0.01 && abs(this->cal_3.T) <= 0.01) {  // So janky.

    //if (cal_3) == nullptr {
        float pH = ph_from_voltage(
            voltage_from_adc(take_reading(this->addr, PH_ORP_CMD)),
            T,
            cal_1,
            cal_2
        );

        this->last_meas = pH;

        return pH;
    }
    else {
        float pH = ph_from_voltage(
            voltage_from_adc(take_reading(this->addr, PH_ORP_CMD)),
            T,
            cal_1,
            cal_2,
            cal_3
        );

        this->last_meas = pH;

        return pH;
    }
}

// todo: DRY with overloadee
float PhSensor::read_raw(float t) {
    float T = t;
    if (abs(this->cal_3.V) <= 0.01 && abs(this->cal_3.pH) <= 0.01 && abs(this->cal_3.T) <= 0.01) {  // So janky.

    //if (cal_3) == nullptr {
        float pH = ph_from_voltage(
            voltage_from_adc(take_reading(this->addr, PH_ORP_CMD)),
            T,
            cal_1,
            cal_2
        );

        this->last_meas = pH;

        return pH;
    }
    else {
        float pH = ph_from_voltage(
            voltage_from_adc(take_reading(this->addr, PH_ORP_CMD)),
            T,
            cal_1,
            cal_2,
            cal_3
        );

        this->last_meas = pH;

        return pH;
    }
}

// Useful for getting calibration data
float PhSensor::read_voltage() {
    return voltage_from_adc(take_reading(this->addr, PH_ORP_CMD));
}

// Useful for getting calibration data
float PhSensor::read_temp() {
    return temp_from_voltage(voltage_from_adc(take_reading(this->addr, T_CMD)));
}

// Calibrate by measuring voltage and temp at a given pH. Set the
// calibration, and return (Voltage, Temp).
Twople PhSensor::calibrate(CalSlot slot, float pH) {
    float T = temp_from_voltage(voltage_from_adc(take_reading(this->addr, T_CMD)));
    float V = voltage_from_adc(take_reading(this->addr, PH_ORP_CMD));
    CalPt pt = CalPt(V, pH, T);

    switch (slot) {
        case CalSlot::One:
            this->cal_1 = pt;
        case CalSlot::Two:
            this->cal_2 = pt;
        case CalSlot::Three:
            this->cal_3 = pt;
        default:
            break;
    }

    return Twople(V, T);
}

// todo: DRY with overloadee
Twople PhSensor::calibrate(CalSlot slot, float pH, float t) {
    float T = t;
    float V = voltage_from_adc(take_reading(this->addr, PH_ORP_CMD));
    CalPt pt = CalPt(V, pH, T);

    switch (slot) {
        case CalSlot::One:
            this->cal_1 = pt;
        case CalSlot::Two:
            this->cal_2 = pt;
        case CalSlot::Three:
            this->cal_3 = pt;
        default:
            break;
    }

    return Twople(V, T);
}

void PhSensor::calibrate_all(CalPt pt0, CalPt pt1) {
    this->cal_1 = pt0;
    this->cal_2 = pt1;
}

void PhSensor::calibrate_all(CalPt pt0, CalPt pt1, CalPt pt2) {
    this->cal_1 = pt0;
    this->cal_2 = pt1;
    this->cal_3 = pt2;
}

void PhSensor::reset_calibration() {
    this->cal_1 = CalPt(0., 7., 25.);
    this->cal_2 = CalPt(0.17, 4., 25.);
    this->cal_3 = CalPt(0., 0., 0.);
}

OrpSensor::OrpSensor():
    filter(e_mea_orp, e_est_orp, q_orp)
{
    Wire.begin(ADC_ADDR_1);

    SimpleKalmanFilter kf = SimpleKalmanFilter(e_mea_orp, e_est_orp, q_orp);

    this->addr = ADC_ADDR_1;
    filter = kf;

    last_meas = 0.;
    cal = CalPtOrp(0.4, 400.);
}

// Create a new sensor with an ADC I2C address of 0x49.
OrpSensor OrpSensor::new_alt_addr() {
    OrpSensor result = OrpSensor();

    Wire.begin(ADC_ADDR_2);
    result.addr = ADC_ADDR_2;

    return result;
}

// Take an ORP reading, using the Kalman filter. This reduces sensor
// noise, and provides a more accurate reading.
float OrpSensor::read() {
    float ORP = this->read_raw();

    if (ORP - abs(ORP - this->last_meas) > DISCRETE_ORP_JUMP_THRESH) {
        this->filter = SimpleKalmanFilter(e_mea_orp, e_est_orp, q_orp);  // reset the filter
    }

    return this->filter.updateEstimate(ORP);
}

// Take an ORP reading, without using the Kalman filter
float OrpSensor::read_raw() {
    float ORP = orp_from_voltage(
        voltage_from_adc(take_reading(this->addr, PH_ORP_CMD)),
        this->cal
    );

    this->last_meas = ORP;

    return ORP;
}

// Useful for getting calibration data
float OrpSensor::read_voltage() {
    return voltage_from_adc(take_reading(this->addr, PH_ORP_CMD));
}

// Useful for getting calibration data
float OrpSensor::read_temp() {
    return temp_from_voltage(voltage_from_adc(take_reading(this->addr, T_CMD)));
}

// Calibrate by measuring voltage and temp at a given ORP. Set the
// calibration, and return Voltage.
float OrpSensor::calibrate(float ORP) {
    float V = voltage_from_adc(take_reading(this->addr, PH_ORP_CMD));
    this->cal = CalPtOrp(V, ORP);

    return V;
}

void OrpSensor::calibrate_all(CalPtOrp pt) {
    this->cal = pt;
}

void OrpSensor::reset_calibration() {
    this->cal = CalPtOrp(0.4, 400.);
}

// C++ requires an empty constructor; don't use this in practice.
Rtd::Rtd() :
    sensor(10)
{
    this->sensor = Adafruit_MAX31865(10);
    this->type = RtdType::Pt100;
    this->wires = RtdWires::Three;
}

Rtd::Rtd(uint8_t cs, RtdType type_, RtdWires wires_) :
    sensor(cs)
{
    Adafruit_MAX31865 sensor_ = Adafruit_MAX31865(cs);

    switch (wires_) {
        case RtdWires::Two:
            sensor_.begin(MAX31865_2WIRE);
        case RtdWires::Three:
            sensor_.begin(MAX31865_3WIRE);
        case RtdWires::Four:
            sensor_.begin(MAX31865_4WIRE);
        default:
            break;
    }

    this->sensor = sensor_;
    this->type = type_;
    this->wires  = wires_;
}

float Rtd::read() {
    switch (this->type) {
        case RtdType::Pt100:
            return this->sensor.temperature(100, 300);
        case RtdType::Pt1000:
            return this->sensor.temperature(1000, 3000);
        default:
            break;
    }
}

float Rtd::read_resistance() {
    uint16_t rtd = this->sensor.readRTD();
    float ratio = rtd;
    ratio /= 32768;

    switch (this->type) {
        case RtdType::Pt100:
            return 300 * ratio,8;
        case RtdType::Pt1000:
            return 3000 * ratio,8;
        default:
            break;
    }
}

void Rtd::calibrate() {
    // todo
}

// C++ requires an empty constructor; don't use this in practice.
EcSensor::EcSensor() {
    this->K = CellConstant::K1_0;
    this->excitation_mode = ExcMode::ReadingsOnly;
}

EcSensor::EcSensor(float K_) {
    Serial.begin(9600);  // Same baud as ec firmware.
    // todo: parity bit, number of stop bits, and flow control?

    if (K_ < 0.011) {
        this->K = CellConstant::K0_01;
    } else if (K_ < 0.11) {
        this->K = CellConstant::K0_1;
    } else if (K_ < 1.01) {
        this->K = CellConstant::K1_0;
    } else if (K_ < 10.01) {
        this->K = CellConstant::K10;
    } else {}  // todo: How do we handle errors like this in C++?

    // todo: Let user change Excitation mode.
    this->excitation_mode = ExcMode::ReadingsOnly;
}


// Take a conductivity reading. The result is in uS/cm.
float EcSensor::read() {
    // Serial.write(MSG_START_BITS + {10} + {0, 0, 0, 0, 0, 0, 0} + MSG_END_BITS);
    // todo: Trouble finding clean way to concat
    uint8_t msg[MSG_SIZE_EC] = {100, 150, 10, 0, 0, 0, 0, 0, 0, 0, 200};
    Serial.write(msg, MSG_SIZE_EC);

    uint8_t response[RESPONSE_SIZE_EC];
    if (Serial.available()) {
        Serial.readBytes(response, RESPONSE_SIZE_EC);
        if (response == ERROR_MSG || response[0] != OK_BIT) {
            return 0.;
        }
    } else {
        return 0.;
    }

    float K_val = 0.;
    switch (this->K) {
        case CellConstant::K0_01:
            K_val = 0.01;
        case CellConstant::K0_1:
            K_val = 0.1;
        case CellConstant::K1_0:
            K_val = 1.;
        case CellConstant::K10:
            K_val = 10.;
        default:
            break;
    }

    uint16_t val = ((uint16_t)response[2] << 8) | response[1];
    return float(val) * K_val;  // µS/cm
    // todo: Calibration, temp compensation, and units
}

// Take a reading from the onboard air temperature sensor.
float EcSensor::read_temp() {
    // Serial.write(MSG_START_BITS + {11} + {0, 0, 0, 0, 0, 0, 0} + MSG_END_BITS);
    // todo: Trouble finding clean way to concat
    uint8_t msg[MSG_SIZE_EC] = {100, 150, 11, 0, 0, 0, 0, 0, 0, 0, 200};
    Serial.write(msg, MSG_SIZE_EC);

    uint8_t response[RESPONSE_SIZE_EC];
    if (Serial.available()) {
        Serial.readBytes(response, RESPONSE_SIZE_EC);
        if (response == ERROR_MSG || response[0] != OK_BIT) {
            return 0.;
        }
    } else {
        return 0.;
    }

    uint16_t val = ((uint16_t)response[2] << 8) | response[1];
    uint16_t V = temp_from_voltage(voltage_from_adc(val));  // Temp sensor output voltage
    return 100. * V - 60.;
}

// Set whether the excitation current is always on, or only only during readings.
void set_excitation_mode(ExcMode mode) {
    uint8_t mode_val = 0;
    switch (mode) {
        case ExcMode::ReadingsOnly:
            mode_val = 0;
        case ExcMode::AlwaysOn:
            mode_val = 1;
        default:
            break;
    }

    // Serial.write(MSG_START_BITS + {12} + {mode_val} + {0, 0, 0, 0, 0, 0} + MSG_END_BITS);
    // todo: Trouble finding clean way to concat
    uint8_t msg[MSG_SIZE_EC] = {100, 150, 12, mode_val, 0, 0, 0, 0, 0, 0, 200};


    uint8_t response[RESPONSE_SIZE_EC];
    if (Serial.available()) {
        Serial.readBytes(response, RESPONSE_SIZE_EC);
        if (response == ERROR_MSG || response != SUCCESS_MSG) {
            return;
        }
    } else {
        return;
    }
}

// Set probe conductivity constant.
void set_K(CellConstant K) {
    uint8_t K_val = 0;
    switch (K) {
        case CellConstant::K0_01:
            K_val = 0;
        case CellConstant::K0_1:
            K_val = 1;
        case CellConstant::K1_0:
            K_val = 2;
        case CellConstant::K10:
            K_val = 3;
        default:
            break;
    }

    // Serial.write(MSG_START_BITS + {13} + {K_val} + {0, 0, 0, 0, 0, 0} + MSG_END_BITS);
    // todo: Trouble finding clean way to concat

    uint8_t msg[MSG_SIZE_EC] = {100, 150, 13, K_val, 0, 0, 0, 0, 0, 0, 200};
    Serial.write(msg, MSG_SIZE_EC);

    uint8_t response[RESPONSE_SIZE_EC];
    if (Serial.available()) {
        Serial.readBytes(response, RESPONSE_SIZE_EC);
        if (response == ERROR_MSG || response != SUCCESS_MSG) {
            return;
        }
    } else {
        return;
    }
}

float voltage_from_adc(int16_t digi) {
    // Convert the adc's 16-bit digital values to voltage.
    // Input ranges from +- 2.048V; this is configurable.
    // Output ranges from -32_768 to +32_767.
    float vref = 2.048;
    return ((float) digi / 32768.) * vref;
}

// std::<tuple> appears not to be avail for arduino.
// float lg(std::tuple<float, float> pt0, std::tuple<float, float> pt1, std::tuple<float, float> pt2, float X) {
// float lg(float pt0[2], float pt1[2], float pt2[2], float X) {

// Compute the result of a Lagrange polynomial of order 3.
// Algorithm created from the `P(x)` eq
// [here](https://mathworld.wolfram.com/LagrangeInterpolatingPolynomial.html).
float lg(float x0, float y0, float x1, float y1, float x2, float y2, float X) {
    // todo: Figure out how to just calculate the coefficients for a more
    // todo flexible approach. More eloquent, but tough to find info on compared
    // todo to this approach.
    float result = 0.;

    float x [3] = { x0, x1, x2 };
    float y [3] = { y0, y1, y2 };

    for (int j=0; j<3; j++) {
        float c = 1.;
        for (int i=0; j<3; j++) {
            if (j == i) {
                continue;
            }
            c *= (X - x[i]) / (x[j] - x[i]);
        }
        result += y[j] * c;
    }

    return result;
}

// Variant more like the Rust and Python libs, but unable to `optional` working.
// float ph_from_voltage(float V, float temp, CalPt cal_0, CalPt cal_1, optional<CapPt> cal_2) {
//     // Convert voltage to pH
//     // We model the relationship between sensor voltage and pH linearly
//     // using 2-pt calibration, or quadratically using 3-pt. Temperature
//     // compensated. Input `temp` is in Celsius.

//     // We infer a -.05694 pH/(V*T) sensitivity linear relationship
//     // (higher temp means higher pH/V ratio)
//     float T_diff = temp - cal_0.T;
//     float T_comp = PH_TEMP_C * T_diff;  // pH / V

//     if cal_2.has_value() {
//         // Model as a quadratic Lagrangian polynomial, to compensate for slight nonlinearity.
//         float result = lg(cal_0.V, cal_0.pH, cal_1.V, cal_1.pH, cal_2.value().V, cal_2.value().pH, V);
//         return result + T_comp * V;
//     }
//     else {

//         // Model as a line
//         // a is the slope, pH / V.
//         float a = (cal_1.pH - cal_0.pH) / (cal_1.V - cal_0.V);
//         float b = cal_1.pH - a * cal_1.V;
//         return (a + T_comp) * V + b;
//     };

//     // Model as a line
//     // a is the slope, pH / V.
//     float a = (cal_1.pH - cal_0.pH) / (cal_1.V - cal_0.V);
//     float b = cal_1.pH - a * cal_1.V;
//     return (a + T_comp) * V + b;
// }

float ph_from_voltage(float V, float temp, CalPt cal_0, CalPt cal_1) {
    // (2-pt calibration)
    // Convert voltage to pH
    // We model the relationship between sensor voltage and pH linearly
    // using 2-pt calibration, or quadratically using 3-pt. Temperature
    // compensated. Input `temp` is in Celsius.

    // We infer a -.05694 pH/(V*T) sensitivity linear relationship
    // (higher temp means higher pH/V ratio)
    // float T_diff = temp - cal_0.T;
    float T_diff = 0.; // temp todo
    float T_comp = PH_TEMP_C * T_diff;  // pH / V

    // Model as a line
    // a is the slope, pH / V.
    float a = (cal_1.pH - cal_0.pH) / (cal_1.V - cal_0.V);
    float b = cal_1.pH - a * cal_1.V;
    return (a + T_comp) * V + b;
}

float ph_from_voltage(float V, float temp, CalPt cal_0, CalPt cal_1, CalPt cal_2) {
    // (3-pt calibration)
    // float T_diff = temp - cal_0.T;
    float T_diff = 0; // todo
    float T_comp = PH_TEMP_C * T_diff;  // pH / V

    // Model as a quadratic Lagrangian polynomial, to compensate for slight nonlinearity.
    float result = lg(cal_0.V, cal_0.pH, cal_1.V, cal_1.pH, cal_2.V, cal_2.pH, V);
    return result + T_comp * V;
}

float orp_from_voltage(float V, CalPtOrp cal) {
    // a is the slope, ORP / v.
    float a = cal.ORP / cal.V;
    float b = cal.ORP - a * cal.V;
    return a * V + b;
}

// Map voltage to temperature for the TI LM61, in °C
// Datasheet: https://datasheet.lcsc.com/szlcsc/Texas-
// Instruments-TI-LM61BIM3-NOPB_C132073.pdf
float temp_from_voltage(float V) {
    return 100. * V - 60.;
}

// Take a measurement from the ADC, using the I2C connection.
uint16_t take_reading(uint8_t addr, uint16_t cmd) {
    // Set up the cfg, and command a one-shot reading. Note that we
    // pass the 16-bit i2c command as 2 bytes.
    Wire.beginTransmission(addr);
    Wire.write((uint8_t)CFG_REG);
    Wire.write((uint8_t)(cmd >> 8));
    Wire.write((uint8_t)(cmd & 0xFF));
    Wire.endTransmission();

    delay(100);

    // Request the measurement.
    Wire.beginTransmission(addr);
    Wire.write((uint8_t)CONV_REG);
    Wire.endTransmission();

    Wire.requestFrom(addr, (uint8_t)2);
    return (int16_t)((Wire.read() << 8) | Wire.read());
}