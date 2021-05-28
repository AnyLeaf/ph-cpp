#ifndef __ANYLEAF_H__
#define __ANYLEAF_H__

#include "Arduino.h"
#include <Adafruit_MAX31865.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
// #include <tuple>
// #include "optional.hpp"
// using nonstd::nullopt;

struct Twople {
    public:
        Twople();
        Twople(float a, float b);
        float a;
        float b;
};

struct CalPt {
public:
    CalPt();
    CalPt(float V_, float pH_, float T_);
    float V; // voltage, in Volts
    float pH;
    float T; // in Celsius
};

struct CalPtOrp {
public:
    CalPtOrp();
    CalPtOrp(float V_, float ORP_);
    float V; // voltage, in Volts
    float ORP;
};

struct CalPtEc {  // todo: Currently unused
public:
    CalPtEc();
    CalPtEc(float reading_, float ec_, float T_);
    float reading; // Raw conductivity in uS/cm
    float ec;  // calibrated conductivity
    float T; // Temp in C
};

enum class CalSlot {
    // Keeps our calibration organized, so we track when to overwrite.
    One,
    Two,
    Three,
};

class PhSensor {
public:
    PhSensor();
    static PhSensor new_alt_addr();  // named constructor
    void cal_nitrate_default();
    void cal_phosphate_default();
    void cal_potassium_default();
    SimpleKalmanFilter filter;
    float read();
    float read(float t);
    float read_raw();
    float read_raw(float t);
    float read_voltage();
    float read_temp();
    Twople calibrate(CalSlot slot, float pH);
    Twople calibrate(CalSlot slot, float pH, float t);
    void calibrate_all(CalPt pt0, CalPt pt1);
    void calibrate_all(CalPt pt0, CalPt pt1, CalPt pt2);
    void reset_calibration();

private:
    uint8_t addr;
    float last_meas;
    CalPt cal_1;
    CalPt cal_2;
    CalPt cal_3;
};

class OrpSensor {
public:
    OrpSensor();
    static OrpSensor new_alt_addr();  // named constructor
    SimpleKalmanFilter filter;
    float read();
    float read_raw();
    float read_voltage();
    float read_temp();
    float calibrate(float ORP);
    void calibrate_all(CalPtOrp pt);
    void reset_calibration();

private:
    uint8_t addr;
    float last_meas;
    CalPtOrp cal;
};

enum class CellConstant {
    K0_01,
    K0_1,
    K1_0,
    K10,
};

enum class ExcMode {
    ReadingsOnly,
    AlwaysOn,
};

class EcSensor {
public:
    EcSensor();
    EcSensor(float K_);

    CellConstant K;
    ExcMode excitation_mode;
    float read();
    float read_temp();
    void set_excitation_mode(ExcMode mode);
    void set_K(CellConstant K);

private:
//    Serial ser; // todo maybe
//    CalPtEc cal; // todo
};

enum class RtdType {
    Pt100,
    Pt1000,
};

enum class RtdWires {
    Two,
    Three,
    Four,
};

class Rtd {
public:
    Rtd();
    Rtd(uint8_t cs, RtdType type_, RtdWires wires_);

    float read();
    float read_resistance();
    void calibrate();

private:
    Adafruit_MAX31865 sensor;
    RtdType type;
    RtdWires wires;
    // CalPtT cal;
};

float voltage_from_adc(int16_t digi);
float lg(float x0, float y0, float x1, float y1, float x2, float y2, float X);
float ph_from_voltage(float V, float temp, CalPt cal_0, CalPt cal_1);
float ph_from_voltage(float V, float temp, CalPt cal_0, CalPt cal_1, CalPt cal_2);
float orp_from_voltage(float V, CalPtOrp cal);
// todo: No `optional` support on Arduino
// float ph_from_voltage(float V, float temp, CalPt cal_0, CalPt cal_1, optional<CapPt> cal_2)
float temp_from_voltage(float V);
uint16_t take_reading(uint8_t addr, uint16_t cmd);

#endif