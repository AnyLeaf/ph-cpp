#include <Anyleaf.h>

Rtd sensor;

void setup(void) {
    Serial.begin(9600);

    // The first argument is the pin connected to the `CS` pin of the module. It can be any
    // digital GPIO pin.
    sensor = Rtd(10, RtdType::Pt100, RtdWires::Three);  // 3-wire Pt100
    // sensor = Rtd(10, RtdType::Pt1000, RtdWires::Two);  // 2-wire Pt1000
}

void loop(void) {
    Serial.print("Temp: "); Serial.println(sensor.read());

    Serial.print("Resistance: "); Serial.println(sensor.read_resistance());

    delay(1000);
}