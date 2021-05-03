#include <Anyleaf.h>

EcSensor ecSensor;

void setup(void) {
    ecSensor = EcSensor(1.0); // Set K = 1.0
}

void loop(void) {
    float ec = ecSensor.read();
    float airTemp = ecSensor.read_temp();

    delay(1000);
}