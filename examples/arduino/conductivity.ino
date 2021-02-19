#include <Anyleaf.h>

EcSensor ecSensor;

void setup(void) {
    Serial.begin(9600);

    ecSensor = EcSensor();
}

void loop(void) {
    Serial.print("conductivity: "); Serial.println(ecSensor.read());

    delay(1000);
}