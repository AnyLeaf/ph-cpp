#include <Anyleaf.h>

EcSensor ec_sensor;

void setup(void) {
    Serial.begin(9600);

    ec_sensor = EcSensor();
}

void loop(void) {
    Serial.print("conductivity: "); Serial.println(ec_sensor.read());

    delay(1000);
}