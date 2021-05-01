#include <Anyleaf.h>

EcSensor ecSensor;

void setup(void) {
    Serial.begin(9600);

    ecSensor = EcSensor(1.0); // Set K = 1.0
}

void loop(void) {
    Serial.print("Conductivity: "); Serial.println(ecSensor.read());
    Serial.print("Air temperature: "); Serial.println(ecSensor.read_temp());

    delay(1000);
}