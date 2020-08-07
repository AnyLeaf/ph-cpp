#include <Anyleaf.h>

PhSensor ph_sensor;

void setup(void) {
    Serial.begin(9600);

    Rtd sensor = Rtd(10, MAX31865_3WIRE);

}

void loop(void) {
    Serial.print("Temp: "); Serial.println(sensor.read());

    delay(1000);
}