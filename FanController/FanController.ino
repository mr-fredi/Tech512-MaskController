#include <AutoPID.h>

#define fanPin 6  // LED connected to digital pin 9

// pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define Kp 10.0
#define Ki 1.0
#define Kd 1.0

double temperature = 35.5;
double setPoint = 36.5;
double fanSpeed = 0.99 * 255;

AutoPID PID(&temperature, &setPoint, &fanSpeed, OUTPUT_MIN, OUTPUT_MAX, Kp, Ki, Kd);

void setup() {
    Serial.begin(115200);
    pinMode(fanPin, OUTPUT); // sets the pin as output

    // if temperature is more than 4 degrees below or above setpoint,
    // OUTPUT will be set to min or max respectively
    PID.setBangBang(4);

    // set PID update interval to n ms
    PID.setTimeStep(500);
}

void loop() {
    // 15.20 mA - max current
    // 6.85 mA - pin06 current
    // analogRead values go from 0 to 1023,
    // analogWrite values from 0 to 255
    PID.run();
    analogWrite(fanPin, int(fanSpeed));
    Serial.println(fanSpeed);
    delay(1000);
    // digitalWrite(fanPin, HIGH);
}
