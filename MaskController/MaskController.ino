// Variables
int fanPin = 6; // LED connected to digital pin 9


void SensorsSetup(void) {
    // TODO:
}


void FanSetup(void) {
    ipinMode(fanPin, OUTPUT); // sets the pin as output
}

void FanSetup(void) {
    // TODO: integrate with bluetooth's setup
}

void setup(void) {
    // SensorsSetup();
    FanSetup();
    // BluetoothSetup();
}

void SensorProcess(void) {
    // TODO:
}

void FanProcess(void) {
    int output = int(255 * 0.99);
    analogWrite(fanPin, output);
}

void FanProcess(void) {
    // TODO:
}

void loop(void) {
    // SensorProcess();
    FanProcess();
    // BluetoothProcess();

    delay(200);
}

/*****************************************************************/
// Helper Functions.
