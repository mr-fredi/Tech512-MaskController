#include <Adafruit_BMP280.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <PDM.h>

// Global Variables
Adafruit_BMP280 bmp280;     // temperautre, barometric pressure
Adafruit_LSM6DS33 lsm6ds33; // accelerometer, gyroscope
Adafruit_SHT31 sht30;       // humidity

float temperature, pressure, altitude;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float humidity;
int32_t mic;
int fanPin = 6;

extern PDMClass PDM;
short sampleBuffer[256];  // buffer to read samples into, each sample is 16-bits
volatile int samplesRead; // number of samples read

void SensorsSetup(void) {
    Serial.begin(115200);
    //  while (!Serial) delay(10);
    //  Serial.println("Feather Sense Sensor Demo");

    // initialize the sensors
    bmp280.begin();
    lsm6ds33.begin_I2C();
    sht30.begin();
    PDM.onReceive(onPDMdata);
    PDM.begin(1, 16000);
}

void FanSetup(void) {
    pinMode(fanPin, OUTPUT); // sets the pin as output
}

void BluetoothSetup(void) {
    // TODO: integrate with bluetooth's setup
}

void setup(void) {
    SensorsSetup();
    FanSetup();
    // BluetoothSetup();
}

void SensorProcess(void) {
    temperature = bmp280.readTemperature();
    pressure = bmp280.readPressure();
    altitude = bmp280.readAltitude(1013.25);

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6ds33.getEvent(&accel, &gyro, &temp);
    accel_x = accel.acceleration.x;
    accel_y = accel.acceleration.y;
    accel_z = accel.acceleration.z;
    gyro_x = gyro.gyro.x;
    gyro_y = gyro.gyro.y;
    gyro_z = gyro.gyro.z;

    humidity = sht30.readHumidity();

    samplesRead = 0;
    mic = getPDMwave(4000);

    Serial.print(temperature);
    Serial.print(",");
    Serial.print(pressure);
    Serial.print(",");
    Serial.print(humidity);
    Serial.print(",");
    Serial.print(altitude);
    Serial.print(",");
    Serial.print(accel_x);
    Serial.print(",");
    Serial.print(accel_y);
    Serial.print(",");
    Serial.print(accel_z);
    Serial.print(",");
    Serial.print(gyro_x);
    Serial.print(",");
    Serial.print(gyro_y);
    Serial.print(",");
    Serial.print(gyro_z);
    Serial.print(",");
    Serial.print(mic);
    Serial.println();
}

void FanProcess(void) {
    int output = int(255 * 0.99);
    analogWrite(fanPin, output);
}

void BluetoothProcess(void) {
    // TODO: integrate with bluetooth's setup
}

void loop(void) {
    SensorProcess();
    FanProcess();
    // BluetoothProcess();

    delay(200);
}

/*****************************************************************/
// Helper Functions.
int32_t getPDMwave(int32_t samples) {
    short minwave = 30000;
    short maxwave = -30000;

    while (samples > 0) {
        if (!samplesRead) {
            yield();
            continue;
        }
        for (int i = 0; i < samplesRead; i++) {
            minwave = min(sampleBuffer[i], minwave);
            maxwave = max(sampleBuffer[i], maxwave);
            samples--;
        }
        // clear the read count
        samplesRead = 0;
    }
    return maxwave - minwave;
}

void onPDMdata() {
    // query the number of bytes available
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    PDM.read(sampleBuffer, bytesAvailable);

    // 16-bit, 2 bytes per sample
    samplesRead = bytesAvailable / 2;
}
