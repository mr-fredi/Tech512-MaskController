#include <Adafruit_BMP280.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <bluefruit.h>
#include <BLEService.h>
#include <BLEClientService.h>
#include <PDM.h>

// Global Variables

// Set this higher to automatically stop advertising after a time
#define ADV_TIMEOUT 0  // seconds. 
#define DEBUG 1

#define FANPIN 6
#define BATPIN A6

#define OUTPUT_MIN 0
#define OUTPUT_MAX 255

// The following code is for setting a name based on the actual device MAC address
// Where to go looking in memory for the MAC
typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)
#define MAC_ADDRESS_HIGH  (*(pREG32 (0x100000a8)))
#define MAC_ADDRESS_LOW   (*(pREG32 (0x100000a4)))

BLEUart bleUart;
Adafruit_BMP280 bmp280;      // temperautre, barometric pressure
Adafruit_SHT31 sht30;        // humidity

float fanSpeedPercent = 0.99;
float temperature, pressure, humidity;
int vBattery = 0;
int32_t mic;

extern PDMClass PDM;
short sampleBuffer[256];  // buffer to read samples into, each sample is 16-bits
volatile int samplesRead; // number of samples read

// Null-terminated string must be 1 longer than you set it, for the null
char ble_name[15] = "SmartMaskXXXX\n";

/*****************************************************************/
// Functions
void SensorSetup(void);
void FanSetup(void);
void BluetoothSetup(void);

void SensorProcess(void);
void FanProcess(void);
void BatteryProcess(void);

void startAdv(void);
void byte_to_str(char* buff, uint8_t val);
char nibble_to_hex(uint8_t nibble);
void adv_stop_callback(void);
void SerialPrint(void);
void BTPrint(void);

/*****************************************************************/
// Main program
void setup(void) {
    // Start serial monitor when debug mode is on
    // NOTE: Under debug mode, open serial monitor once
    // to activate bluetooth.
    if (DEBUG) {
      Serial.begin(115200);
      Serial.println("Serial port activated.");
    }
    SensorsSetup();
    FanSetup();
//    BluetoothSetup();
    MicSetup();
}

void loop(void) {
    SensorProcess();
    FanProcess();
    BatteryProcess();
    MicProcess();

//    if (Bluefruit.connected()) BTPrint();
    if (DEBUG) SerialPrint();

    delay(1000);
}

/*****************************************************************/
// Main functions implementations.

void SensorsSetup(void) {
    // initialize the sensors
    bmp280.begin();
    sht30.begin();
}

void FanSetup(void) {
    pinMode(FANPIN, OUTPUT);  // sets the pin as output
}

void BluetoothSetup(void) {
    Bluefruit.begin();
    Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

    // Replace the XXXX with the lowest two bytes of the MAC Address
    // The commented lines show you how to get the WHOLE MAC address
    //  uint32_t addr_high = ((MAC_ADDRESS_HIGH) & 0x0000ffff) | 0x0000c000;
    uint32_t addr_low  = MAC_ADDRESS_LOW;

    // Fill in the XXXX in ble_name
    byte_to_str(&ble_name[9], (addr_low >> 8) & 0xFF);
    byte_to_str(&ble_name[11], addr_low & 0xFF);
    // Set the name we just made
    Bluefruit.setName(ble_name);

    bleUart.begin();

    // start advertising
    startAdv();
}

void startAdv(void) {
    // Advertising packet
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(bleUart);

    // Tell the BLE device we want to send our name in a ScanResponse if asked.
    Bluefruit.ScanResponse.addName();

    /* Start Advertising
    * - Enable auto advertising if disconnected
    * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
    * - Timeout for fast mode is 30 seconds
    * - Start(timeout) with timeout = 0 will advertise forever (until connected)
    *
    * For recommended advertising interval
    * https://developer.apple.com/library/content/qa/qa1931/_index.html
    */
    Bluefruit.Advertising.setStopCallback(adv_stop_callback);
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);  // number of seconds in fast mode
    // Stop advertising entirely after ADV_TIMEOUT seconds
    Bluefruit.Advertising.start(ADV_TIMEOUT);
}

void MicSetup() {
  PDM.onReceive(onPDMdata);
  // PDM.begin(channel, SampleRate);
  while (!PDM.begin(1, 16000)) {
    Serial.println("Fail to start PDM.");
    delay(1000);
  }
}

void SensorProcess(void) {
    temperature = bmp280.readTemperature();
    pressure = bmp280.readPressure();
 
    humidity = sht30.readHumidity();
}

void FanProcess(void) {
    fanSpeedPercent = (0.0098 * humidity) + 0.0257;
    analogWrite(FANPIN, round(fanSpeedPercent * 255));
}

void BatteryProcess(void) {
    vBattery = round((((analogRead(BATPIN) * 2 * 3.3) / 1024) / 4.2) * 100);
    if (vBattery == 100) --vBattery;
}

void MicProcess() {
  samplesRead = 0;
  mic = getPDMwave(44100);
}

/*****************************************************************/
// Helper functions implementations.

// convert an 8-bit byte to a string of 2 hexadecimal characters
void byte_to_str(char* buff, uint8_t val) {
    buff[0] = nibble_to_hex(val >> 4);
    buff[1] = nibble_to_hex(val);
}

// convert a 4-bit nibble to a hexadecimal character
char nibble_to_hex(uint8_t nibble) {
    nibble &= 0xF;
    return nibble > 9 ? nibble - 10 + 'A' : nibble + '0';
}

// Callback invoked when advertising is stopped by timeout
void adv_stop_callback(void) {
    if (DEBUG) Serial.println("Advertising time passed, advertising will now stop.");
}

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
//  Serial.println(minwave);
//  Serial.println(maxwave);
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

// Serial print
void SerialPrint(void) {
    Serial.print(temperature);
    Serial.print(",");
    Serial.print(pressure);
    Serial.print(",");
    Serial.print(humidity);
    Serial.print(",");
    Serial.print(vBattery);
    Serial.print(",");
    Serial.print(round(fanSpeedPercent * 255));
    Serial.print(",");
    Serial.print(mic);
    Serial.println();
}

// Bluetooth Print
void BTPrint(void) {
    String tempBuff = String(temperature + 0.05);
    tempBuff[tempBuff.length() - 1] = ',';

    tempBuff += String(int(fmod(round(pressure), 100000) * 10));
    tempBuff[tempBuff.length() - 1] = ',';
    
    tempBuff += String(humidity + 0.05);
    tempBuff[tempBuff.length() - 1] = ',';

    tempBuff += (String(vBattery) + String(","));
    
    tempBuff += (String(round(fanSpeedPercent * 100) + String(",")));

    tempBuff += String(mic);

    tempBuff += String("\n\r");
    char buff[tempBuff.length() + 1] = {0};
    tempBuff.toCharArray(buff, tempBuff.length());
               
    bleUart.write(buff);
    bleUart.flush(); 
}
