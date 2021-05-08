# include <bluefruit.h>
# include <BLEService.h>
# include <BLEClientService.h>

// Set this higher to automatically stop advertising after a time
#define ADV_TIMEOUT   0  // seconds.

// The following code is for setting a name based on the actual device MAC address
// Where to go looking in memory for the MAC
typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)
#define MAC_ADDRESS_HIGH  (*(pREG32 (0x100000a8)))
#define MAC_ADDRESS_LOW   (*(pREG32 (0x100000a4)))

BLEUart bleUart;

// Null-terminated string must be 1 longer than you set it, for the null
char ble_name[14] = "SmartMaskXXXX";

void setup() {
    Serial.begin(115200);
    while ( !Serial ) delay(10);   // for nrf52840 with native usb

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
    // Serial.print("Advertising is started: ");
    // Serial.println(ble_name);
}

void startAdv(void) {
    // Advertising packet
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
//    Bluefruit.Advertising.addTxPower();
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

void loop() {
    if (Bluefruit.connected()) {
        // TODO: pass data here.
        // uint8_t buff[];
        bleUart.write(ble_name, sizeof(ble_name));
    }
}

/**
 * Callback invoked when advertising is stopped by timeout
 */
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

void adv_stop_callback(void) {
    Serial.println("Advertising time passed, advertising will now stop.");
}
