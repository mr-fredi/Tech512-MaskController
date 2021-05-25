#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#define PIN A0
#define NUMPIXELS 8
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#include <Adafruit_MLX90614.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

#include <SPI.h>
#include <SD.h>

#include <cppQueue.h>
#define  IMPLEMENTATION  FIFO
//OTA DFU service
BLEDfu bledfu;

//Peripheral uart service
BLEUart bleuart;

//Central HRM service Client
/* HRM Service Definitions
   Heart Rate Monitor Service:  0x180D
   Heart Rate Measurement Char: 0x2A37 (Mandatory)
   Body Sensor Location Char:   0x2A38 (Optional)
*/
BLEClientService        hrms(UUID16_SVC_HEART_RATE);
BLEClientCharacteristic hrmc(UUID16_CHR_HEART_RATE_MEASUREMENT);
const String target = "A0:9E:1A:76:5F:8A";
const char* boardName = "Firefighter #3";

double BPM = 0;
double myBPM = 0;//moving average BPM
uint16_t myAge = 30;
double max_BPM = 190;

/*Tempreture*/
Adafruit_MLX90614 mlx = Adafruit_MLX90614(); //Tempreture sensor object
double myTem = 0.0;//moving average Temperature
double T = 98.0; //initial Temperature

/*Accelerometer*/
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

/*SD Card*/
File myFile;
String filename = "DATA.txt";

/*Timer*/
uint16_t start_time;
uint16_t cnt_1 = 0, cnt_2 = 0, cnt_3 = 0, cnt_4 = 0, cnt_5 = 0, cnt_6 = 0, cnt_7 = 0;
bool flag_fell = false;

/*Alert Signal*/
/*Normal 0
  Drink_Water 1
  Helmet_OFF 2
  High_HeartRate 3
  Heat_Stroke 4
  Heat_Overexhaustion 5
  Fall_Down 6*/
byte alert_signal = 0;
//Pin
const int ledButtonPin = A1;
const int vibrateButtonPin = 12;
const int buzzerButtonPin = 10;
const int ledPin = A0;
const int vibratePin = 13;
const int buzzerPin = 11;


/*Data Queue*/
typedef struct strRec {
  double  myBPM;
  double  myTem;
} Rec;
Queue  q(sizeof(Rec), 10, IMPLEMENTATION); // Instantiate queue

void setup()
{
  Serial.begin(115200);
  pixels.begin();
  //  pinMode(ledPin,OUTPUT);
  //  pinMode(ledButtonPin, INPUT);

  pinMode(vibratePin, OUTPUT);
  pinMode(vibrateButtonPin, INPUT);

  pinMode(buzzerPin, OUTPUT);
  pinMode(buzzerButtonPin, INPUT);

  //  digitalWrite(ledPin,LOW);
  digitalWrite(vibratePin, LOW);
  noTone(buzzerPin);

//  #if CFG_DEBUG
//    // Blocking wait for connection when debug mode is enabled via IDE
//    while ( !Serial ) yield();
//  #endif

//  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  for (int i = 0; i < 10; i++) {
    Rec temp = {0.0, 0.0};
    q.push(&temp);
  }

  Serial.println("Firefighter Dual Role onBoard System");
  Serial.println("-------------------------------------\n");

  /*Tempreture Sensor*/
  if (mlx.begin()) {
    Serial.println("Temperature sensor found!");
  }

  /*Accelerometer*/
  //  Serial.println("LIS3DH test!");
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");
  lis.setRange(LIS3DH_RANGE_4_G);
  lis.setDataRate(LIS3DH_DATARATE_50_HZ);

  Serial.print("test");
  /*SD card*/
  if (!SD.begin(A4)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  int i=0;
  while(true){
    filename = "DATA_" + String(i) + ".txt";
    if(SD.exists(filename))i++;
    else break;
  }
  myFile = SD.open(filename, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to data file...");
    myFile.println("timestamp,BPM,Temperature,acc_x,acc_y,acc_z,alert_signal");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening data file");
  }


  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(1, 1);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName(boardName);

  // Callbacks for Peripheral
  Bluefruit.Periph.setConnectCallback(prph_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);

  bledfu.begin();

  // Initialize HRM client
  hrms.begin();

  // Initialize client characteristics of HRM.
  // Note: Client Char will be added to the last service that is begin()ed.
  // set up callback for receiving measurement
  hrmc.setNotifyCallback(hrm_notify_callback);
  hrmc.begin();

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setDisconnectCallback(cent_disconnect_callback);
  Bluefruit.Central.setConnectCallback(cent_connect_callback);

  bleuart.begin();
  bleuart.setRxCallback(prph_bleuart_rx_callback);

  /* Start Central Scanning
     - Enable auto scan if disconnected
     - Interval = 100 ms, window = 80 ms
     - Don't use active scan
     - Filter only accept HRM service
     - Start(timeout) with timeout = 0 will scan forever (until connected)
  */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.filterUuid(hrms.uuid);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds

  // Set up and start advertising
  start_time = millis() / 1000;
  startAdv();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loop()
{
  pixels.clear();

  if (Bluefruit.Central.connected()) {
    if (hrms.discovered()) {
      if (hrmc.discovered()) {
        uint16_t current_time = millis() / 1000;
        //        Serial.print(start_time);
        //        Serial.print(",");
        //        Serial.println(current_time);
        if (current_time - start_time >= 900) {//nudge the user to drink water every 15 minutes
          start_time = current_time;
          T = myTem;
          alert_signal = 1;
          Serial.print("T_i: ");
          Serial.println(T);
        }
        String message = "";
        message += String(current_time) + ",";

        /*Tempreture Sensor*/
        double Tem = mlx.readObjectTempF();

        //pop up the first item and push the latest item.

        Rec rec_out, rec_in;
        rec_in = {BPM, Tem};

        q.pop(&rec_out);
        q.push(&rec_in);
        Serial.print("BPM out:");
        Serial.print(rec_out.myBPM);
        Serial.print(" Tem out:");
        Serial.println(rec_out.myTem);
        Serial.print("BPM in:");
        Serial.print(rec_in.myBPM);
        Serial.print(" Tem in:");
        Serial.println(rec_in.myTem);
        Serial.print("Last BPM:");
        Serial.print(myBPM);
        Serial.print(" Last Tem:");
        Serial.println(myTem);
        double last_BPM = myBPM;
        double last_Tem = myTem;
        //calculate an updated moving average of Tem and BPM;
        myBPM = (myBPM * 10 + rec_in.myBPM - rec_out.myBPM ) / 10;
        myTem = (myTem * 10 + rec_in.myTem - rec_out.myTem ) / 10;

        int BPM_for_send = 0;
        double Tem_for_send = 0;

        if ((current_time - start_time < 20 && abs(myTem - last_Tem) > 1) || myTem < 90 || myTem > 106)Tem_for_send = 0;//sensor check
        else Tem_for_send = myTem;
        if ((current_time - start_time < 20 && abs(myBPM - last_BPM) > 3) || myBPM > 220 || myBPM < 40) BPM_for_send = 0;//sensor check
        else BPM_for_send = (int)myBPM;

        double Delta = myTem - T;
        if (Delta < -15)cnt_1++;
        else cnt_1 = 0;

        if (Delta > 7) cnt_2++;
        else cnt_2 = 0;

        if (Delta > 3.5) cnt_3++;
        else cnt_3 = 0;

        if (Delta > 3) cnt_4++;
        else cnt_4 = 0;

        if (cnt_1 >= 120) {
          alert_signal = 2;
          cnt_1 = 0;
          start_time = current_time;
        } else if (cnt_2 >= 120) {
          alert_signal = 4;
          cnt_2 = 0;
          start_time = current_time;
        } else if (cnt_3 >= 360) {
          alert_signal = 5;
          cnt_3 = 0;
          start_time = current_time;
        } else if (cnt_4 >= 120) {
          alert_signal = 1;
          cnt_4 = 0;
          start_time = current_time;
        }

        if (myBPM > 0.9*max_BPM) {
          cnt_5++;
        }
        else {
          if (myBPM > 0.75*max_BPM) {
            cnt_6++;
          } else cnt_6 = 0;
          cnt_5 = 0;
        }

        if (cnt_5 >= 360) {
          alert_signal = 3;
          cnt_5 = 0;
          start_time = current_time;
        } else if (cnt_5 >= 120) {
          alert_signal = 1;
        } else if (cnt_6 >= 1200) {
          alert_signal = 1;
          cnt_6 = 0;
          start_time = current_time;
        }

//        char buf_c[20];
//        String c_m = String(Delta);
//        c_m += ",";
//        int len_c = c_m.length() + 1;
//        //        bleuart.print(message);
//        //        Serial.println(len);
//        c_m.toCharArray(buf_c, len_c);
//        bleuart.write(buf_c, len_c);

        if (alert_signal == 2) {
          message += "0,";
          message += "0.0,";
        } else {
          message += String(BPM_for_send) + ",";
          message += String(Tem_for_send) + ",";
        }


        /*Accelerometer*/
        lis.read();
        sensors_event_t event;
        lis.getEvent(&event);
        double Acc = sqrt(sq(event.acceleration.x) + sq(event.acceleration.y) + sq(event.acceleration.z));

//        if (Acc >= 20 && (event.acceleration.x >= event.acceleration.z || event.acceleration.y >= event.acceleration.z)) {
//          alert_signal = 6;
//        }
        Serial.print("Acc: ");
        Serial.println(Acc);
        if(Acc >= 20)flag_fell = true;
        if(flag_fell == true && (abs(event.acceleration.x) >7 ||abs(event.acceleration.y) > 7))cnt_7++;
        else{
          cnt_7 = 0;
          flag_fell = false;
        }
        if(cnt_7 >= 20){
          alert_signal = 6;
          flag_fell = false;
          cnt_7 = 0;
        }

        String message_for_SD(message);
        message += String(alert_signal) + "\n";
        message_for_SD += String(event.acceleration.x) + ","; //x-axis
        message_for_SD += String(event.acceleration.y) + ","; //y-axis
        message_for_SD += String(event.acceleration.z) + ","; //z-axis
        message_for_SD += String(alert_signal) + "\n";

        AlertOn(alert_signal);

        myFile = SD.open(filename, FILE_WRITE);
        if (myFile) {
          Serial.print("Writing to data file...");
          myFile.print(message_for_SD);
          // close the file:
          myFile.close();
          Serial.println("done.");
        } else {
          // if the file didn't open, print an error:
          Serial.println("error opening data file");
        }

        //        myFile = SD.open("raw_data.txt");
        //        if (myFile) {
        //          Serial.println("raw_data.txt:");
        //
        //          // read from the file until there's nothing else in it:
        //          while (myFile.available()) {
        //            Serial.write(myFile.read());
        //          }
        //          // close the file:
        //          myFile.close();
        //        } else {
        //          // if the file didn't open, print an error:
        //          Serial.println("error opening raw_data.txt");
        //        }

        char buf2[40];
        int len = message.length() + 1;
        //        bleuart.print(message);
        //        Serial.println(len);
        message.toCharArray(buf2, len);
        bleuart.write(buf2, len);
        alert_signal = 0;
        Serial.print(message);
        Serial.print(message_for_SD);
        delay(500);

      }
    }
  }

}

/*------------------------------------------------------------------*/
/* Peripheral(connect with mobile phone)
  ------------------------------------------------------------------*/
void prph_connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char peer_name[32] = { 0 };
  connection->getPeerName(peer_name, sizeof(peer_name));

  Serial.print("[Prph] Connected to ");
  Serial.println(peer_name);
}

void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("[Prph] Disconnected");
}

void prph_bleuart_rx_callback(uint16_t conn_handle)
{
  (void) conn_handle;

  // Forward data from Mobile to our peripheral
  char msg[20 + 1] = { 0 };
  bleuart.read(msg, 20);

  Serial.print("[Prph] RX: ");
  Serial.println(msg);

  int len = strlen(msg);
  if(len >3){
    int i = 0;
    while(true){
      filename = "DATA_" + String(i) + ".txt";
      if(SD.exists(filename))i++;
      else break;
    }
    myFile = SD.open(filename, FILE_WRITE);

  // if the file opened okay, write to it:
    if (myFile) {
      Serial.print("Writing to data file...");
      myFile.println(String(msg));
      myFile.println("timestamp,BPM,Temperature,acc_x,acc_y,acc_z,alert_signal");
      // close the file:
      myFile.close();
      Serial.println("done.");
    } else {
      // if the file didn't open, print an error:
      Serial.println("error opening data file");
    }
  }else{
    int num =  atoi(msg);
    Serial.println(num);
    //[LED],[Haptic],[Buzzer]
    switch (num) {
      case 0:
        Serial.println("Normal");
      case 1:
        Serial.println("Drink_Water");
        AlertOn(1);
        break;
      case 2:
        Serial.println("Helmet_OFF");
        AlertOn(2);
        break;
      case 3:
        Serial.println("High_HeartRate");
        AlertOn(3);
        break;
      case 4:
        Serial.println("Heat_Stroke");
        AlertOn(4);
        break;
      case 5:
        Serial.println("Heat_Overexertion");
        AlertOn(5);
        break;
      case 6:
        Serial.println("Fall_Down");
        AlertOn(6);
        break;
      default:
        myAge = num;
        max_BPM = 220 - myAge;
        break;
    }
  }



  if (!hrms.discovered() )
  {
    //    hrms.print(str);
    bleuart.println("[Prph] Central role not connected");
  }
}

void AlertOn(int flag) {
  if (flag == 0) return;
  for (int i = 0; i < 5; i++) {
    //    if(flag == 1 || flag == 4 || flag == 5 || flag == 7)digitalWrite(ledPin,HIGH);
    //    if(flag == 1 || flag == 4 || flag == 5 || flag == 7){
    //      pixels.setPixelColor(4, pixels.Color(0, 0, 150));
    //      pixels.show();
    //    }
    //    if(flag == 2 || flag == 4 || flag == 6 || flag == 7)tone(buzzerPin, 2000);
    //    if(flag == 3 || flag == 5 || flag == 6 || flag == 7)digitalWrite(vibratePin,HIGH);
    if (flag == 1 || flag == 3) {
      pixels.setPixelColor(4, pixels.Color(0, 0, 100));
      pixels.show();
    }
    if (flag == 2 || flag == 4 || flag == 5 || flag == 6) {
      pixels.setPixelColor(4, pixels.Color(0, 0, 250));
      pixels.show();
    }
    if (flag == 2 || flag == 3 || flag == 4 || flag == 5 || flag == 6)tone(buzzerPin, 2000);
    if (flag == 1 || flag == 3 || flag == 4 || flag == 5)digitalWrite(vibratePin, HIGH);

    delay(500);
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
    pixels.show();
    //    digitalWrite(ledPin,LOW);
    noTone(buzzerPin);
    digitalWrite(vibratePin, LOW);
    delay(500);
  }
  pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  pixels.show();
  //  digitalWrite(ledPin,LOW);
  noTone(buzzerPin);
  digitalWrite(vibratePin, LOW);
}

/**
   Callback invoked when scanner pick up an advertising data
   @param report Structural advertising data
*/
/*------------------------------------------------------------------*/
/* Central
  ------------------------------------------------------------------*/
void scan_callback(ble_gap_evt_adv_report_t* report)//scan the available hrms device and connect to the target one
{
  PRINT_LOCATION();

  Serial.println("Scanning");
  //  // MAC is in little endian --> print reverse
  //  Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  //  Serial.print("\n");
  //  Serial.println(report->peer_addr.addr[0]);

  char mac[18];

  snprintf(mac, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
           report->peer_addr.addr[5],
           report->peer_addr.addr[4],
           report->peer_addr.addr[3],
           report->peer_addr.addr[2],
           report->peer_addr.addr[1],
           report->peer_addr.addr[0]
          );

  if (String(mac) == target)Bluefruit.Central.connect(report); //connect
  else Bluefruit.Scanner.resume();//continue scanning
}

/**
   Callback invoked when an connection is established
   @param conn_handle
*/
void cent_connect_callback(uint16_t conn_handle)
{
  Serial.println("Connected");
  Serial.print("Discovering HRM Service ... ");

  // If HRM is not found, disconnect and return
  if ( !hrms.discover(conn_handle) )
  {
    Serial.println("Found NONE");

    // disconnect since we couldn't find HRM service
    //    Bluefruit.disconnect(conn_handle);

    //    return;
  }
  // Once HRM service is found, we continue to discover its characteristic
  Serial.println("Found it");

  Serial.print("Discovering Measurement characteristic ... ");
  if ( !hrmc.discover() )
  {
    // Measurement chr is mandatory, if it is not found (valid), then disconnect
    Serial.println("not found !!!");
    Serial.println("Measurement characteristic is mandatory but not found");
    //    Bluefruit.disconnect(conn_handle);
    //    return;
  }
  Serial.println("Found it");

  // Reaching here means we are ready to go, let's enable notification on measurement chr
  if ( hrmc.enableNotify() )
  {
    Serial.println("Ready to receive HRM Measurement value");
  } else
  {
    Serial.println("Couldn't enable notify for HRM Measurement. Increase DEBUG LEVEL for troubleshooting");
  }
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
*/
void cent_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.print("[Cent]Disconnected, reason = 0x"); Serial.println(reason, HEX);
}


/**
   Hooked callback that triggered when a measurement value is sent from peripheral
   @param chr   Pointer client characteristic that even occurred,
                in this example it should be hrmc
   @param data  Pointer to received data
   @param len   Length of received data
*/
void hrm_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)//receive data from hrms device
{
  //  Serial.print("HRM Measurement: ");
  if ( data[0] & bit(0) )
  {
    uint16_t value;
    memcpy(&value, data + 1, 2);
    BPM = (double)value;
  }
  else
  {
    BPM = (double)data[1];
  }
}
