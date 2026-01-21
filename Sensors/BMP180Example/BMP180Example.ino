/**
 * @brief Uses the built in Mama Duck with a DHT11 amd BMP180 sensor
 * 
 * @date 2024-06-14
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <string>
#include <arduino-timer.h>
#include <CDP.h>

#ifdef SERIAL_PORT_USBVIRTUAL
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

// Setup BMP180
#include <Adafruit_BMP085_U.h>
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

bool sendData(std::vector<byte> message);
bool runSensor(void *);

// create a built-in mama duck
MamaDuck duck;

auto timer = timer_create_default();
const int INTERVAL_MS = 10000;

int counter = 1;
bool setupOK = false;

void setup() {
  
  // The Device ID must be exactly 8 bytes otherwise it will get rejected
  std::string deviceId("MAMABMP1");
  std::vector<byte> devId;
  devId.insert(devId.end(), deviceId.begin(), deviceId.end());
  if (duck.setupWithDefaults(devId) != DUCK_ERR_NONE) {
    Serial.println("[MAMA] Failed to setup MamaDuck");
    return;
  }

  // BMP setup
  if (!bmp.begin()) {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print(
        "Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  } else {
    Serial.println("BMP on");
  }

  // initialize the timer. The timer will trigger sending a counter message.
  timer.every(INTERVAL_MS, runSensor);
  
  setupOK = true;
  Serial.println("[MAMA] Setup OK!");
}

void loop() {
  timer.tick();
  // Use the default run(). The Mama duck is designed to also forward data it receives
  // from other ducks, across the network. It has a basic routing mechanism built-in
  // to prevent messages from hoping endlessly.
  duck.run();
}

std::vector<byte> stringToByteVector(const String& str) {
    std::vector<byte> byteVec;
    byteVec.reserve(str.length());

    for (unsigned int i = 0; i < str.length(); ++i) {
        byteVec.push_back(static_cast<byte>(str[i]));
    }

    return byteVec;
}

bool runSensor(void *) {
  bool result;

  String bmp = getBMP();

  String message = String("Counter:")+String(counter)+" "+bmp; 

  result = sendData(stringToByteVector(message));
  if (result) {
    Serial.println("[MAMA] runSensor ok.");
  } else {
    Serial.println("[MAMA] runSensor failed.");
  }
  
  return result;
}

String getBMP() {
  float T, P;

  bmp.getTemperature(&T);
  Serial.print("[SENSOR]: Temperature: ");
  Serial.println(T);

  bmp.getPressure(&P);
  Serial.print("[SENSOR]: Pressure: ");
  Serial.println(P);

  String sensorVal = "Temp:" + String(T) + " Pres:" + String(P);

  return sensorVal;
}

bool sendData(std::vector<byte> message) {
  bool sentOk = false;
  
  int err = duck.sendData(topics::sensor, message);
  if (err == DUCK_ERR_NONE) {
     counter++;
     sentOk = true;
  }
  if (!sentOk) {
    Serial.println("[MAMA] Failed to send data. error = " + String(err));
  }
  return sentOk;
}