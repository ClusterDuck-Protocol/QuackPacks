/**
 * @file TBeam-AXP-Example.ino
 * @brief Uses the built in Mama Duck with some customizations.
 * 
 * This example is a Mama Duck for the TTGO T-Beam that provides feedback and status on the battery and charging of the duck.
 * 
 * @date 2020-11-10
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <string>
#include "arduino-timer.h"
#include <MamaDuck.h>
#include <DuckDisplay.h>

#ifdef SERIAL_PORT_USBVIRTUAL
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

// AXP setup
#include <Wire.h>
#define XPOWERS_CHIP_AXP192
#define ARDUINO_TTGO_LoRa32_V1
#include <XPowersLib.h>

XPowersPMU axp;

#define XPOWERS_AXP192_BAT_CHG_DONE_IRQ 0x01
#define XPOWERS_AXP192_BAT_CHG_START_IRQ 0x02

const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;


DuckDisplay* display = NULL;

//#define LORA_FREQ 915.0 // Frequency Range. Set for US Region 915.0Mhz
//#define LORA_TXPOWER 20 // Transmit Power
//// LORA HELTEC PIN CONFIG
//#define LORA_CS_PIN 18
//#define LORA_DIO0_PIN 26
//#define LORA_DIO1_PIN -1 // unused
//#define LORA_RST_PIN 14

// Set device ID between ""
String deviceId = "MAMA001";
MamaDuck duck;

auto timer = timer_create_default();
const int INTERVAL_MS = 20000;
char message[32]; 
int counter = 1;

void setup() {
  // We are using a hardcoded device id here, but it should be retrieved or
  // given during the device provisioning then converted to a byte vector to
  // setup the duck NOTE: The Device ID must be exactly 8 bytes otherwise it
  // will get rejected
  std::string deviceId("MAMA0001");

  // Use the default setup provided by the SDK
    duck.setDeviceId(deviceId);
    // initialize the serial component with the hardware supported baudrate
    duck.setupSerial(115200);
    duck.setupRadio();
   // duck.setupRadio(LORA_FREQ, LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN, LORA_DIO1_PIN, LORA_TXPOWER);
  Serial.println("MAMA-DUCK...READY!");

  // initialize the timer. The timer thread runs separately from the main loop
  // and will trigger sending a counter message.
  timer.every(INTERVAL_MS, runSensor);

   Wire.begin(i2c_sda, i2c_scl);

    int ret = axp.begin(Wire, 0x34,i2c_sda,i2c_scl);
    axp.setSysPowerDownVoltage(3000);

    // Set the minimum common working voltage of the PMU VBUS input,
    // below this value will turn off the PMU
    axp.setVbusVoltageLimit(XPOWERS_AXP192_VBUS_VOL_LIM_4V5);

    // Turn off USB input current limit
    axp.setVbusCurrentLimit(XPOWERS_AXP192_VBUS_CUR_LIM_OFF);

    // DC1 700~3500mV, IMAX=1.2A
    axp.setDC1Voltage(3300);
    Serial.printf("DC1  :%s   Voltage:%u mV \n",  axp.isEnableDC1()  ? "+" : "-", axp.getDC1Voltage());

    // DC2 700~2750 mV, IMAX=1.6A;
    axp.setDC2Voltage(700);
    Serial.printf("DC2  :%s   Voltage:%u mV \n",  axp.isEnableDC2()  ? "+" : "-", axp.getDC2Voltage());

    // DC3 700~3500 mV,IMAX=0.7A;
    axp.setDC3Voltage(3300);
    Serial.printf("DC3  :%s   Voltage:%u mV \n",  axp.isEnableDC3()  ? "+" : "-", axp.getDC3Voltage());


    //LDO2 1800~3300 mV, 100mV/step, IMAX=200mA
    axp.setLDO2Voltage(1800);

    //LDO3 1800~3300 mV, 100mV/step, IMAX=200mA
    axp.setLDO3Voltage(1800);

    //LDOio 1800~3300 mV, 100mV/step, IMAX=50mA
    axp.setLDOioVoltage(3300);

    axp.enableDC2();
    axp.enableDC3();
    axp.enableLDO2();
    axp.enableLDO3();
    axp.enableLDOio();

    axp.enableTemperatureMeasure();
    axp.enableBattDetection();
    axp.enableVbusVoltageMeasure();
    axp.enableBattVoltageMeasure();
    axp.enableSystemVoltageMeasure();

    axp.clearIrqStatus();

    axp.enableIRQ(
            XPOWERS_AXP192_BAT_INSERT_IRQ    | XPOWERS_AXP192_BAT_REMOVE_IRQ      |   //BATTERY
            XPOWERS_AXP192_VBUS_INSERT_IRQ   | XPOWERS_AXP192_VBUS_REMOVE_IRQ     |   //VBUS
            XPOWERS_AXP192_PKEY_SHORT_IRQ    | XPOWERS_AXP192_PKEY_LONG_IRQ       |   //POWER KEY
            XPOWERS_AXP192_BAT_CHG_DONE_IRQ  | XPOWERS_AXP192_BAT_CHG_START_IRQ   |    //CHARGE
            // XPOWERS_AXP192_PKEY_NEGATIVE_IRQ | XPOWERS_AXP192_PKEY_POSITIVE_IRQ   |   //POWER KEY
            XPOWERS_AXP192_TIMER_TIMEOUT_IRQ               //Timer
    );
    runSensor(nullptr);
}

void loop() {
  timer.tick();
  // Use the default run(). The Mama duck is designed to also forward data it receives
  // from other ducks, across the network. It has a basic routing mechanism built-in
  // to prevent messages from hoping endlessly.
  duck.run();


}

bool runSensor(void *) {


bool isCharging = axp.isCharging();
bool isFullyCharged = axp.isBatChargeDoneIrq();
float batteryVoltage = axp.getBattVoltage();
float batteryDischarge = axp.getAcinCurrent();
float getTemp = axp.getTemperature();
int battPercentage = axp.getBatteryPercent();
   
    Serial.println("--- T-BEAM Power Information ---");
    Serial.print("Duck charging (1 = Yes): ");
    Serial.println(isCharging);
    Serial.print("Fully Charged: ");
    Serial.println(isFullyCharged);
    Serial.print("Battery Voltage: ");
    Serial.println(batteryVoltage);
    Serial.print("Battery Discharge: ");
    Serial.println(batteryDischarge);  
    Serial.print("Board Temperature: ");
    Serial.println(getTemp);
    Serial.print("battery Percentage: ");
    Serial.println(battPercentage);


    std::string sensorVal =
            "Charging: ";
    sensorVal.append(isCharging ? "Yes" : "No")
            .append(" BattFull: ")
            .append(isFullyCharged ? "Yes" : "No")
            .append(" Voltage: ")
            .append(std::to_string(batteryVoltage))
            .append(" Temp: ")
            .append(std::to_string(getTemp));

  
  duck.sendData(topics::sensor, sensorVal);

  return true;
}
