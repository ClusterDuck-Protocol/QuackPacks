# QuackPacks
QuackPacks are powerful modular add-ons that snap right onto Ducks to expand what the ClusterDuck Protocol can do. Think of them as plug-and-play upgrades: sensor integrations, new hardware boards, network isolation features, fresh captive portal designs. If it extends CDP, it can be a QuackPack.

Our goal is to grow a rich, community-driven library of these packs so developers can move faster, build smarter, and stand on the shoulders of Ducks. Whether you're customizing deployments or inventing entirely new capabilities, QuackPacks give you the freedom to innovate without wrestling with the basics.

Build boldly. Share widely. And let your Ducks do a whole lot more.

## Sensors
These sensors are tested and confirmed that it works by CDP.

#### BMP180Example
The BMP180 example is a custom MamaDuck that will send sensor data from the BMP 180 (Temperature, Pressure, Altitude) if integrated with the .

#### WS2812Example
The WS2812 example is a Mama Duck, which also brings some light to the world by incorporating some WS2812 LEDs via the FastLED library.

## TTGO T-Beam
If you have a TTGO T-Beam you can use these custom examples for it.

#### TBeam-GPS-Example
This example is a MamaDuck, that uses using TinyGPS++ to get and send GPS data with the GPS topic based on the set timer.

#### TBeam-Telemetry-Example
Using the TTGO's hardware we can collect all different kinds of live board information such as: onboard temperature, battery voltage (if 18650 installed), and charging.

#### TBeam-AXP-Example

#### TBeam-Sleep-Example

## Custom Papa

#### PaPa-DishDuck and PaPa-DishDuck-WiFi
The papa Iridium Example is built to have an extra Iridium Satelite Backhaul when WiFi breakes down.

#### Papa-Downtime-Counter
This custom PapaDuck keeps track of when it loses access to the internet and for how long. 

### PaPi-DMS-Lite-Examples
[Link to DMS Lite](https://github.com/Project-Owl/dms-lite-docker)

#### PapiDuck-Serial-Example
The Serial Papi Example is a custom papa that will print the incoming data into the Serial Monitor for the PAPI to read.

#### PapiDuck-WiFi-Example
The WiFi Papi Example is a custom papa that will send the incoming data into the Papi.

## Encryption
You can enable encryption. CDP comes with default settings, but set your own IV and AES256 key when in use. **NOTE** It is not secure since the keys are hardcoded in the firmware. This is a work in progress. 

#### DecryptionPapa
Decryption is a very fast operation and using this example your messages will be decrypted before sending to the cloud. Remember to set IV and AES256 key to be the same as what you use on other devices.

#### MamaDecrypt
A custom mama example that uses AES256 to encrypt the messages it generates and relays it. Any message it receives from the network, it can decrypt it.
