///
/// @mainpage	Bluemix-IOT-Demo
///
/// @details	Description of the project
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		10/5/16 5:53 PM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2016
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Bluemix_IOT_Demo.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		10/5/16 5:53 PM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2016
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
#include "libpandora_types.h"
#include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
#include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#include "application.h"
#elif defined(ESP8266) // ESP8266 specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE


/*
 Basic MQTT example
 
 This sketch demonstrates the basic capabilities of the library.
 It connects to an MQTT server then:
 - publishes "hello world" to the topic "outTopic"
 - subscribes to the topic "inTopic", printing out any messages
 it receives. NB - it assumes the received payloads are strings not binary
 
 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.
 
*/

// Include application, user and local libraries
#include "SPI.h"
#include "Adafruit_CC3000.h"
#include "PubSubClient.h"
//#include <string.h>
//#include "utility/debug.h"
//#include <stdlib.h>
#include "GPS.h"      // Code and Library from: https://github.com/rvnash/ultimate_gps_teensy3
//#include <WProgram.h>
#include <i2c_t3.h>
#include "KeysandPasswords.h"   // Comment this out if you don't want to store keys and passwords in a separate file


/*    Uncomment this section and provide your own keys and passwords or store in a separate KeysandPasswords file as I do
// Wifi Setup
#define MOBILE 0    // allows you to store a different set of credentials if you are using a wifi hotspot
#if MOBILE
#define WLAN_SSID       "Mobile SSID"  // cannot be longer than 32 characters!
#define WLAN_PASS       "Mobile Password"
#else
#define WLAN_SSID       "Home or Office SSID"  // cannot be longer than 32 characters!
#define WLAN_PASS       "Home of Office Passwor"
#endif
#define WLAN_SECURITY            WLAN_SEC_WPA2       // Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
// Bluemix MQTT Setup
#define MS_PROXY                "<insert your user ID here>.messaging.internetofthings.ibmcloud.com"
#define MQTT_PORT               1883
#define MQTT_CLIENT_ID          "Your client ID from Bluemix" // d:org_id:device_type:device_id, d - device, <org_id>, <device_type>, <device_id>
#define MQTT_TOPIC              "iot-2/evt/status/fmt/json"
#define AUTHMETHOD              "use-token-auth"
#define AUTHTOKEN               "Your authorization token"
byte mac[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // replace with your device's MAC consistent with <device_id>
*/

 // CC3000 Setup
#define ADAFRUIT_CC3000_IRQ     2                               // MUST be an interrupt pin!
#define ADAFRUIT_CC3000_VBAT    5                               // Can be any pin
#define ADAFRUIT_CC3000_CS      10                              // Can be any pin

// Set up the Accelerometer
const int MMA8452_ADDRESS = 0x1D; // The MMA8452 breakout board defaults to 1 for an address of 0x1D
const byte SCALE = 2;  // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
const byte dataRate = 3;  // output data rate - 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
const int int1Pin = 4;         // Not used now but wired for future use
const int int2Pin = 3;         // This is the interrupt pin that registers taps
// Carrier Board Pins
const int TrimPot = A0;        // Potentiometer used to adjust sensitivity
const int TrimPot1 = A1;       // Potentiometer used to adjust delay
const int ThermistorPin = A6;  // 10k Thermistor voltage divider here
const int TempPowerPin = 9;    // We turn this pin on to collect temperature data
const int ledRED = 16;         // Tap Indicator LED
const int ledYELLOW = 17;      // Indicates setup mode
// Global Variables
unsigned long ipaddr;
int value = 0;
unsigned int ReportingInterval = 10000;  // How often do you want to send to Watson IOT
unsigned long LastReport = 0;            // Keep track of when we last sent data
int TimeOut = 20000;                      // How long will we wait for a command to complete
char c;                                  // Used to relay input from the GPS serial feed
String Location = "";                    // Will build the Location string here
String Orientation = "";                   // Where we store orientation
int InputValue = 0;            // Raw sensitivity input
byte Sensitivity = 0x00;       // Hex variable for sensitivity
static byte source;
int ledState = LOW;            // variable used to store the last LED status, to toggle the light
int Debounce = 1000;           // Time in ms between knocks
float tempC = 0.0;
char Axes[4]={'x','y','z'};
int accelCount[3];  // Stores the 12-bit signed value


// Use hardware SPI for the remaining pins, for UNO, SCK = 13, MISO = 12, MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIVIDER);

// Initialize the GPS
HardwareSerial &gpsSerial = Serial1;
GPS gps(&gpsSerial,true);

// Prototypes
String buildJson();         // Function where we build the JSON payload
void callback(char* topic, byte* payload, int length);      // Here we can capture reply messages from Bluemix
boolean ParseLocation(); // Sample data: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
void readAccelData(int * destination);   // Here is where we get acceleration on x,y,and z axes
void getData();     // This section will sample the thermister for a temp measurement
boolean testConnectionToBluemix();      // Let's make sure we can get to Bluemix before proceeding
boolean GetConnected();  // Connection to Wifi
void displayMACAddress(void);  //Tries to read the 6-byte MAC address of the CC3000 module
bool getIPAddress(void); // Tries to read the IP address and other connection details
void initMMA8452(byte fsr, byte dataRate);   // Initialize the MMA8452 registers
void portraitLandscapeHandler(); // Reads the p/l source register and prints what direction the sensor is now facing
void MMA8452Standby();  // Sets the MMA8452 to standby mode to change most register settings
void MMA8452Active();   // Sets the MMA8452 to active mode to output data
byte readRegister(int I2CAddress, byte address); // Read a single byte from address and return it as a byte
void readRegisters(int I2CAddress, byte address, int i, byte * dest);    // Read i registers sequentially, starting at address into the dest byte
void writeRegister(int I2CAddress, unsigned char address, unsigned char data);  // Writes a single byte (data) into address
void BlinkForever(); // When something goes badly wrong...
void NonBlockingDelay(int millisDelay);  // Used for a non-blocking delay


//EthernetClient ethClient;
Adafruit_CC3000_Client ethClient;
PubSubClient client(MS_PROXY, 1883, 0, ethClient);

void setup()
{
    Serial.begin(19200);
    delay(1000);    // Need this delay or we will miss the serial stream - unique to Teensy
    analogReference(EXTERNAL);
    pinMode(TempPowerPin, OUTPUT);
    pinMode(ledRED, OUTPUT);
    pinMode(ledYELLOW,OUTPUT);
    digitalWrite(ledYELLOW,HIGH);
    pinMode(int2Pin, INPUT);     // Set up the Accelerometer interrupt pins, they're set as active high, push-pull
    digitalWrite(int2Pin, HIGH);
    pinMode(int1Pin, INPUT);
    digitalWrite(int2Pin, HIGH);
    Serial.println();
    Serial.println(F("---------------------------"));
    Serial.println(F("Connected Location Logger  "));
    Serial.println(F("by Chip McClelland  "));
    Serial.println(F("Open Source - Hardware & Software"));
    Serial.println(F("---------------------------"));
    Wire.begin(); // Start serial on i2c
    // Initialize the Accelerometer
    Serial.print("Initializing the accelerometer: ");
    byte c = readRegister(MMA8452_ADDRESS, 0x0D);            // Read WHO_AM_I register to test communications
    if (c == 0x2A) {                  // WHO_AM_I should always be 0x2A
        initMMA8452(SCALE, dataRate);   // init the accelerometer if communication is OK
        Serial.println("Succeeded");
    }
    else {                          // Problem with communucations
        Serial.print("Failed at address: 0x");
        Serial.println(c, HEX);
        BlinkForever();  // We are done
    }
    // Initialize the Wi-Fi Module
    if (!cc3000.begin())
    {
        Serial.println(F("Unable to initialise the CC3000! Check your wiring?"));
        BlinkForever();
    }
    Serial.println("CC3000 Initialized");
    displayMACAddress();
    if(!GetConnected())
    {
        Serial.println(F("Could not connect to the WiFi network"));
        BlinkForever();
    }
    if(!getIPAddress())
    {
        Serial.println(F("Could not get IP address from DCHP"));
        BlinkForever();
    }
    if(!testConnectionToBluemix())
    {
        Serial.println(F("Could not resolve Bluemix Address"));
        BlinkForever();
    }
    Serial.println("Success");
    
    digitalWrite(ledYELLOW,LOW);
}

void loop()
{
    if (digitalRead(int2Pin)==1) {    // If int2 goes high, either p/l has changed or there's been a single/double tap
        source = readRegister(MMA8452_ADDRESS, 0x0C);  // Read the interrupt source reg.
        readRegister(MMA8452_ADDRESS,0x22);  // Reads the PULSE_SRC register to reset it
        if ((source & 0x08)==0x08) { // If we see the tap register go high, we will take a reading
            Serial.println("Movement detected...");
            if (millis() >= LastReport + ReportingInterval) {
                LastReport = millis();
                ledState = !ledState;                        // toggle the status of the ledPin:
                digitalWrite(ledRED, ledState);              // update the LED pin itself
                if (gps.sentenceAvailable()) gps.parseSentence();
                if (gps.newValuesSinceDataRead()) {
                    gps.dataRead();
                    ParseLocation();
                }
                getData();
                readAccelData(accelCount);  // Read the x/y/z adc values
                //Below we'll print out the ADC values for acceleration
                Serial.print("= ");
                for (int i=0; i<3; i++)
                {
                    Serial.print(Axes[i]);
                    Serial.print(accelCount[i]);
                    Serial.print("\t\t");
                }
                Serial.println();
                portraitLandscapeHandler();
                Serial.println("logging");
                if (!client.connected()) {
                    Serial.print("Trying to connect to: ");
                    Serial.println(MQTT_CLIENT_ID);
                    client.connect(MQTT_CLIENT_ID, AUTHMETHOD, AUTHTOKEN);
                }
                if (client.connected() ) {
                    String json = buildJson();
                    char jsonStr[200];
                    json.toCharArray(jsonStr,200);
                    boolean pubresult = client.publish(MQTT_TOPIC,jsonStr);
                    Serial.print("attempt to send ");
                    Serial.println(jsonStr);
                    Serial.print("to ");
                    Serial.println(MQTT_TOPIC);
                    if (pubresult) Serial.println("successfully sent");
                    else Serial.println("unsuccessfully sent");
                }
            }

        }
    }
}

String buildJson()  // Function where we build the JSON payload
{
    String data = "{\"d\":{\"Temp\":\"";
    data+=int(tempC);
    data+="\",\"Orientation\":\"";
    data+=Orientation;
    data+="\,";
    data+=Location;
    data+="}}";
    Serial.println(data);
    return data;
}

void callback(char* topic, byte* payload, int length)   // Here we can capture reply messages from Bluemix
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i=0;i<length;i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

boolean ParseLocation() // Sample data: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
// Refer to http://www.gpsinformation.org/dale/nmea.htm#GGA
{
    char Latitude[10];
    char Longitude[10];
    float Lat = gps.latitude;
    float Lon = gps.longitude;
    dtostrf(Lat,7,4,Latitude);
    dtostrf(Lon,7,4,Longitude);
    Location = "\"lat\":\"" + String(Latitude) + "\",\"lng\":\"" + String(Longitude)+"\"";
    Serial.println(Location);
}

void readAccelData(int * destination)   // Here is where we get acceleration on x,y,and z axes
{
    byte rawData[6];  // x/y/z accel register data stored here
    
    readRegisters(MMA8452_ADDRESS, 0x01, 6, &rawData[0]);  // Read the six raw data registers into data array
    
    // Loop to calculate 12-bit ADC and g value for each axis
    for (int i=0; i<6; i+=2)
    {
        destination[i/2] = ((rawData[i] << 8) | rawData[i+1]) >> 4;  // Turn the MSB and LSB into a 12-bit value
        if (rawData[i] > 0x7F)
        {
            // If the number is negative, we have to make it so manually (no 12-bit data type)
            destination[i/2] = ~destination[i/2] + 1;
            destination[i/2] *= -1;  // Transform into negative 2's complement #
        }
    }
}

void getData()      // This section will sample the thermister for a temp measurement
{
    // Temperature Data Section
    // I did not have a fancy temp sensor but I did have some 10k NTC 3950 Thermistors so I built a voltage divider
    // Followed this Adafruit leaning module - https://learn.adafruit.com/thermistor/testing-a-thermistor
    const int seriesResistor = 10000;    // the value of the 'other' resistor
    const int thermistorNominal = 8800;  // // resistance at 25 degrees C as measured by me
    const int temperatureNominal = 25;   // temp. for nominal resistance (almost always 25 C)
    const int numSamples = 5;  // how many samples to take and average, more takes longer - more is smoother
    const int bCoefficient = 3950; // The beta coefficient of the thermistor (usually 3000-4000)
    int samples[numSamples];
    uint8_t i;
    float average;
    digitalWrite(TempPowerPin, HIGH);
    for (i=0; i< numSamples; i++) {    // take N samples in a row, with a slight delay
        samples[i] = analogRead(ThermistorPin);
        NonBlockingDelay(10);
    }
    average = 0;  // average all the samples out
    for (i=0; i< numSamples; i++) {
        average += samples[i];
    }
    average /= numSamples;
    Serial.print("Average analog reading ");
    Serial.println(average);
    // convert the value to resistance
    average = (seriesResistor/average)*1023 - seriesResistor;
    Serial.print("Thermistor resistance ");
    Serial.println(average);
    float steinhart;
    steinhart = average / thermistorNominal;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= bCoefficient;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (temperatureNominal + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C
    Serial.print("Temperature ");
    Serial.print(steinhart);
    Serial.println(" *C");
    tempC = steinhart;
    digitalWrite(TempPowerPin, LOW); // Saves energy
}
boolean testConnectionToBluemix()       // Let's make sure we can get to Bluemix before proceeding
{
    Serial.print(MS_PROXY);
    Serial.print(F(" -> "));   // Try looking up the website's IP address
    while (ipaddr == 0) {
        if (! cc3000.getHostByName(MS_PROXY, &ipaddr)) {        // Need to add a timeout
            Serial.println(F("Couldn't resolve!"));
        }
        NonBlockingDelay(1000);  // give it a little more time
    }
    cc3000.printIPdotsRev(ipaddr);  // Success! Print restuls
    Serial.println("");
    return 1;
}

boolean GetConnected()  // Connection to Wifi
{
    
    Serial.print(F("\nAttempting to connect to ")); Serial.print(WLAN_SSID);
    if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
        Serial.println(": Failed!");
        return 0;
    }
    Serial.println(": Succeeded");
    return 1;
}

void displayMACAddress(void)  //Tries to read the 6-byte MAC address of the CC3000 module
{
    uint8_t macAddress[6];
    
    if(!cc3000.getMacAddress(macAddress))
    {
        Serial.println(F("Unable to retrieve MAC Address!\r\n"));
    }
    else
    {
        Serial.print(F("MAC Address : "));
        cc3000.printHex((byte*)&macAddress, 6);
    }
}

bool getIPAddress(void)  // Tries to read the IP address and other connection details
{
    unsigned long CommandTime = 0;
    uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
    Serial.print("Request DHCP: ");
    CommandTime = millis();
    while (!cc3000.checkDHCP())    // Wait for DHCP to complete
    {
        if (millis() >= CommandTime + TimeOut) {
            Serial.println("Failed");
            return 0;
        }
    }
    Serial.println("Succeeded");
    
    if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
    {
        Serial.println(F("Unable to retrieve the IP Address!"));
        return false;
    }
    else
    {
        Serial.print(F("IP Addr: ")); cc3000.printIPdotsRev(ipAddress);Serial.println("");
        Serial.print(F("Netmask: ")); cc3000.printIPdotsRev(netmask);Serial.println("");
        Serial.print(F("Gateway: ")); cc3000.printIPdotsRev(gateway);Serial.println("");
        Serial.print(F("DHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);Serial.println("");
        Serial.print(F("DNSserv: ")); cc3000.printIPdotsRev(dnsserv);Serial.println("");
        return true;
    }
    
}

// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
// Feel free to modify any values, these are settings that work well for me.
void initMMA8452(byte fsr, byte dataRate)   // Initialize the MMA8452 registers
{
    MMA8452Standby();  // Must be in standby to change registers
    // Set up the full scale range to 2, 4, or 8g.
    if ((fsr==2)||(fsr==4)||(fsr==8))
        writeRegister(MMA8452_ADDRESS, 0x0E, fsr >> 2);
    else
        writeRegister(MMA8452_ADDRESS,0x0E, 0);
    // Setup the 3 data rate bits, from 0 to 7
    writeRegister(MMA8452_ADDRESS, 0x2A, readRegister(MMA8452_ADDRESS,0x2A) & ~(0x38));
    if (dataRate <= 7)
        writeRegister(MMA8452_ADDRESS,0x2A, readRegister(MMA8452_ADDRESS,0x2A) | (dataRate << 3));
    // Set up portrait/landscap registers - 4 steps:
    // 1. Enable P/L
    // 2. Set the back/front angle trigger points (z-lock)
    // 3. Set the threshold/hysteresis angle
    // 4. Set the debouce rate
    // For more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
    writeRegister(MMA8452_ADDRESS,0x11, 0x40);  // 1. Enable P/L
    writeRegister(MMA8452_ADDRESS,0x13, 0x44);  // 2. 29deg z-lock (don't think this register is actually writable)
    writeRegister(MMA8452_ADDRESS,0x14, 0x84);  // 3. 45deg thresh, 14deg hyst (don't think this register is writable either)
    writeRegister(MMA8452_ADDRESS,0x12, 0x50);  // 4. debounce counter at 100ms (at 800 hz)
    // Set up single and double tap - 5 steps:
    // 1. Set up single and/or double tap detection on each axis individually.
    // 2. Set the threshold - minimum required acceleration to cause a tap.
    // 3. Set the time limit - the maximum time that a tap can be above the threshold
    // 4. Set the pulse latency - the minimum required time between one pulse and the next
    // 5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
    // for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf
    //writeRegister(0x21, 0x7F);  // 1. enable single/double taps on all axes
    writeRegister(MMA8452_ADDRESS, 0x21, 0x55);  // 1. single taps only on all axes
    // writeRegister(0x21, 0x6A);  // 1. double taps only on all axes
    writeRegister(MMA8452_ADDRESS, 0x23, Sensitivity);  // 2. x thresh at 2g (0x20), multiply the value by 0.0625g/LSB to get the threshold
    writeRegister(MMA8452_ADDRESS, 0x24, Sensitivity);  // 2. y thresh at 2g (0x20), multiply the value by 0.0625g/LSB to get the threshold
    writeRegister(MMA8452_ADDRESS, 0x25, Sensitivity);  // 2. z thresh at .5g (0x08), multiply the value by 0.0625g/LSB to get the threshold
    writeRegister(MMA8452_ADDRESS, 0x26, 0x30);  // 3. 30ms time limit at 800Hz odr, this is very dependent on data rate, see the app note
    writeRegister(MMA8452_ADDRESS, 0x27, 0xC8);  // 4. 1000ms (at 100Hz odr, Normal, and LPF Disabled) between taps min, this also depends on the data rate
    writeRegister(MMA8452_ADDRESS, 0x28, 0xFF);  // 5. 318ms (max value) between taps max
    // Set up interrupt 1 and 2
    writeRegister(MMA8452_ADDRESS, 0x2C, 0x02);  // Active high, push-pull interrupts
    writeRegister(MMA8452_ADDRESS, 0x2D, 0x19);  // DRDY, P/L and tap ints enabled
    writeRegister(MMA8452_ADDRESS, 0x2E, 0x01);  // DRDY on INT1, P/L and taps on INT2
    MMA8452Active();  // Set to active to start reading
}

void portraitLandscapeHandler() // Reads the p/l source register and prints what direction the sensor is now facing

{
    byte pl = readRegister(MMA8452_ADDRESS,0x10);  // Reads the PL_STATUS register
    switch((pl&0x06)>>1)  // Check on the LAPO[1:0] bits
    {
        case 0:
            Serial.print("On it's back");
            Orientation = "On it's back";
            break;
        case 1:
            Serial.print("On it's face");
            Orientation = "On it's face";
            break;
        case 2:
            Serial.print("Rightside Up");
            Orientation = "Rightside Up";
            if (pl&0x40) { // Check the LO bit
                Serial.print(", but tilted");
                Orientation += ", but tilted";
            }
            break;
        case 3:
            Serial.print("Upside Down");
            Orientation = "Upside Down";
            if (pl&0x40) {  // Check the LO bit
                Serial.print(", and tilted");
                Orientation += ", and tilted";
            }
            break;
    }
    /*
    if (pl&0x01)  // Check the BAFRO bit
        Serial.print("Back");
    else
        Serial.print("Front");
    if (pl&0x40)  // Check the LO bit
        Serial.print(", Z-tilt!");
     */
    Serial.println();
}


void MMA8452Standby()   // Sets the MMA8452 to standby mode to change most register settings
{
    byte c = readRegister(MMA8452_ADDRESS,0x2A);
    writeRegister(MMA8452_ADDRESS,0x2A, c & ~(0x01));
}


void MMA8452Active()// Sets the MMA8452 to active mode to output data
{
    byte c = readRegister(MMA8452_ADDRESS,0x2A);
    writeRegister(MMA8452_ADDRESS,0x2A, c | 0x01);
}



byte readRegister(int I2CAddress, byte address) // Read a single byte from address and return it as a byte
{
    //Send a request
    //Start talking to the device at the specified address
    Wire.beginTransmission(I2CAddress);
    //Send a bit asking for requested register address
    Wire.write(address);
    //Complete Transmission
    Wire.endTransmission(false);
    //Read the register from the device
    //Request 1 Byte from the specified address
    Wire.requestFrom(I2CAddress, 1);
    //wait for response
    while(Wire.available() == 0);
    // Get the temp and read it into a variable
    byte data = Wire.read();
    return data;
}


void readRegisters(int I2CAddress, byte address, int i, byte * dest)    // Read i registers sequentially, starting at address into the dest byte array
{
    //Send a request
    //Start talking to the device at the specified address
    Wire.beginTransmission(I2CAddress);
    //Send a bit asking for requested register address
    Wire.write(address);
    //Complete Transmission
    Wire.endTransmission(false);
    //Request i Bytes from the specified address
    Wire.requestFrom(I2CAddress, i);
    //wait for response
    while(Wire.available() == 0);
    //Read the register from the device    for (int j=0; j<i; j++)
    for (int j=0; j<i; j++)
    {
        dest[j] = Wire.read();
    }
}

void writeRegister(int I2CAddress, unsigned char address, unsigned char data)   // Writes a single byte (data) into address
{
    //Send a request
    //Start talking to the device at the specified address
    Wire.beginTransmission(I2CAddress);
    //Send a bit asking for requested register address
    Wire.write(address);
    Wire.write(data);
    //Complete Transmission
    Wire.endTransmission(false);
}

void BlinkForever() // When something goes badly wrong...
{
    Serial.println(F("Error - Reboot"));
    while(1) {
        digitalWrite(ledRED,HIGH);
        delay(200);
        digitalWrite(ledRED,LOW);
        delay(200);
    }
}

void NonBlockingDelay(int millisDelay)  // Used for a non-blocking delay
{
    unsigned long commandTime = millis();
    while (millis() <= millisDelay + commandTime) { }
    return;
}


/*
 *************************************************************************************************************
 
 // Set parameters
 // Set up the WiFi Module
 #define CC3000_IRQ   2  // MUST be an interrupt pin!
 #define CC3000_VBAT  5  // These can be any two pins
 #define CC3000_CS    10 // Use hardware SPI for the remaining pins
 
 
 #define MOBILE 0
 #if MOBILE
 #define WLAN_SSID       "iPhone"           // Personal LTE Hotspot
 #define WLAN_PASS       "usna1986"
 #else
 #define WLAN_SSID       "JKJ Wi-Fi Network"  // cannot be longer than 32 characters!
 #define WLAN_PASS       "usna1986"
 #endif
 #define WLAN_SECURITY   WLAN_SEC_WPA2 // Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
 #define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data
 
 // Set up the Accelerometer
 const int MMA8452_ADDRESS = 0x1D; // The SparkFun MMA8452 breakout board defaults to 1 for an address of 0x1D
 const byte SCALE = 2;  // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
 const byte dataRate = 3;  // output data rate - 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
 const int int1Pin = 4;         // Not used now but wired for future use
 const int int2Pin = 3;         // This is the interrupt pin that registers taps
 
 // Set up the TrimPots and Indicator Leds
 const int TrimPot = A0;        // Potentiometer used to adjust sensitivity
 const int TrimPot1 = A1;       // Potentiometer used to adjust delay
 const int ledPin = 16;         // Tap Indicator LED
 const int ledPin1 = 17;         // TBD
 
 // Include application, user and local libraries
 #include <Adafruit_CC3000.h>
 #include <ccspi.h>
 #include <SPI.h>
 #include <string.h>
 #include "utility/debug.h"
 #include <stdlib.h>
 #include <GPS.h>      // Code and Library from: https://github.com/rvnash/ultimate_gps_teensy3
 //#include <WProgram.h>
 #include <i2c_t3.h>
 
 
 // Define structures and classes
 Adafruit_CC3000 cc3000 = Adafruit_CC3000(CC3000_CS, CC3000_IRQ, CC3000_VBAT, SPI_CLOCK_DIVIDER); // you can change this clock speed
 
 // Define variables and constants
 const int LEDpin = 13;
 // Set up the Ubidots Account and Variables
 #define WEBSITE      "things.ubidots.com"
 String token = "6EALvOEabrFiA6XViWY3hxcFMG0yDr";  // For your account
 String idvariable = "54f5201b7625426e88aff410";  // For your variable
 
 // Global Variables
 uint32_t ip;
 int value = 0;
 unsigned int ReportingInterval = 10000;  // How often do you want to send to Ubidots (in millis)
 unsigned long LastReport = 0;            // Keep track of when we last sent data
 int TimeOut = 4000;                      // How long will we wait for a command to complete
 char c;                                  // Used to relay input from the GPS serial feed
 String Location = "";                    // Will build the Location string here
 int InputValue = 0;            // Raw sensitivity input
 byte Sensitivity = 0x00;       // Hex variable for sensitivity
 static byte source;
 int ledState = LOW;            // variable used to store the last LED status, to toggle the light
 unsigned long LastKnock = 0;   // Used for debouncing
 int Debounce = 1000;           // Time in ms between knocks
 
 // Prototypes
 boolean Send2Ubidots(String value);  // Here is where we send the information to Ubidots
 boolean GetConnected();  // Connection to Wifi
 boolean GetDisconnected();   // Disconnect from Wifi
 boolean displayConnectionDetails(void);  // WiFi Connection Details
 boolean ParseLocation(); // Sample data: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
 void initMMA8452(byte fsr, byte dataRate);   // Initialize the MMA8452 registers
 void MMA8452Standby();  // Sets the MMA8452 to standby mode to change most register settings
 void MMA8452Active();   // Sets the MMA8452 to active mode to output data
 byte readRegister(int I2CAddress, byte address); // Read a single byte from address and return it as a byte
 void writeRegister(int I2CAddress, unsigned char address, unsigned char data);  // Writes a single byte (data) into address
 
 
 // Set up the GPS using Hardware Serial1 - Pins 0 and 1
 HardwareSerial &gpsSerial = Serial1;
 GPS gps(&gpsSerial,true);
 
 // Add setup code
 void setup()
 {
 // Initialize Serial ports
 Serial.begin(19200);  // Start serial terminal
 delay(2000);
 Serial.println();
 Serial.println(F("---------------------------"));
 Serial.println(F("Connected Location Logger  "));
 Serial.println(F("---------------------------"));
 gps.startSerial(9600);  // Start serial1 for GPS
 gps.setSentencesToReceive(OUTPUT_RMC_GGA);  // Sets the format for the GPS sentences
 Wire.begin(); // Start serial on i2c
 
 // Initialize the Accelerometer
 pinMode(int2Pin, INPUT);     // Set up the Accelerometer interrupt pins, they're set as active high, push-pull
 digitalWrite(int2Pin, HIGH);
 pinMode(int1Pin, INPUT);
 digitalWrite(int2Pin, HIGH);
 Serial.print("Initializing the accelerometer: ");
 byte c = readRegister(MMA8452_ADDRESS, 0x0D);            // Read WHO_AM_I register to test communications
 if (c == 0x2A) {                  // WHO_AM_I should always be 0x2A
 initMMA8452(SCALE, dataRate);   // init the accelerometer if communication is OK
 Serial.println("Succeeded");
 }
 else {                          // Problem with communucations
 Serial.print("Failed at address: 0x");
 Serial.println(c, HEX);
 while(1);                   // Loop forever if communication doesn't happen
 }
 // Initialise the Wi-Fi module
 Serial.print("Initializing CC3000 Wi-Fi: ");
 if (!cc3000.begin())
 {
 Serial.println(F("Failed"));
 while(1);
 }
 Serial.println("Succeeded");
 GetConnected();             // Test connection to Internet
 Serial.print(WEBSITE);
 Serial.print(F(" -> "));   // Try looking up the website's IP address
 while (ip == 0) {
 if (! cc3000.getHostByName(WEBSITE, &ip)) {
 Serial.println(F("Couldn't resolve!"));
 }
 delay(1000);  // give it a little more time
 }
 cc3000.printIPdotsRev(ip);  // Success! Print restuls
 while (! displayConnectionDetails()) {  // Display the IP address DNS, Gateway, etc.
 delay(1000);
 }
 // GetDisconnected();  // If you want to save batteries - let the logger disconnect (two comments below)
 Serial.println("Location Logger is on-line any movement will be logged");
 }
 
 // Add loop code
 void loop()
 {
 if (digitalRead(int2Pin)==1){    // If int2 goes high, either p/l has changed or there's been a single/double tap
 source = readRegister(MMA8452_ADDRESS, 0x0C);  // Read the interrupt source reg.
 if ((source & 0x08)==0x08) { // We are only interested in the TAP register so read that
 if (millis() >= (LastKnock + Debounce)) {    // Need to debounce the knocks
 Serial.println("movement detected...");
 ledState = !ledState;                        // toggle the status of the ledPin:
 digitalWrite(ledPin, ledState);              // update the LED pin itself
 LastKnock = millis();
 if (millis() >= LastReport + ReportingInterval) {
 if (gps.sentenceAvailable()) gps.parseSentence();
 if (gps.newValuesSinceDataRead()) {
 gps.dataRead();
 Serial.println("logging");
 }
 //GetConnected();
 if(Send2Ubidots(String(value))) {
 Serial.println(F("Data successfully sent to Ubidots"));
 LastReport = millis();  // Reset the timer
 value++;
 }
 else {
 Serial.println(F("Data not accepted by Ubidots - try again"));
 }
 //GetDisconnected();
 }
 Serial.print("Looking for movement...");
 }
 byte source = readRegister(MMA8452_ADDRESS, 0x22);  // Reads the PULSE_SRC register to reset it
 }
 }
 }
 
 
 boolean Send2Ubidots(String value)  // Here is where we send the information to Ubidots
 
 {
 char replybuffer[64];          // this is a large buffer for replies
 int count = 0;
 int complete = 0;
 String var = "";
 String le = "";
 ParseLocation();              // Update the location value from the GPS feed
 var="{\"value\":"+ value + ", \"context\":"+ Location + "}";
 //  var="{\"value\":"+ value + "}";
 int num=var.length();                                       // How long is the payload
 le=String(num);                                             //this is to calcule the length of var
 // Make a TCP connection to remote host
 Serial.println(F("Sending Data to Ubidots"));
 Adafruit_CC3000_Client client = cc3000.connectTCP(ip, 80);
 if (!client.connected()) {
 Serial.println(F("Error: Could not make a TCP connection"));
 }
 // Make a HTTP GET request
 client.fastrprint(F("POST /api/v1.6/variables/"));
 client.print(idvariable);
 client.fastrprintln(F("/values HTTP/1.1"));
 client.fastrprintln(F("Content-Type: application/json"));
 client.fastrprint(F("Content-Length: "));
 client.println(le);
 client.fastrprint(F("X-Auth-Token: "));
 client.println(token);
 client.fastrprintln(F("Host: things.ubidots.com"));
 client.println();
 client.println(var);
 client.println();
 
 // See if Ubidots acknowledges the creation of a new "dot" with a "201" code
 unsigned long commandClock = millis();                      // Start the timeout clock
 while(!complete && millis() <= commandClock + TimeOut)         // Need to give the modem time to complete command
 {
 while(!client.available() &&  millis() <= commandClock + TimeOut);  // Keep checking to see if we have data coming in
 while (client.available()) {
 replybuffer[count] = client.read();
 count++;
 if(count==63) break;
 }
 Serial.print(F("Reply: "));
 for (int i=0; i < count; i++) {
 if (replybuffer[i] != '\n') Serial.write(replybuffer[i]);
 }
 Serial.println("");                           // Uncomment if needed to debug
 for (int i=0; i < count; i++) {
 if(replybuffer[i]=='2' && replybuffer[i+1]=='0' && replybuffer[i+2] == '1') {  // here is where we parse "201"
 complete = 1;
 break;
 }
 }
 }
 unsigned long CommandTime = millis();
 Serial.print("Closing the socket: ");
 while (!client.close()) {   // Close socket
 if (millis() >= CommandTime + TimeOut) {
 Serial.println("Failed");
 }
 }
 Serial.println("Succeeded");
 if (complete ==1) return 1;            // Returns "True"  if we get the 201 response
 else return 0;
 }
 
 
 boolean GetDisconnected()   // Disconnect from Wifi
 {
 unsigned long CommandTime = millis();
 Serial.print("Disconnecting: ");
 while (!cc3000.disconnect() ) {    // Disconnect WiFi
 if (millis() >= CommandTime + TimeOut) {
 Serial.println("Failed");
 return 0;
 }
 }
 Serial.println("Succeeded");
 return 1;
 }
 
 
 
 boolean GetConnected()  // Connection to Wifi
 {
 unsigned long CommandTime = 0;
 Serial.print(F("\nAttempting to connect to ")); Serial.print(WLAN_SSID);
 if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
 Serial.println(": Failed!");
 return 0;
 }
 Serial.println(": Succeeded");
 Serial.print("Request DHCP: ");
 CommandTime = millis();
 while (!cc3000.checkDHCP())    // Wait for DHCP to complete
 {
 if (millis() >= CommandTime + TimeOut) {
 Serial.println("Failed");
 return 0;
 }
 }
 Serial.println("Succeeded");
 return 1;
 }
 
 boolean displayConnectionDetails(void)  // WiFi Connection Details
 {
 uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
 
 if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
 {
 Serial.println(F("Unable to retrieve the IP Address!\r\n"));
 return false;
 }
 else
 {
 Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
 Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
 Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
 Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
 Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
 Serial.println();
 return true;
 }
 }
 
 
 boolean ParseLocation() // Sample data: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
 // Refer to http://www.gpsinformation.org/dale/nmea.htm#GGA
 {
 char Latitude[10];
 char Longitude[10];
 float Lat = gps.latitude;
 float Lon = gps.longitude;
 dtostrf(Lat,7,4,Latitude);
 dtostrf(Lon,7,4,Longitude);
 Location = "{\"lat\":" + String(Latitude) + ",\"lng\":" + String(Longitude) + "}";
 }
 
 
 
 */
