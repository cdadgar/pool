/*
 * module is a esp-12  (Generic ESP8266 Module)
 * flash size set to 4MB (FS:1MB OTA:~1019KB)
 */

/*
 * todo:
 *  - add ota
 *  - add mqtt
 */

/*
 * library sources:
 * ESP8266WiFi, ESP8266WebServer, FS, DNSServer, Hash, EEPROM, ArduinoOTA - https://github.com/esp8266/Arduino
 * WebSocketsServer - https://github.com/Links2004/arduinoWebSockets (git)
 * WiFiManager - https://github.com/tzapu/WiFiManager (git)
 * ESPAsyncTCP - https://github.com/me-no-dev/ESPAsyncTCP (git)
 * ESPAsyncUDP - https://github.com/me-no-dev/ESPAsyncUDP (git)
 * OneWire - https://github.com/PaulStoffregen/OneWire (git)
 * DallasTemperature - https://github.com/milesburton/Arduino-Temperature-Control-Library (git)
 * PubSub - https://github.com/knolleary/pubsubclient (git)
 * TimeLib - https://github.com/PaulStoffregen/Time (git)
 * Timezone - https://github.com/JChristensen/Timezone (git)
 * ArduinoJson - https://github.com/bblanchon/ArduinoJson  (git)
 * LiquidCrystal_I2C - https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library (git)
 */
 
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <TimeLib.h> 
#include <Timezone.h>

//US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

// --------------------------------------------

// web server library includes
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <EEPROM.h>


// --------------------------------------------

// file system (spiffs) library includes
#include <FS.h>

// --------------------------------------------

// wifi manager library includes
#include <DNSServer.h>
#include <WiFiManager.h>

WiFiManager wifiManager;
String ssid;

// --------------------------------------------

// aync library includes
#include <ESPAsyncTCP.h>
#include <ESPAsyncUDP.h>

// --------------------------------------------

// mqtt library includes
#include <PubSubClient.h>

// --------------------------------------------

// arduino ota library includes
#include <ArduinoOTA.h>

#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
ESP8266HTTPUpdateServer httpUpdater;

// --------------------------------------------

// display includes
#include <LiquidCrystal_I2C.h>

// --------------------------------------------

// temperature sensor includes
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --------------------------------------------

const char *weekdayNames[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

// --------------------------------------------

#define LOGGING_IP_ADDR "192.168.1.210"
#define LOGGING_IP_PORT 80
#define LOG_URL "http://cpd.us.to:90/pool/"

// --------------------------------------------


/*
note: had to modify LiquidCrystal_I2C.cpp and remove Wire.begin()
so that we can reconfigure the i2c pins
could have also used Wire.pins(sda,scl), but it has been deprecated

the 16x2 display needs to run on 5v so that the brightness is visible.
all the the other lines work at 3.3V
*/

#define SDA 0
#define SCL 2

// --------------------------------------------

#define PUMP_BUTTON  12
#define SOLAR_BUTTON 13
#define UP_BUTTON    16    // special case...we are providing our own pullup resistor
#define DOWN_BUTTON   5

const int buttonPins[] = {PUMP_BUTTON, SOLAR_BUTTON, UP_BUTTON, DOWN_BUTTON};
const int defaultButtonState = HIGH;

int numButtons;
int *buttonStates;
int *lastButtonStates;

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long *lastDebounceTimes;  // the last time the output pin was toggled
const long debounceDelay = 50;    // the debounce time

// --------------------------------------------

/*
 * esp-12 pin usage: 
 * gpio5 - down button
 * gpio4 - 1-wire - temp sensor
 * gpio0 - i2c - sda
 * gpio2 - i2c - scl
 * gpio15 - solar value      use this for the solar value?     (note: this pin can't be used as an input....it can only be used as an output) 
 * gpio16 - up button (note: on startup, this line starts out high before config takes it low, so it can't be used for the pump or solar)
 *            is also didn't work for an input button.  it read 1 until it was pressed, then read 0 after pressing, but didn't
 *            read 1 after being released.   check web for solutions
 *            16 doesn't have internal pullup resistor support
 *            it has internal pulldown resistor.   can it be used as normal input, and then I provide my own pull up resistor? use 10K ohn
 * gpio14 - pump ssr
 * gpio12 - pump mode button
 * gpio13 - solar mode button
 */

// --------------------------------------------

// gpio 4 is available on the esp-12, but it blinks the blue led when used....weird
// so use gpio16 instead...tried this, and didn't work...is gpio 1 not usable?
#define ONE_WIRE_BUS 4

#define PUMP_GPIO    14
#define SOLAR_GPIO   15

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// device address
DeviceAddress thermometer[3];
int numDevices;

#define TEMP_ERROR -999.0
#define resolution 12
float lastTemp[3];
float lastPressure = -1;
long firstMinPressureTime = 0;
long firstMaxPressureTime = 0;
unsigned long lastTempRequest = 0;
int delayInMillis = 0;

// --------------------------------------------

#define MenuIdleTime                 10000

LiquidCrystal_I2C lcd(0x27,16,2);
bool isSetup = false;
bool isDisplayOn;
unsigned long lastButtonPressTime;
unsigned long lastMinutes;
unsigned long lastSeconds;

enum screenState { MAIN, MENU };
screenState screen;

#define ON    0
#define OFF   1
#define AUTO  2
const char *modeNames[] = { "On  ", "Off ", "Auto" };

// actions
#define PUMP_OFF          'p'
#define PUMP_ON           'P'
#define SOLAR_OFF         's'
#define SOLAR_ON          'S'

typedef struct {
  byte pumpStart;
  byte pumpStop;
  byte targetTemp;
  byte pressureLow;
  byte pressureHigh;
} programType;

programType program;

ESP8266WebServer server(80);
AsyncServer* aserver;
AsyncClient* clientClients[10];
int numClientClients;
#define ASYNC_PORT 9090

File fsUploadFile;

WebSocketsServer webSocket = WebSocketsServer(81);
int webClient = -1;
int programClient = -1;
int setupClient = -1;
int testClient = -1;

unsigned int testHeap = 0;

bool isTimeSet = false;

WiFiClient espClient;
PubSubClient client(espClient);

#define HOST_NAME "POOL"
#define MQTT_IP_ADDR "192.168.1.210"
#define MQTT_IP_PORT 1883

bool isPromModified;
bool isMemoryReset = false;
//bool isMemoryReset = true;

bool isPumpOn = false;
bool isSolarOn = false;

typedef struct {
  char host_name[17];
  char mqtt_ip_addr[17];
  int mqtt_ip_port;
  byte use_mqtt;
  byte pumpMode;
  byte solarMode;
  byte sensorId[3];
  byte temperature_span;
  byte display_timeout;
  byte use_logging;
  char logging_ip_addr[17];
  int  logging_ip_port;
  char log_url[40];
} configType;

configType config;


#define POOL 0
#define ROOF 1
#define AIR  2
const char *tempNames[] = { "poolTemp", "roofTemp", "airTemp" }; 


void print(const char *str);
void println(const char *str);
void setupDisplay(void);
void setupButtons(void);
void loadConfig(void);
void loadProgramConfig(void);
bool setupWifi(void);
void setupTime(void);
bool setupTempSensor(void);
void setupPressureSensor(void);
void setupPump(void);
void setupSolar(void);
void setupWebServer(void);
void setupAsyncServer(void);
void setupClients(void);
void addClient(AsyncClient* c);
void removeClient(AsyncClient* c);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void drawMainScreen(void);
//void set(char *name, const char *value);
void saveProgramConfig(void);
unsigned long sendNTPpacket(IPAddress& address);
void printTime(bool isCheckProgram, bool isDisplay, bool isTest);
void displayBacklight(bool);
void printPumpModeState(bool);
void printSolarModeState(bool);
void printPumpState(bool);
void printSolarState(bool);
void printTemperature(bool, int);
void printTargetTemperature(bool);
void printPressure(bool);
void sendWeb(const char *command, const char *value);
void checkTimeMinutes(void);
void flashTime(unsigned long time);
void checkButtons(void);
void eraseTime(void);
void doButton(int pin);
void doUpButton(void);
void doDownButton(void);
void doSolarButton(void);
void doPumpButton(void);
void saveConfig(void);
void update(int addr, byte data);
void logTemperature(float air, float roof, float pool);
void logAction(char action);
void logTarget(int temperature);
void checkTemperature(int index);
void checkTemperature(unsigned long time);
void checkTemperature(DeviceAddress deviceAddress);
void checkPump(void);
void checkSolar(void);
void checkPressure(void);
void pump(bool isOn);
void solar(bool isOn);
void sendAirTemperature(float temp);
void pumpModeChange(void);
void solarModeChange(void);


void setup() {
  // start serial port
  Serial.begin(115200);
  Serial.print(F("\n\n"));

  Wire.begin(SDA,SCL);

  Serial.println(F("esp8266 pool"));
  Serial.println(F("compiled:"));
  Serial.print( __DATE__);
  Serial.print(F(","));
  Serial.println( __TIME__);

  setupDisplay();
  setupButtons();

  if (!setupWifi())
    return;
    
  // must specify amount of eeprom to use (max is 4k?)
  EEPROM.begin(512);
  
  loadConfig();
  loadProgramConfig();
  isMemoryReset = false;

  if (!setupTempSensor())
    return;

  setupPressureSensor();
  setupPump();
  setupSolar();

  setupTime();
  setupWebServer();
  setupAsyncServer();
  setupMqtt();
  setupOta();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
    
  lastMinutes = 0;
  lastSeconds = 0;

  isSetup = true;
  drawMainScreen();
}


void setupAsyncServer() {
  setupClients();
  
  aserver = new AsyncServer( ASYNC_PORT );

  aserver->onClient([](void *obj, AsyncClient* c) {
    Serial.printf("[A-TCP] onClient\n");
    addClient(c);

    c->onDisconnect([](void *obj, AsyncClient* c) {
      Serial.printf("[A-TCP] onDisconnect\n");
      free(c);
      removeClient(c);
    }, NULL);

    c->onError([](void *obj, AsyncClient* c, int8_t err) {
      Serial.printf("[A-TCP] onError: %s\n", c->errorToString(err));
    }, NULL);
  }, aserver);

  Serial.printf("starting socket server\n");
  aserver->begin();
}


void setupClients(void) {
  for(int i=0; i<10; ++i)
    clientClients[i] = NULL;
  numClientClients = 0;
}


void sendAirTemperature(float temp) {
  if (numClientClients == 0)
    return;

  Serial.print("sending air temp ");
  Serial.print(temp);
  Serial.print(" to ");
  Serial.print(numClientClients);
  Serial.println(" clients");

  char buf[8];
  dtostrf(temp, 4, 1, buf);

  for (int i=0; i < numClientClients; ++i) {
    clientClients[i]->write(buf);
//    clientClients[i]->write((const char *)&temp, sizeof(float));
  }
}


void addClient(AsyncClient* c) {
  if (numClientClients == 10) {
    Serial.printf("ERROR: too many temperature clients\n");
    return;
  }
  clientClients[numClientClients] = c;
  ++numClientClients;

  // send the current air temp to the new client
  char buf[8];
  dtostrf(lastTemp[AIR], 4, 1, buf);
  c->write(buf);
//  c->write((const char *)&lastTemp[AIR], sizeof(float));
}


void removeClient(AsyncClient* c) {
  // look for a temperature client to disconnect
  // shift everyone after down
  bool isFound = false;
  for (int i=0; i < numClientClients; ++i) {
    if (clientClients[i] == c)
      isFound = true;
    if (isFound)
      clientClients[i] = clientClients[i+1];         
  }
  if (isFound) {
    clientClients[numClientClients] = NULL;         
    --numClientClients;
  }
}


void initProgram(void) {
/*
pool timer

start time:   10:00am
stop time:     4:00pm

time starts at 12am = 0, and goes in 15 minute increments
10:00am = 4 * 10 = 40
 4:00pm = 4 * 16 = 64

target temperature: 90 degrees

low pressure: 10 psi
  under this, and we'll shut off the pump since we think the water level is too low
high pressure: 40 psi
  above this and we pool filter needs to be cleaned
*/
  program.pumpStart = 40;
  program.pumpStop  = 64;
  program.targetTemp = 90;
  program.pressureLow  = 10;
  program.pressureHigh = 40;

  saveProgramConfig(); 
}


void configModeCallback(WiFiManager *myWiFiManager) {
  // this callback gets called when the enter AP mode, and the users
  // need to connect to us in order to configure the wifi
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Join:"));
  lcd.print(config.host_name);
  lcd.setCursor(0,1);
  lcd.print(F("Goto:"));
  lcd.print(WiFi.softAPIP());
}


bool setupWifi(void) {
  WiFi.hostname(config.host_name);
  
//  wifiManager.setDebugOutput(false);
  
  //reset settings - for testing
  //wifiManager.resetSettings();

  ssid = WiFi.SSID();
  if (ssid.length() > 0) {
    Serial.print(F("Connecting to "));
    Serial.println(ssid);
    lcd.setCursor(0,0);
    lcd.print(F("Connecting to:"));
    lcd.setCursor(0,1);
    lcd.print(ssid);
  }
  
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  if(!wifiManager.autoConnect(config.host_name)) {
    Serial.println(F("failed to connect and hit timeout"));
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  } 

  return true;
}


void setupTime(void) {
  Serial.println(F("Getting time"));

  AsyncUDP* udp = new AsyncUDP();

  // time.nist.gov NTP server
  // NTP requests are to port 123
  if (udp->connect(IPAddress(129,6,15,28), 123)) {
//    Serial.println("UDP connected");
    
    udp->onPacket([](void *arg, AsyncUDPPacket packet) {
//      Serial.println(F("received NTP packet"));
      byte *buf = packet.data();
      
      //the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, esxtract the two words:
    
      // convert four bytes starting at location 40 to a long integer
      unsigned long secsSince1900 =  (unsigned long)buf[40] << 24;
      secsSince1900 |= (unsigned long)buf[41] << 16;
      secsSince1900 |= (unsigned long)buf[42] << 8;
      secsSince1900 |= (unsigned long)buf[43];
      time_t utc = secsSince1900 - 2208988800UL;
    
      TimeChangeRule *tcr;
      time_t local = myTZ.toLocal(utc, &tcr);
      Serial.printf("\ntime zone %s\n", tcr->abbrev);
    
      setTime(local);
    
      // just print out the time
      printTime(false, false, true);
    
      isTimeSet = true;

      free(arg);
    }, udp);
    
//    Serial.println(F("sending NTP packet"));

    const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
    byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold outgoing packet

    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;
    
    // all NTP fields have been given values, now
    // send a packet requesting a timestamp:
    udp->write(packetBuffer, NTP_PACKET_SIZE);
  }
  else {
    free(udp);
    Serial.println(F("\nWiFi:time failed...will retry in a minute"));
  }
}


void setupDisplay(void) {
  // initialize the lcd 
  lcd.begin();
  lcd.clear();

  displayBacklight(true);
}


void setupButtons(void) {
  numButtons = sizeof(buttonPins)/sizeof(buttonPins[0]);
  buttonStates = new int[numButtons];
  lastButtonStates = new int[numButtons];
  lastDebounceTimes = new long[numButtons];

  for (int i=0; i < numButtons; ++i) {
    if (buttonPins[i] == 16)
      // gpio 16 doesn't support internal pullup..only internal pulldown or normal input
      pinMode(buttonPins[i], INPUT);
    else
      pinMode(buttonPins[i], INPUT_PULLUP);

    buttonStates[i] = defaultButtonState;
    lastButtonStates[i] = defaultButtonState;
    lastDebounceTimes[i] = 0;
  }
}


bool setupTempSensor(void) {
  // locate devices on the bus
  Serial.print(F("temp sensor: "));
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("temp sensor:"));

  sensors.begin();
  numDevices = sensors.getDeviceCount();
  Serial.print(numDevices);
  Serial.println(F(" found"));
  lcd.setCursor(0,1);
  lcd.print(numDevices);
  lcd.print(F(" found"));
  if (numDevices != 3)
    return false;

  for (int i=0; i < 3; ++i) {
    if (!sensors.getAddress(thermometer[i], i))
      return false;
    sensors.setResolution(thermometer[i], resolution);
  }
  
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  delayInMillis = 750 / (1 << (12 - resolution)); 
  lastTempRequest = millis(); 

  return true;
}


void setupPump(void) {
  /*
   * the pump ssr has a 200 ohm resistor, or uses 17mA
   * a gpio can drive up to 12mA or sink up to 20mA
   * so, we are sinking the ssr
   * HIGH is off, LOW is ON
   */
  pinMode(PUMP_GPIO, OUTPUT);
  digitalWrite(PUMP_GPIO, HIGH);
}


void setupSolar(void) {
  /*
   * the solar gpoi: 15
   * can't be setup to sink current, or else we can't program the esp8266
   * so, we gotta use it as a source, which limits us to 12mA
   */
  pinMode(SOLAR_GPIO, OUTPUT);
  digitalWrite(SOLAR_GPIO, LOW);
}


void setupPressureSensor(void) {
  /* 
   * enable the adc as an input 
   * 
   * it allows voltage from 0 to 1.0 volts
   * the pressure sensor outputs 0.5v at 0 psi
   * and 4.5v at 80psi
   * the psi change should give linear voltage changes
   * 
   * the voltage divider works this way:
   * Vout = Vin * R2 / (R1 + R2)
   * R2 is 1k ohm + 500 ohm = 1.5k ohm
   * R1 is 4.7k ohm
   * 
   * 0 psi = .5v => .12v at the adc
   * 80psi = 4.5v => 1.09v at the adc (we should never see this high of a pressure,
   * so we won't exceed the 1v max at the adc)
   * 
   * 4v/80psi = 0.05v/psi => 0.0121v/psi at the adc
   * 
   * the adc reads an unsigned int from 0 to 1024
   * value / 1024 = volts
   * (volts - .12) / 0.0121 = psi
   */
  pinMode(A0, INPUT);
}


void drawMainScreen(void) {
  screen = MAIN;

  lcd.clear();
  
  printTime(true, true, false);
  printPumpModeState(true);
  printPumpState(true);
  printSolarModeState(true);
  printSolarState(true);
  for (int i=0; i < 3; ++i)
    printTemperature(true, i);
  printTargetTemperature(true);
  printPressure(true);
  
  checkPump();
}


void printPumpModeState(bool isDisplay) {
  if (webClient != -1) {
    char buf[3];
    sprintf(buf, "%d", config.pumpMode);    
    sendWeb("pumpMode", buf);
  }
}


void printSolarModeState(bool isDisplay) {
  if (webClient != -1) {
    char buf[3];
    sprintf(buf, "%d", config.solarMode);    
    sendWeb("solarMode", buf);
  }
}


void printPumpState(bool isDisplay) {
  if (isDisplay && screen == MAIN) {
    lcd.setCursor(0,0);
    if (isPumpOn)
      lcd.print("On  ");
    else
      lcd.print(modeNames[config.pumpMode]);
  }
  
  if (webClient != -1) {
    sendWeb("pump", isPumpOn ? "On" : "Off");
  }
}


void printSolarState(bool isDisplay) {
  if (isDisplay && screen == MAIN) {
    lcd.setCursor(5,0);
    if (isSolarOn)
      lcd.print("On  ");
    else
      lcd.print(modeNames[config.solarMode]);
  }
  
  if (webClient != -1) {
    sendWeb("solar", isSolarOn ? "On" : "Off");
  }
}


void printTemperature(bool isDisplay, int index) {
  if (lastTemp[index] == TEMP_ERROR)
    return;
    
  char buf[8];
  dtostrf(lastTemp[index], 4, 1, buf);

//  Serial.printf("temp %d %s\n", index, buf);

  if (isDisplay && screen == MAIN) {
    // only dispay the pool temperature on the display
    if (index == POOL) {
      lcd.setCursor(3, 1);
      lcd.print(buf);
    }
  }
  
  if (webClient != -1) {
    sendWeb(tempNames[index], buf);  
  }

  // send out the air temperature to connected clients
  if (index == AIR)
    sendAirTemperature(lastTemp[index]);
}


void setTargetTemp(int newTemp) {
  if (config.solarMode != AUTO)
    return;

  // not saved as new program temp 
  program.targetTemp = newTemp;
  logTarget(program.targetTemp);
  printTargetTemperature(true);
  checkPump();
}


void printTargetTemperature(bool isDisplay) {
//  Serial.printf("target temp %d\n", program.targetTemp);

  if (isDisplay && screen == MAIN) {
    lcd.setCursor(0, 1);
    if (config.solarMode != AUTO)
      lcd.print("  ");
    else
      lcd.print(program.targetTemp);
  }
  
  if (webClient != -1) {
    if (config.solarMode != AUTO)
      sendWeb("targetTemp", "");  
    else {
      char buf[8];
      sprintf(buf, "%d", program.targetTemp); 
      sendWeb("targetTemp", buf);
    }  
  }
}


void printPressure(bool isDisplay) {
  char buf[8];
  dtostrf(lastPressure, 4, 1, buf);
  
  if (isDisplay && screen == MAIN) {
    lcd.setCursor(10, 0);
    lcd.print(buf);
  }

  if (webClient != -1) {
    sendWeb("pressure", buf);  
  }
}


void loop(void)
{
  if (!isSetup)
    return;
   
  unsigned long time = millis();

  if (screen == MAIN) {
    // turn off the display if idle
    if (isDisplayOn && (time - lastButtonPressTime > (config.display_timeout*1000)) ) {
      displayBacklight(false);
    }

    checkTimeMinutes();
    checkTemperature(time);

    // flash the time if its not set
    if (!isTimeSet)
      flashTime(time);
  }
  else {
    // go back to main screen if idle
    if (time - lastButtonPressTime > MenuIdleTime ) {
      drawMainScreen();
    }
  }

  checkButtons();

  webSocket.loop();
  server.handleClient();
  MDNS.update();

  // mqtt
  if (config.use_mqtt) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  }

  ArduinoOTA.handle();
}


void checkTemperature(unsigned long time) {
  if (time - lastTempRequest >= delayInMillis) // waited long enough??
  {
//    Serial.println("checking temperatures");
    for (int i=0; i < 3; ++i)
      checkTemperature(i);

    sensors.requestTemperatures(); 
    lastTempRequest = millis();

    // now that we have new temperature readings,
    // check to see of the solar needs to be turned on or off
    checkSolar();

    // read the pressure sensor here as well
    checkPressure();
  }
}


void checkPressure(void) {
  /*  
   * see setupPressureSensor for calculation details
   * the adc reads an unsigned int from 0 to 1024
   * value / 1024 = volts
   * (volts - .12) / 0.0121 = psi
   */
  unsigned int adc = (unsigned int) analogRead(A0);
  Serial.print("read adc ");
  Serial.print(adc);

  double vadc = (double)adc / 1024.0;

  Serial.print("    v ");
  Serial.print(vadc);

  float psi = (vadc - .12) / 0.0121;

  Serial.print("    psi ");
  Serial.println(psi);

  if (lastPressure != psi) {
    lastPressure = psi;
    printPressure(true);
  }
  
  // check for min and max pressure
  // if the pump is running, the pressure should be above the min pressure
  if (!isPumpOn)
    return;
      
  if (psi < program.pressureLow) {
    if (firstMinPressureTime == 0) {
      firstMinPressureTime = millis();
      Serial.println("low pressure start");
    }
    else {
      // if the pressure error has been happening long enough, take action
      if ((millis() - firstMinPressureTime) > 30*1000) {
        // shutdown down the pump to avoid damaging it
        // cpd...not done
        Serial.println("low pressure for too long");
      }
    }
  }

  // if the pressure is above the max pressure, the filter needs to be cleaned
  else if (psi > program.pressureHigh) {
    if (firstMaxPressureTime == 0) {
      firstMaxPressureTime = millis();
      Serial.println("high pressure start");
    }
    else {
      // if the pressure error has been happening long enough, take action
      if ((millis() - firstMaxPressureTime) > 30*1000) {
        // sending a warning (email) that the filters needs cleaning
        // cpd...not done
        Serial.println("high pressure for too long");
      }
    }
  }

  else {
    // no pressure issues....reset the pressure timers
    firstMinPressureTime = 0;
    firstMaxPressureTime = 0;
  }
}


void checkTemperature(int sensorIndex) {
  float tempF = sensors.getTempF(thermometer[sensorIndex]);
  tempF = (float)round(tempF*10)/10.0;

  int index = config.sensorId[sensorIndex];
  if ( tempF == lastTemp[index] )
    return;
    
  lastTemp[index] = tempF;

  if (screen == MAIN) {
    printTemperature(true, index);
  }
}


#define TimeBetweenFlashes 500
unsigned long lastFlashTime = 0;
bool isFlash = false;


void flashTime(unsigned long time) {
  // the temperature sensor blocks the app from running while it is reading.
  // so, it may make the flash look off every time it is being checked
  if (time - lastFlashTime > TimeBetweenFlashes) {
    lastFlashTime = time;
    isFlash = !isFlash;
    if (isFlash)
      eraseTime();
    else
      printTime(false, true, false);
  }
}


void checkTimeMinutes() {
  int minutes = minute();
  if (minutes == lastMinutes)
    return;

  // resync time at 3am every morning
  // this also catches daylight savings time changes which happen at 2am
  if (minutes == 0 && hour() == 3)
    isTimeSet = false;

  if (!isTimeSet)
    setupTime();
  
  lastMinutes = minutes;
  printTime(true, true, false);

  // if we're at a 15 minute mark, check to see if the pump needs to be turned on/off
  // (the program accuracy is granular to 15 minutes)
  if (minutes % 15 == 0)
    checkPump();

  if (minutes % 5 == 0) {
    // log the temperatures to the database
    if (lastTemp[0] != TEMP_ERROR)
      logTemperature(lastTemp[AIR], lastTemp[ROOF], lastTemp[POOL]);
  }
}


void checkButtons(void) {
  if (testClient != -1) {
    unsigned int heap = ESP.getFreeHeap();
    if (heap != testHeap) {
      testHeap = heap;
      char json[128];
      sprintf(json, "{\"command\":\"read\",\"heap\":\"%u\"}", heap);
      Serial.printf("sending %s\n", json);
      webSocket.sendTXT(testClient, json, strlen(json));
    }
  }

  for (int i=0; i < numButtons; ++i) {
    int reading = digitalRead(buttonPins[i]);
    
    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH),  and you've waited
    // long enough since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonStates[i]) {
      // reset the debouncing timer
      lastDebounceTimes[i] = millis();
    }

    if ((millis() - lastDebounceTimes[i]) > debounceDelay) {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading != buttonStates[i]) {
        buttonStates[i] = reading;
        if (reading != defaultButtonState)
          doButton(buttonPins[i]);
      }
    }

    // save the reading.  Next time through the loop,
    // it'll be the lastButtonState:
    lastButtonStates[i] = reading;
  }
}


void doButton(int pin) {
  lastButtonPressTime = millis();
  
  if (!isDisplayOn) {
    // just turn on the display, ignore the button press
    displayBacklight(true);
    return;
  }
  
  switch (pin) {
    case UP_BUTTON:
      doUpButton();
      break;
    case DOWN_BUTTON:
      doDownButton();
      break;
    case PUMP_BUTTON:
      doPumpButton();
      break;
    case SOLAR_BUTTON:
      doSolarButton();
      break;
  }
}


void doUpButton(void) {
  setTargetTemp(program.targetTemp+1);
}


void doDownButton(void) {
  setTargetTemp(program.targetTemp-1);
}


void doSolarButton(void) {
  if (screen == MAIN) {
    // first press, draw the menu screen
    screen = MENU;
    lcd.clear();

    lcd.setCursor(0,0);
    lcd.print(F("Solar Mode: "));
    lcd.print(modeNames[config.solarMode]);

    // display our ip address
    lcd.setCursor(0,1);
    lcd.print(WiFi.localIP());
  }
  else {
    // next press(es), change the mode
    if (++config.solarMode > AUTO)
      config.solarMode = ON;

    lcd.setCursor(12,0);
    lcd.print(modeNames[config.solarMode]);

    solarModeChange();
  }
}


void doPumpButton(void) {
  if (screen == MAIN) {
    // first press, draw the menu screen
    screen = MENU;
    lcd.clear();

    lcd.setCursor(0,0);
    lcd.print(F("Pump Mode: "));
    lcd.print(modeNames[config.pumpMode]);

    // display our ip address
    lcd.setCursor(0,1);
    lcd.print(WiFi.localIP());
  }
  else {
    // next press(es), change the mode
    if (++config.pumpMode > AUTO)
      config.pumpMode = ON;

    lcd.setCursor(11,0);
    lcd.print(modeNames[config.pumpMode]);

    pumpModeChange();
  }
}


void pumpModeChange(void) {
  saveConfig();
  printPumpModeState(true);
  printPumpState(true);
  checkPump();
}


void solarModeChange(void) {
  saveConfig();
  printSolarModeState(true);
  printSolarState(true);
  printTargetTemperature(true);
  checkSolar();
}


void sendWeb(const char *command, const char *value) {
  char json[128];
  sprintf(json, "{\"command\":\"%s\",\"value\":\"%s\"}", command, value);
  webSocket.sendTXT(webClient, json, strlen(json));
}


void displayBacklight(bool isOn) {
  isDisplayOn = isOn;
  if (isOn)
    lcd.backlight();
  else
    lcd.noBacklight();
}


void eraseTime(void) {
  lcd.setCursor(13,0);
  lcd.print(F("   "));
  lcd.setCursor(10,1);
  lcd.print(F("      "));
}


void printTime(bool isCheckProgram, bool isDisplay, bool isTest) {
  int dayOfWeek = weekday()-1;
  int hours = hour();
  int minutes = minute();

  const char *ampm = "a";
  int h = hours;
  if (h == 0)
    h = 12;
  else if (h == 12)
    ampm = "p";
  else if (h > 12) {
    h -= 12;
    ampm = "p";
  }
  char buf[7];
  sprintf(buf, "%2d:%02d%s", h, minutes, ampm); 
//  Serial.println(buf);

  if (isDisplay && screen == MAIN) {
    lcd.setCursor(10,1);
    lcd.print(buf);
  }
   
  if (webClient != -1 || isTest) {
    char msg[6+1+4];
    sprintf(msg, "%s %s", buf, weekdayNames[dayOfWeek]); 
    if (webClient != -1)
      sendWeb("time", msg);
    if (isTest)
      Serial.printf("time is %s\n", msg);
  }
}


void checkPump(void) {
  // turn the pump on or off if needed
  if (config.pumpMode == OFF) {
    if (isPumpOn) {
      logAction(PUMP_OFF);
      isPumpOn = false;
      pump(isPumpOn);
      printPumpState(true);
    }
  }
  else if (config.pumpMode == ON) {
    if (!isPumpOn) {
      logAction(PUMP_ON);
      isPumpOn = true;
      pump(isPumpOn);
      printPumpState(true);
    }
  }
  else {
    // pump on auto..check time
    bool isOn = false;
    byte ctime = hour()*4+(minute()/15);
//  Serial.printf("ctime %d\n", ctime);
    if (ctime >= program.pumpStart && ctime < program.pumpStop)
      isOn = true;

    if (isPumpOn != isOn) {
      if (isOn) {
        logAction(PUMP_ON);
        isPumpOn = true;
        pump(isPumpOn);
        printPumpState(true);
      }
      else {
        logAction(PUMP_OFF);
        isPumpOn = false;
        pump(isPumpOn);
        printPumpState(true);
      }
    }
  }

  checkSolar();
}


void checkSolar(void) {
  if (!isPumpOn) {
    // if the pump is off:  if the solar is on, turn it off
    if (isSolarOn) {
      logAction(SOLAR_OFF);
      isSolarOn = false;
      solar(isSolarOn);
      printSolarState(true);
    }
    return;
  }

  // if the pump is on:  see if the solar needs to be turned on or off
  if (config.solarMode == OFF) {
    if (isSolarOn) {
      logAction(SOLAR_OFF);
      isSolarOn = false;
      solar(isSolarOn);
      printSolarState(true);
    }
    return;
  }
  
  if (config.solarMode == ON) {
    if (!isSolarOn) {
      logAction(SOLAR_ON);
      isSolarOn = true;
      solar(isSolarOn);
      printSolarState(true);
    }
    return;
  }
  
  // solar on auto...check temperature
  bool isOn = false;
  float span = (float)config.temperature_span / 10.0;

  if (isSolarOn) {
    // to prevent short cycling, don't turn off until we're
    // <span> degrees above target temp (heating)
    if (lastTemp[POOL] < program.targetTemp+span) {
      // the pool temperature is less than the target temperature, so we'd like to turn the solar on
      // but only do so if the roof temperature is higher than the pool temperature
      if (lastTemp[POOL] < lastTemp[ROOF])
        isOn = true;
    }
  }
  else {
    // to prevent short cycling, don't turn on until we're
    // <span> degrees below target temp (heating)
    if (lastTemp[POOL] < program.targetTemp-span) {
      // the pool temperature is less than the target temperature, so we'd like to turn the solar on
      // but only do so if the roof temperature is higher than the pool temperature
      // (plus the span to prevent short cycling)
      if (lastTemp[POOL] < lastTemp[ROOF]-span)
        isOn = true;
    }
  }

  if (isSolarOn != isOn) {
    if (isOn) {
      logAction(SOLAR_ON);
      isSolarOn = true;
      solar(isSolarOn);
      printSolarState(true);
    }
    else {
      logAction(SOLAR_OFF);
      isSolarOn = false;
      solar(isSolarOn);
      printSolarState(true);
    }
  }
}


void pump(bool isOn) {
  digitalWrite(PUMP_GPIO, isOn ? LOW : HIGH);
}


void solar(bool isOn) {
  /*  
   *   LOW turns on the first scr, and the other one off
   *   HIGH turns the first scr off, and the other one on
   *   the solar value will open or close until it is complete (and a limit switch turns it off)
   */
  digitalWrite(SOLAR_GPIO, isOn ? HIGH : LOW);
}


#define MAGIC_NUM   0xAC

#define MAGIC_NUM_ADDRESS      0
#define CONFIG_ADDRESS         1
#define PROGRAM_ADDRESS        CONFIG_ADDRESS + sizeof(config)


void set(char *name, const char *value) {
  for (int i=strlen(value); i >= 0; --i)
    *(name++) = *(value++);
}


void loadConfig(void) {
  int magicNum = EEPROM.read(MAGIC_NUM_ADDRESS);
  if (magicNum != MAGIC_NUM) {
    Serial.println(F("invalid eeprom data"));
    isMemoryReset = true;
  }
  
  if (isMemoryReset) {
    // nothing saved in eeprom, use defaults
    Serial.println(F("using default config"));
    set(config.host_name, HOST_NAME);
    set(config.mqtt_ip_addr, MQTT_IP_ADDR);
    config.mqtt_ip_port = MQTT_IP_PORT;
    config.use_mqtt = 0;
    config.pumpMode = OFF;
    config.solarMode = OFF;
    config.sensorId[0] = 0;
    config.sensorId[1] = 1;
    config.sensorId[2] = 2;
    config.temperature_span = 5;
    config.display_timeout = 20;
    config.use_logging = 1;
    set(config.logging_ip_addr, LOGGING_IP_ADDR);
    config.logging_ip_port = LOGGING_IP_PORT;
    set(config.log_url, LOG_URL);

    saveConfig();
  }
  else {
    int addr = CONFIG_ADDRESS;
    byte *ptr = (byte *)&config;
    for (int i=0; i < sizeof(config); ++i, ++ptr)
      *ptr = EEPROM.read(addr++);
  }

  for (int i=0; i < 3; ++i)
    lastTemp[i] = TEMP_ERROR;  

  Serial.printf("host_name %s\n", config.host_name);
  Serial.printf("use_mqtt %d\n", config.use_mqtt);
  Serial.printf("mqqt_ip_addr %s\n", config.mqtt_ip_addr);
  Serial.printf("mqtt_ip_port %d\n", config.mqtt_ip_port);
  Serial.printf("pumpMode %d\n", config.pumpMode);
  Serial.printf("solarMode %d\n", config.solarMode);
  Serial.printf("sensorId[0] %d\n", config.sensorId[0]);
  Serial.printf("sensorId[1] %d\n", config.sensorId[1]);
  Serial.printf("sensorId[2] %d\n", config.sensorId[2]);
  Serial.printf("temperature_span %d\n", config.temperature_span);
  Serial.printf("display_timeout %d\n", config.display_timeout);
  Serial.printf("use_logging %d\n", config.use_logging);
  Serial.printf("logging_ip_addr %s\n", config.logging_ip_addr);
  Serial.printf("logging_ip_port %d\n", config.logging_ip_port);
  Serial.printf("log_url %s\n", config.log_url);
}


void loadProgramConfig(void) {
  if (isMemoryReset) {
    // nothing saved in eeprom, use defaults
    Serial.printf("using default programs\n");
    initProgram();  
  }
  else {
    Serial.printf("loading programs from eeprom\n");
    int addr = PROGRAM_ADDRESS;
    byte *ptr = (byte *)&program;
    for (int i = 0; i < sizeof(program); ++i, ++ptr, ++addr)
      *ptr = EEPROM.read(addr);
  }

  logTarget(program.targetTemp);

  Serial.printf("pumpStart %d\n", program.pumpStart);
  Serial.printf("pumpStop %d\n", program.pumpStop);
  Serial.printf("targetTemp %d\n", program.targetTemp);
  Serial.printf("pressureLow %d\n", program.pressureLow);
  Serial.printf("pressureHigh %d\n", program.pressureHigh);
}


void saveConfig(void) {
  isPromModified = false;
  update(MAGIC_NUM_ADDRESS, MAGIC_NUM);

  byte *ptr = (byte *)&config;
  int addr = CONFIG_ADDRESS;
  for (int j=0; j < sizeof(config); ++j, ++ptr)
    update(addr++, *ptr);
  
  if (isPromModified)
    EEPROM.commit();
}


void update(int addr, byte data) {
  if (EEPROM.read(addr) != data) {
    EEPROM.write(addr, data);
    isPromModified = true;
  }
}


void setupOta(void) {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(config.host_name);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    const char *msg = "Unknown Error";
    if (error == OTA_AUTH_ERROR) {
      msg = "Auth Failed";
    } else if (error == OTA_BEGIN_ERROR) {
      msg = "Begin Failed";
    } else if (error == OTA_CONNECT_ERROR) {
      msg = "Connect Failed";
    } else if (error == OTA_RECEIVE_ERROR) {
      msg = "Receive Failed";
    } else if (error == OTA_END_ERROR) {
      msg = "End Failed";
    }
    Serial.println(msg);
  });
  
  ArduinoOTA.begin();
  Serial.println("Arduino OTA ready");

  char host[20];
  sprintf(host, "%s-webupdate", config.host_name);
  MDNS.begin(host);
  httpUpdater.setup(&server);
  MDNS.addService("http", "tcp", 80);
  Serial.println("Web OTA ready");
}


void setupMqtt() {
  client.setServer(config.mqtt_ip_addr, config.mqtt_ip_port);
  client.setCallback(callback);
}


void saveProgramConfig(void) {
  isPromModified = false;
  Serial.printf("saving programs to eeprom\n");
  int addr = PROGRAM_ADDRESS;
  byte *ptr = (byte *)&program;
  for (int i = 0; i < sizeof(program); ++i, ++ptr, ++addr)
    update(addr, *ptr);

  if (isPromModified)
    EEPROM.commit();
}


void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      if (num == webClient)
        webClient = -1;
      else if (num == programClient)
        programClient = -1;
      else if (num == setupClient)
        setupClient = -1;
      else if (num == testClient) {
        testClient = -1;
        testHeap = 0;
      }
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      
      if (strcmp((char *)payload,"/") == 0) {
        webClient = num;
      
        // send the current state
        printPumpModeState(false);
        printSolarModeState(false);
        printPumpState(false);
        printSolarState(false);
        for (int i=0; i < 3; ++i)
          printTemperature(false, i);
        printTargetTemperature(false);
        printPressure(false);
        printTime(false, false, false);
        sendWeb("log_url", config.log_url);
      }
      else if (strcmp((char *)payload,"/program") == 0) {
        programClient = num;
        
        // send program
        char json[265];
        sprintf(json, "{\"pumpStart\":\"%d\",\"pumpStop\":\"%d\",\"targetTemp\":\"%d\",\"pressureLow\":\"%d\",\"pressureHigh\":\"%d\"}",
          program.pumpStart, program.pumpStop, program.targetTemp, program.pressureLow, program.pressureHigh);
        //Serial.printf("len %d\n", strlen(json));
        webSocket.sendTXT(programClient, json, strlen(json));
      }
      else if (strcmp((char *)payload,"/setup") == 0) {
        setupClient = num;

        char json[256];
        strcpy(json, "{");
        sprintf(json+strlen(json), "\"date\":\"%s\"", __DATE__);
        sprintf(json+strlen(json), ",\"time\":\"%s\"", __TIME__);
        sprintf(json+strlen(json), ",\"host_name\":\"%s\"", config.host_name);
        sprintf(json+strlen(json), ",\"use_mqtt\":\"%d\"", config.use_mqtt);
        sprintf(json+strlen(json), ",\"mqtt_ip_addr\":\"%s\"", config.mqtt_ip_addr);
        sprintf(json+strlen(json), ",\"mqtt_ip_port\":\"%d\"", config.mqtt_ip_port);
        sprintf(json+strlen(json), ",\"ssid\":\"%s\"", ssid.c_str());
        sprintf(json+strlen(json), ",\"span\":\"%d\"", config.temperature_span);
        sprintf(json+strlen(json), ",\"timeout\":\"%d\"", config.display_timeout);
        sprintf(json+strlen(json), ",\"sensor0\":\"%d\"", config.sensorId[0]);
        sprintf(json+strlen(json), ",\"sensor1\":\"%d\"", config.sensorId[1]);
        sprintf(json+strlen(json), ",\"sensor2\":\"%d\"", config.sensorId[2]);
        sprintf(json+strlen(json), ",\"use_logging\":\"%d\"", config.use_logging);
        sprintf(json+strlen(json), ",\"logging_ip_addr\":\"%s\"", config.logging_ip_addr);
        sprintf(json+strlen(json), ",\"logging_ip_port\":\"%d\"", config.logging_ip_port);
        sprintf(json+strlen(json), ",\"log_url\":\"%s\"", config.log_url);
        strcpy(json+strlen(json), "}");
//        Serial.printf("len %d\n", strlen(json));
        webSocket.sendTXT(setupClient, json, strlen(json));
      }
      else if (strcmp((char *)payload,"/test") == 0) {
        testClient = num;
      }
      else {
        Serial.printf("unknown call %s\n", payload);
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);
      
      if (num == webClient) {
        const char *target = "command";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        if (strncmp(ptr,"tempUp",6) == 0) {
          setTargetTemp(program.targetTemp+1);
        }
        else if (strncmp(ptr,"tempDown",8) == 0) {
          setTargetTemp(program.targetTemp-1);
        }  
        else if (strncmp(ptr,"pumpMode",8) == 0) {
          target = "value";
          ptr = strstr(ptr, target) + strlen(target)+3;
          config.pumpMode = strtol(ptr, &ptr, 10);
          pumpModeChange();
        }        
        else if (strncmp(ptr,"solarMode",9) == 0) {
          target = "value";
          ptr = strstr(ptr, target) + strlen(target)+3;
          config.solarMode = strtol(ptr, &ptr, 10);
          solarModeChange();
        }        
      }
      else if (num == programClient) {
        Serial.printf("save programs\n");

        const char *target = "pumpStart";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        program.pumpStart = strtol(ptr, &ptr, 10);
        
        target = "pumpStop";
        ptr = strstr((char *)payload, target) + strlen(target)+3;
        program.pumpStop = strtol(ptr, &ptr, 10);
        
        target = "targetTemp";
        ptr = strstr((char *)payload, target) + strlen(target)+3;
        program.targetTemp = strtol(ptr, &ptr, 10);

        logTarget(program.targetTemp);
      
        target = "pressureLow";
        ptr = strstr((char *)payload, target) + strlen(target)+3;
        program.pressureLow = strtol(ptr, &ptr, 10);
        
        target = "pressureHigh";
        ptr = strstr((char *)payload, target) + strlen(target)+3;
        program.pressureHigh = strtol(ptr, &ptr, 10);
        
        saveProgramConfig();
        drawMainScreen();
      }
      else if (num == setupClient) {
        const char *target = "command";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        if (strncmp(ptr,"reboot",6) == 0) {
          ESP.restart();
        }
        else if (strncmp(ptr,"save",4) == 0) {
          Serial.printf("save setup\n");

          const char *target = "host_name";
          char *ptr = strstr((char *)payload, target) + strlen(target)+3;
          char *end = strchr(ptr, '\"');
          memcpy(config.host_name, ptr, (end-ptr));
          config.host_name[end-ptr] = '\0';

          target = "use_mqtt";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.use_mqtt = strtol(ptr, &ptr, 10);

          target = "mqtt_ip_addr";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          end = strchr(ptr, '\"');
          memcpy(config.mqtt_ip_addr, ptr, (end-ptr));
          config.mqtt_ip_addr[end-ptr] = '\0';

          target = "mqtt_ip_port";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.mqtt_ip_port = strtol(ptr, &ptr, 10);

          target = "span";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.temperature_span = strtol(ptr, &ptr, 10);
  
          target = "timeout";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.display_timeout = strtol(ptr, &ptr, 10);
  
          target = "sensor0";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.sensorId[0] = strtol(ptr, &ptr, 10);

          target = "sensor1";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.sensorId[1] = strtol(ptr, &ptr, 10);

          target = "sensor2";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.sensorId[2] = strtol(ptr, &ptr, 10);
  
          target = "use_logging";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.use_logging = strtol(ptr, &ptr, 10);
          
          target = "logging_ip_addr";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          end = strchr(ptr, '\"');
          memcpy(config.logging_ip_addr, ptr, (end-ptr));
          config.logging_ip_addr[end-ptr] = '\0';

          target = "logging_ip_port";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.logging_ip_port = strtol(ptr, &ptr, 10);

          target = "log_url";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          end = strchr(ptr, '\"');
          memcpy(config.log_url, ptr, (end-ptr));
          config.log_url[end-ptr] = '\0';

    Serial.printf("host_name %s\n", config.host_name);
    Serial.printf("use_mqtt %d\n", config.use_mqtt);
    Serial.printf("mqtt_ip_addr %s\n", config.mqtt_ip_addr);
    Serial.printf("mqtt_ip_port %d\n", config.mqtt_ip_port);
    Serial.printf("temperature_span %d\n", config.temperature_span);
    Serial.printf("display_timeout %d\n", config.display_timeout);
    Serial.printf("sensor0 %d\n", config.sensorId[0]);
    Serial.printf("sensor1 %d\n", config.sensorId[1]);
    Serial.printf("sensor2 %d\n", config.sensorId[2]);
    Serial.printf("use_logging %d\n", config.use_logging);
    Serial.printf("logging_ip_addr %s\n", config.logging_ip_addr);
    Serial.printf("logging_ip_port %d\n", config.logging_ip_port);
    Serial.printf("log_url %s\n", config.log_url);
          saveConfig();
        }
      }
      else if (num == testClient) {
      }
      break;
  }
}


const char http_path_temperature[] = "/pool/remote/insertTemperature.php";
const char http_path_action[] = "/pool/remote/insertAction.php";
const char http_path_target[] = "/pool/remote/insertTarget.php";


void logTemperature(float air, float roof, float pool) {
  if (!config.use_logging)
    return;

  AsyncClient* aclient = new AsyncClient();

  float* data = new float[3];
  data[0] = air;
  data[1] = roof;
  data[2] = pool;
  
  aclient->onConnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onConnect\n");
    
    float* data = (float*) obj;
    float air = data[0];
    float roof = data[1];
    float pool = data[2];
    free(obj);
    
    // after connecting, send the request
    // Make an HTTP GET request
    c->write( "GET ");
    c->write( http_path_temperature );
    c->write("?air=");
    c->write( String(air).c_str() );
    c->write("&");
    c->write("roof=");
    c->write( String(roof).c_str() );
    c->write("&");
    c->write("pool=");
    c->write( String(pool).c_str() );
    c->write( " HTTP/1.1\r\n");
    c->write("Host: ");
    c->write(config.logging_ip_addr);
    c->write( "\r\nContent-Type: application/x-www-form-urlencoded\r\n" );
    c->write("Connection: close\r\n\r\n");
    c->stop();

    Serial.print("logged temperature: air: ");
    Serial.print(air,1);
    Serial.print(" roof: ");
    Serial.print(roof,1);
    Serial.print(" pool: ");
    Serial.println(pool,1);
  }, data);

  aclient->onDisconnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onDisconnect\n");
    free(c);
  }, NULL);

  aclient->onError([](void *obj, AsyncClient* c, int8_t err) {
    Serial.printf("logTemperature [A-TCP] onError: %s\n", c->errorToString(err));
  }, NULL);

  aclient->onData([](void *obj, AsyncClient* c, void *buf, size_t len) {
//    Serial.printf("[A-TCP] onData: %s\n", buf);
  }, NULL);

//  Serial.printf("connecting to %s:%d\n", config.logging_ip_addr, config.logging_ip_port);
//  unsigned long t = millis();
  if (!aclient->connect(config.logging_ip_addr, config.logging_ip_port)) {
//    Serial.printf("connect failed %d\n", (millis() - t));
    free(aclient);
    free(data);
  }
}


void logAction(char action) {
  if (!config.use_logging)
    return;
    
  char* str = new char[2];
  str[0] = action;
  str[1] = '\0';

  AsyncClient* aclient = new AsyncClient();

  aclient->onConnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onConnect\n");
    // after connecting, send the request
    // Make an HTTP GET request
    c->write( "GET ");
    c->write( http_path_action );
    c->write("?action=");
    char* str = (char*) obj;
    c->write( str );
    c->write( " HTTP/1.1\r\n");
    c->write("Host: ");
    c->write(config.logging_ip_addr);
    c->write( "\r\nContent-Type: application/x-www-form-urlencoded\r\n" );
    c->write("Connection: close\r\n\r\n");
    c->stop();

    Serial.print("logged action: ");
    Serial.println(str);
    free(str);
  }, str);

  aclient->onDisconnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onDisconnect\n");
    free(c);
  }, NULL);

  aclient->onError([](void *obj, AsyncClient* c, int8_t err) {
    Serial.printf("logAction [A-TCP] onError: %s\n", c->errorToString(err));
  }, NULL);

  aclient->onData([](void *obj, AsyncClient* c, void *buf, size_t len) {
//    Serial.printf("[A-TCP] onData: %s\n", buf);
  }, NULL);

//  Serial.printf("connecting to %s:%d\n", config.logging_ip_addr, config.logging_ip_port);
  if (!aclient->connect(config.logging_ip_addr, config.logging_ip_port)) {
    free(aclient);
    free(str);
  }
}


void logTarget(int temperature) {
  if (!config.use_logging)
    return;

  if (temperature == TEMP_ERROR)
    return;
    
  int* data = new int[1];
  data[0] = temperature;

  AsyncClient* aclient = new AsyncClient();

  aclient->onConnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onConnect\n");
    
    int* data = (int*) obj;
    int temperature = data[0];
    free(obj);
    
    // after connecting, send the request
    // Make an HTTP GET request
    c->write( "GET ");
    c->write( http_path_target );
    c->write("?temperature=");
    c->write( String(temperature).c_str() );
    c->write( " HTTP/1.1\r\n");
    c->write("Host: ");
    c->write(config.logging_ip_addr);
    c->write( "\r\nContent-Type: application/x-www-form-urlencoded\r\n" );
    c->write("Connection: close\r\n\r\n");
    c->stop();

    Serial.print("logged target: ");
    Serial.println(temperature);
  }, data);

  aclient->onDisconnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onDisconnect\n");
    free(c);
  }, NULL);

  aclient->onError([](void *obj, AsyncClient* c, int8_t err) {
    Serial.printf("logTarget [A-TCP] onError: %s\n", c->errorToString(err));
  }, NULL);

  aclient->onData([](void *obj, AsyncClient* c, void *buf, size_t len) {
//    Serial.printf("[A-TCP] onData: %s\n", buf);
  }, NULL);

//  Serial.printf("connecting to %s:%d\n", config.logging_ip_addr, config.logging_ip_port);
  if (!aclient->connect(config.logging_ip_addr, config.logging_ip_port)) {
    free(aclient);
    free(data);
  }
}


//format bytes
String formatBytes(size_t bytes){
  if (bytes < 1024){
    return String(bytes)+"B";
  } else if(bytes < (1024 * 1024)){
    return String(bytes/1024.0)+"KB";
  } else if(bytes < (1024 * 1024 * 1024)){
    return String(bytes/1024.0/1024.0)+"MB";
  } else {
    return String(bytes/1024.0/1024.0/1024.0)+"GB";
  }
}


String getContentType(String filename){
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}


bool handleFileRead(String path){
  Serial.println("handleFileRead: " + path);
  if(path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}


void handleFileUpload_edit(){
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    Serial.print("handleFileUpload Name: "); Serial.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    //Serial.print("handleFileUpload Data: "); Serial.println(upload.currentSize);
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile)
      fsUploadFile.close();
    Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
  }
}


void handleFileDelete(){
  if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.println("handleFileDelete: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}


void handleFileCreate(){
  if(server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.println("handleFileCreate: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(SPIFFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if(file)
    file.close();
  else
    return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
}


void handleFileList() {
  if(!server.hasArg("dir")) {server.send(500, "text/plain", "BAD ARGS"); return;}
  
  String path = server.arg("dir");
  Serial.println("handleFileList: " + path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while(dir.next()){
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir)?"dir":"file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }
  
  output += "]";
  server.send(200, "text/json", output);
}


void countRootFiles(void) {
  int num = 0;
  size_t totalSize = 0;
  Dir dir = SPIFFS.openDir("/");
  while (dir.next()) {
    ++num;
    String fileName = dir.fileName();
    size_t fileSize = dir.fileSize();
    totalSize += fileSize;
    Serial.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
  }
  Serial.printf("FS File: serving %d files, size: %s from /\n", num, formatBytes(totalSize).c_str());
}


void setupWebServer(void) {
  SPIFFS.begin();

  countRootFiles();

  //list directory
  server.on("/list", HTTP_GET, handleFileList);
  
  //load editor
  server.on("/edit", HTTP_GET, [](){
    if(!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound");
  });
  
  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleFileUpload_edit);

  //called when the url is not defined here
  //use it to load content from SPIFFS
  server.onNotFound([](){
    if(!handleFileRead(server.uri()))
      server.send(404, "text/plain", "FileNotFound");
  });

  server.begin();

  Serial.println("HTTP server started");
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
//    // Create a random client ID
//    String clientId = "ESP8266Client-";
//    clientId += String(random(0xffff), HEX);
//    // Attempt to connect
//    if (client.connect(clientId.c_str())) {
    if (client.connect(config.host_name)) {
      Serial.println("connected");
      // ... and resubscribe
      char topic[30];
      sprintf(topic, "%s/command", config.host_name);
      client.subscribe(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void callback(char* topic, byte* payload, unsigned int length) {
  // only topic we get is <host_name>/command or nightlight
  
  char value[12];
  memcpy(value, payload, length);
  value[length] = '\0';
  Serial.printf("Message arrived [%s] %s\n", topic, value);

//  if (strcmp(topic, "command") == 0) {
//    // cpd...do something
//      
//    // also send to main display
//    if (webClient != -1) {
//      sendWeb("code", value);
//    }
//  }
//  else {
//    Serial.printf("Unknown topic\n");
//  }
}




/*
todo:

remove solar controller and desolder 3 pin connector
wire connector and solar value circuit
make a daughter board?

remove old timer box, and wire up new one

power supply issues....ordered better dc-dc voltage regulator
is the higher input voltage blowing up the power supplies?
test with oscope

-----------------------------------

mail.php on pogoplug2 will send an email
(installed PhpMailer on pogoplug)
used yahoo as the smtp server
had to change account and set allow less secure
pool app will need to hit php page to send the email.
since yahoo and most other email servers require ssl which the esp8266 can't do


database setup:
CREATE database `pool`
create table action (
  ts timestamp DEFAULT CURRENT_TIMESTAMP,
  action char
);
insert into action (state) values ('c')

create table target (
  ts timestamp DEFAULT CURRENT_TIMESTAMP,
  temperature int
);
insert into target (temperature) values (78)

CREATE TABLE temps (
  ts TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  air FLOAT,
  roof FLOAT,
  pool FLOAT
);


probably need to add a i2c mux since we're out of gpio pins and we'd to interface to the spa heater someday


it would be nice to use interupts for buttons instead of current polling code.  (low priority)
*/
