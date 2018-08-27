/******************************************************************************
* Simple OBD test sketch for Freematics ONE+
* Written by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products for hardware information
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/
////library for OBDII interface 
#include <FreematicsPlus.h> //OBD2 library for Freematics development board
#include "config.h"
//#include <telelogger.h>

///Library for GSM_MODEM_MQTT
// Select your modem:
#define TINY_GSM_MODEM_SIM800
//#define TINY_GSM_MODEM_SIM900
//#define TINY_GSM_MODEM_A6
//#define TINY_GSM_MODEM_M590
//#define DUMP_AT_COMMANDS
//#define DUMP_AT_COMMANDS


// Your GPRS credentials
// Leave empty, if missing user or pass
//const char UNLOCK_CODE[] = "0000";
const char apn[]  = "v-internet"; //Vietnam / Viettel 
//const char apn[]  = "m-wap"; //Vietnam / Mobifone 

const char user[] = "";
const char pass[] = "";
#include <TinyGsmClient.h>
#include <PubSubClient.h>

HardwareSerial SerialNum2(1);
//#define SerialAT Serial2
#define SerialMon Serial
#define SerialAT SerialNum2

#define TINY_GSM_DEBUG SerialMon    
//#define DUMP_AT_COMMANDS 1 //comment out this line to debug data streammings
#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif
///////////////////////////////////////////////////////////////////////////////////////////////


const char* broker = "mqtt.mydevices.com";
const char* _user = "bb0ca970-46df-11e7-afce-8d5fd2a310a7";    
const char* _pass = "92a63d91d0f06220df912254da9b338ac7e9aa7c";
const char* _ClientID = "b44fa3a0-58c1-11e7-9118-bfd202a30a41";
const char* rpmTopic = "v1/bb0ca970-46df-11e7-afce-8d5fd2a310a7/things/b44fa3a0-58c1-11e7-9118-bfd202a30a41/data/1";
const char* battTopic = "v1/bb0ca970-46df-11e7-afce-8d5fd2a310a7/things/b44fa3a0-58c1-11e7-9118-bfd202a30a41/data/2";
const char* ledStateTopic="v1/bb0ca970-46df-11e7-afce-8d5fd2a310a7/things/b44fa3a0-58c1-11e7-9118-bfd202a30a41/data/3";
const char* sampleTopic="v1/bb0ca970-46df-11e7-afce-8d5fd2a310a7/things/b44fa3a0-58c1-11e7-9118-bfd202a30a41/data/4";
///////////////////////////


///////////////////////////
#define PIN_LED 4

bool connected = false;
bool server_connected = false; 
bool ledStatus = false;
unsigned long count = 0;
unsigned long lastReconnectAttempt;
uint16_t MQTT_port = 1883; 
///////////////
//TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
////////////////////////////////////////////
///System variable 
// logger states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_CONNECTED 0x20
#define STATE_WORKING 0x40
#define STATE_STANDBY 0x80

typedef struct {
  byte pid;
  byte tier;
  int16_t value;
  uint32_t ts;
} PID_POLLING_INFO;

PID_POLLING_INFO obdData[]= {
  {PID_SPEED, 1},
  {PID_RPM, 1},
  {PID_THROTTLE, 1},
  {PID_ENGINE_LOAD, 1},
  {PID_FUEL_PRESSURE, 2},
  {PID_TIMING_ADVANCE, 2},
  {PID_COOLANT_TEMP, 3},
  {PID_INTAKE_TEMP, 3},
};

#if MEMS_MODE
float accBias[3] = {0}; // calibrated reference accelerometer data
float acc[3] = {0};
uint32_t lastMotionTime = 0;
#endif

// live data
char vin[18] = {0};
uint16_t dtc[6] = {0};
int16_t batteryVoltage = 0;

/*#if ENABLE_GPS
GPS_DATA* gd = 0;
uint32_t lastGPSts = 0;
uint8_t lastGPSDay = 0;
#endif
*/
char isoTime[24] = {0};
uint8_t deviceTemp = 0; // device temperature

// stats data
float lastKph = 0;
uint32_t lastKphTime = 0;
float distance = 0;
uint32_t timeoutsOBD = 0;
uint32_t timeoutsNet = 0;
#define DATA_SENDING_INTERVAL 5000
uint32_t sendingInterval = DATA_SENDING_INTERVAL;

uint8_t connErrors = 0;

int fileid = 0;
//static uint8_t lastSizeKB = 0;

uint32_t lastCmdToken = 0;
String serialCommand;

byte ledMode = 0;
class State {
public:
  bool check(byte flags) { return (m_state & flags) == flags; }
  void set(byte flags) { m_state |= flags; }
  void clear(byte flags) { m_state &= ~flags; }
private:
  byte m_state = 0;
};
/////////////////////////////////
//FreematicsESP32 sys;
State state;
/*
//////////////////////////////////////////////
#if MEMS_MODE == MEMS_ACC
MPU9250_ACC mems;
#elif MEMS_MODE == MEMS_9DOF
MPU9250_9DOF mems;
#elif MEMS_MODE == MEMS_DMP
MPU9250_DMP mems;
#endif
//////////////////////////////////////////////
CStorageRAM cache;
#if STORAGE == STORAGE_SPIFFS
SPIFFSLogger store;
#elif STORAGE == STORAGE_SD
SDLogger store;
#endif
*/
//////////////////////////////////////////////////////////////////
COBDSPI obd;
void printTimeoutStats() 
{
  Serial.print("Timeouts: OBD:");
  Serial.print(timeoutsOBD);
 // Serial.print(" Network:");
 // Serial.println(timeoutsNet);
}
//////////////////////////////////////////////////////////////////
void Starting_modem(){
bool Network_Connected = false; 
bool GSM_Connected = false;   
while (!Network_Connected) {
  while (!GSM_Connected) {
    Serial.println("Initializing modem...");
    modem.restart();
  //pinMode(PIN_BEE_PWR, OUTPUT);
	digitalWrite(PIN_BEE_PWR, HIGH);
    delay(3000);
  
  String modemInfo = modem.getModemInfo();
  Serial.println(String("Modem info:")+modemInfo);


     Serial.println("Waiting for GSM network...");
       if (!modem.waitForNetwork()) {
      Serial.println(" fail...Restart modem..");
      
         }
      else GSM_Connected = true; 

    }
  if (modem.isNetworkConnected()) {
    Serial.println("Telecom Network connected");
  }
  Serial.println(" OK");
  
  Serial.print("Connecting to ");
  Serial.println(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(" GPRS fail...restart..");
    
    }
  else  {
    Serial.println("GPRS  connected");
    Network_Connected = true;
   }
  }
  Serial.println(" IoT System Connected!Ready for operation");
  delay(2000);
}///
void mqtt_Callback(char* topic, byte* payload, unsigned int len) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.write(payload, len);
  Serial.println();

  // Only proceed if incoming message's topic matches
  if (String(topic) == sampleTopic) {
    ledStatus = !ledStatus;
   // digitalWrite(LED_PIN, ledStatus);
    mqtt.publish(sampleTopic, ledStatus ? "1" : "0");
  }
}
///////////////
/*
void processOBD()
{
  static int idx[2] = {0, 0};
  int tier = 1;
  uint32_t t = millis();
  for (byte i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
    if (obdData[i].tier > tier) {
        // reset previous tier index
        idx[tier - 2] = 0;
        // keep new tier number
        tier = obdData[i].tier;
        // move up current tier index
        i += idx[tier - 2]++;
        // check if into next tier
        if (obdData[i].tier != tier) {
            idx[tier - 2]= 0;
            i--;
            continue;
        }
    }
    byte pid = obdData[i].pid;
    if (!obd.isValidPID(pid)) continue;
    int value;
    if (obd.readPID(pid, value)) {
        obdData[i].ts = millis();
        cache.log((uint16_t)pid | 0x100, value);
    } else {
        timeoutsOBD++;
        printTimeoutStats();
    }
    if (tier > 1) break;
  }

#if ENABLE_GPS
  if (!gd || !gd->ts)
#endif
  {
    // calculate distance for speed
    float kph = obdData[0].value;
    if (lastKphTime) distance += (kph + lastKph) * (t - lastKphTime) / 3600 / 2;
    lastKphTime = t;
    lastKph = kph;
  }
}*/
/////
void init_system(){

}
void test_MQTT_cayenne();
/////
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  delay(1000);
  digitalWrite(PIN_LED, LOW);
  //Serial.begin(9600);
  //sys.begin();
  SerialMon.begin(115200);
   
  //config OBD interface
  byte ver = obd.begin();
  Serial.print("OBD Firmware Version ");
  Serial.println(ver);
  //Config Modem interface
  SerialAT.begin(115200,SERIAL_8N1,16,17,false);//hardware UART #1 in Esp32
  //sys.xbBegin(115200);
  //SerialAT.begin(115200);//hardware UART #1 in Esp32              
  #ifdef PIN_BEE_PWR
	pinMode(PIN_BEE_PWR, OUTPUT);
	digitalWrite(PIN_BEE_PWR, HIGH);
  #endif
  //delay(15000);
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  Starting_modem();
  
  // MQTT Broker setup
  mqtt.setServer(broker, MQTT_port);
  mqtt.setCallback(mqtt_Callback);

  test_MQTT_cayenne();
}
/////

////////////////////////
bool MQTT_pub_bool(const char* topic,bool value ) {

return mqtt.publish(topic,value ? "1" : "0");
}
///////////////////////
bool MQTT_pub_int(const char* topic,int value) {
char str[1 + 8 * sizeof(value)];
utoa(value, str, 10);
return mqtt.publish(topic,str);
}
////////////////////////////////////
bool MQTT_pub_long(const char* topic,unsigned long value ) {
char str[1 + 8 * sizeof(value)];
ultoa(value, str, 10);
return mqtt.publish(topic,str);
}
//////////////////////////////////
////////////////////////////////////
bool MQTT_pub_double(const char* topic,double value ) {
char str[33];
dtostrf(value, 5, 3, str);
return mqtt.publish(topic,str);
}
/////////////////////////////////////
////////////////////////////////////
bool MQTT_pub_float(const char* topic,float value ) {
char str[33];
dtostrf(value, 5, 3, str);
return mqtt.publish(topic,str);   
}
///////////////////////////
bool mqttConnect() {
  Serial.print("Connecting to ");
  Serial.print(broker);
  if (!mqtt.connect(_ClientID,_user,_pass)) {
    Serial.println(" fail");
    return false;
  }
  Serial.println(" OK");
 // mqtt.publish(topicInit, "GsmClientTest started");
  MQTT_pub_bool(ledStateTopic,connected);// publish OBD connection status
 // mqtt.subscribe(topicLed);
  return mqtt.connected();
}
/////////////////////////////////////////////
void MQTT_processing(){
  Serial.println("MQTT processing..");
  server_connected = mqtt.connected();
  if (server_connected) {
      if(!mqtt.loop()) server_connected = false; //check Ping
      else {
           server_connected = true; 
           Serial.println("connection good");
             } 
      }
      else {
    Serial.println("disconnected...try to reconnect..");
    // Reconnect every 10 seconds
    unsigned long t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
        Serial.println("Reconnect successfully!");
      }
    }
  }
}
/////////////////////////////////////
char* makeFloatData(const char* type, const char* unit,float value ){
char* strdata = new char[20];
sprintf(strdata,"%s,%s=%f",type,unit,value);
Serial.println(strdata);
return strdata;
}
/////////////////////////////////////
void test_MQTT_cayenne(){
  int value; 
  while(true) {

  for (int i=0;i<10;i++){
    value = i*100;
    Serial.print("send RPM: ");Serial.println(value);
    if(MQTT_pub_int(rpmTopic,value)) {
      Serial.println("...Sent OK");  
       }
    else {
      Serial.println("...Fail");
       }
     Serial.print("send Battery Voltage: ");Serial.println(i);
    if(mqtt.publish(battTopic,makeFloatData("batt","v",(float)i))) {
      Serial.println("...Sent OK");  
       }
    else {
      Serial.println("...Fail");
       }   
      delay(2000);
      MQTT_processing();//check connection and reconnect if lost
 
    }
  }
}
////


/////////////////////////////////////////
void loop() {
 // uint32_t ts = millis();
  digitalWrite(PIN_LED, HIGH);
  test_MQTT_cayenne();
 // MQTT_processing();  // Make sure server connection 

//#if CONNECT_OBD
  if (!connected) {
    digitalWrite(PIN_LED, HIGH);
    Serial.print("Connecting to OBD...");
    if (obd.init()) {
      Serial.println("OK");
      connected = true;
    } else {
      Serial.println("OBD...connect fail");
    }
    digitalWrite(PIN_LED, LOW);
    return;
  }
//#endif
  int value;
  Serial.print('[');
  Serial.print(millis());
  Serial.print("] #");
  Serial.print(count++);

  if (obd.readPID(PID_RPM, value)) {
    Serial.print(" RPM:");
    Serial.print(value);
    MQTT_pub_int(rpmTopic,value);
  }
  if (obd.readPID(PID_SPEED, value)) {
    Serial.print(" SPEED:");
    Serial.print(value);
  }

  float volt = obd.getVoltage();
  Serial.print(" BATTERY:");
  Serial.print(volt);
  Serial.print('V');
  MQTT_pub_float(battTopic,volt);
#ifdef ESP32
  int temp = (int)readChipTemperature() * 165 / 255 - 40;
  Serial.print(" CPU TEMP:");
  Serial.print(temp);
#endif
  Serial.println();
  if (obd.errors > 2) {
    Serial.println("OBD disconnected");
    connected = false;
    obd.reset();
  }
  digitalWrite(PIN_LED, LOW);

 delay(2000);
}
///////////////////////////////////////////////////////////////////////////////////////////
