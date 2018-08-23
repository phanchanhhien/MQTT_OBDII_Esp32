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




const char* broker = "mqtt.mydevices.com";
const char* _user = "bb0ca970-46df-11e7-afce-8d5fd2a310a7";    
const char* _pass = "92a63d91d0f06220df912254da9b338ac7e9aa7c";
const char* _ClientID = "b44fa3a0-58c1-11e7-9118-bfd202a30a41";
const char* rpmTopic = "v1/bb0ca970-46df-11e7-afce-8d5fd2a310a7/things/b44fa3a0-58c1-11e7-9118-bfd202a30a41/data/1";
const char* ledStateTopic="v1/bb0ca970-46df-11e7-afce-8d5fd2a310a7/things/b44fa3a0-58c1-11e7-9118-bfd202a30a41/data/2";
const char* sampleTopic="v1/bb0ca970-46df-11e7-afce-8d5fd2a310a7/things/b44fa3a0-58c1-11e7-9118-bfd202a30a41/data/3";
const char* battTopic = "v1/bb0ca970-46df-11e7-afce-8d5fd2a310a7/things/b44fa3a0-58c1-11e7-9118-bfd202a30a41/data/4";



#define PIN_LED 4

bool connected = false;
bool server_connected = false; 
bool ledStatus = false;
unsigned long count = 0;
unsigned long lastReconnectAttempt;
uint16_t MQTT_port = 1883; 
///////////////
#define CONNECT_OBD 1
COBDSPI obd;

////
//TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
////////////////////////////////////////////

////////////////////////////////////////////
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
    // Unlock your SIM card with a PIN
  //modem.simUnlock(UNLOCK_CODE);

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
  Serial.println(" IoT System Connected");
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
/////
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  delay(1000);
  digitalWrite(PIN_LED, LOW);
  //Serial.begin(9600);
    SerialMon.begin(115200);

  //config OBD interface
  byte ver = obd.begin();
  Serial.print("OBD Firmware Version ");
  Serial.println(ver);
  //Config Modem interface
  SerialAT.begin(115200,SERIAL_8N1,16,17,false);//hardware UART #1 in Esp32
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
////
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
void test_MQTT_cayenne(){
  int value; 
  while(true) {

  for (int i=0;i<10;i++){
    value = i*100;
    Serial.print("send value: ");Serial.println(value);
    if(MQTT_pub_int(rpmTopic,value)) {
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
/////////////////////////////////////////
void loop() {
 // uint32_t ts = millis();
  digitalWrite(PIN_LED, HIGH);
  test_MQTT_cayenne();
  MQTT_processing();

#if CONNECT_OBD
  if (!connected) {
    digitalWrite(PIN_LED, HIGH);
    Serial.print("Connecting to OBD...");
    if (obd.init()) {
      Serial.println("OK");
      connected = true;
    } else {
      Serial.println();
    }
    digitalWrite(PIN_LED, LOW);
    return;
  }
#endif
  int value;
  Serial.print('[');
  Serial.print(millis());
  Serial.print("] #");
  Serial.print(count++);
#if CONNECT_OBD
  if (obd.readPID(PID_RPM, value)) {
    Serial.print(" RPM:");
    Serial.print(value);
    MQTT_pub_int(rpmTopic,value);
  }
  if (obd.readPID(PID_SPEED, value)) {
    Serial.print(" SPEED:");
    Serial.print(value);
  }
#endif
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

  delay(100);
}
///////////////////////////////////////////
