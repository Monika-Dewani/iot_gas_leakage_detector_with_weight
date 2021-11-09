//This code is edited by Sachin Soni(techiesms) for a project tutorital on
//Controlling Appliances and monitoring sensor's data over Internet using Ubidots MQTT server
//The video is uploaded on youtube whose link is :- https://youtu.be/LvzCeBce2mU

/****************************************
 * Include Libraries
 ****************************************/
#include <HX711_ADC.h> //weight
#include <EEPROM.h>  //weight

//#include <ESP32Servo.h>

//Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32

//int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
//int servoPin = 2;
//#include <PubSubClient.h>
//#define WIFISSID "OPPO" // Put your WifiSSID here
//#define PASSWORD "12345678" // Put your wifi password here
//#define TOKEN "A1E-CdtUSAlEFwAznL4zva6Dtco3zaFN8h" // Put your Ubidots' TOKEN
//#define MQTT_CLIENT_NAME "RandomName" // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string; 
#include <HX711_ADC.h>
#include <EEPROM.h>

//HX711 constructor (dout pin, sck pin):
HX711_ADC LoadCell(14,12);

const int eepromAdress = 0;

long t;

 int buz = 25;  
 int temp=0; 
 //it should be a random and unique ascii string and different from all other devices
#define l1 5 // Set the GPIO26 as RELAY
#define pump 17 // Set the GPIO26 as RELAY
#define l3 16 // Set the GPIO26 as RELAY
/****************************************
 * Define Constants
 ****************************************/
#define VARIABLE_LABEL "weight" // Assing the variable label
#define VARIABLE_LABEL1 "gas"
#define VARIABLE_LABEL2 "temp"
//#define VARIABLE_LABEL3 "ph" // Assing the variable label


#define VARIABLE_LABEL_SUBSCRIBE "pump" // Assing the variable label
#define DEVICE_LABEL "esp32" // Assig the device label
const int rs = 18, en = 19, d4 = 4, d5 = 21, d6 = 22, d7 = 23;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


char mqttBroker[]  = "things.ubidots.com";
char payload[100];

char topicw[150];
char topicg[150];
//char topicph[150];
char topict[150];

char topicSubscribe[100];
// Space to store values to send
char str_sensor[10];

/****************************************
 * Auxiliar Functions
 ****************************************/
WiFiClient ubidots;
PubSubClient client(ubidots);



void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    
    // Attemp to connect
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
      client.subscribe(topicSubscribe);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}
void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  if (message == "0") {
    digitalWrite(pump, LOW);
  } else {
    digitalWrite(pump, HIGH);
  }
  
  Serial.write(payload, length);
  Serial.println();
}

/****************************************
 * Main Functions
 ****************************************/
void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
lcd.print("Gas Lkge Detc.");
delay(1000);
lcd.clear();
lcd.print("Gas Lkge Detc.");
delay(1000);
lcd.clear();
lcd.print("Gas Lkge Detc.");
delay(1000);
myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  

  WiFi.begin(WIFISSID, PASSWORD);
  // Assign the pin as INPUT 
  pinMode(l1, OUTPUT);
  pinMode(pump, OUTPUT);
 pinMode(l3, OUTPUT);
pinMode(buz, OUTPUT);
  Serial.println();
  Serial.print("Wait for WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);

    
  }
  
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqttBroker, 1883);
  client.setCallback(callback);

  sprintf(topicSubscribe, "/v1.6/devices/%s/%s/lv", DEVICE_LABEL, VARIABLE_LABEL_SUBSCRIBE);
  
  client.subscribe(topicSubscribe);
float calValue; // calibration value
  calValue = 696.0; // uncomment this if you want to set this value in the sketch 
  #if defined(ESP8266) 
  //EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
  #endif
  //EEPROM.get(eepromAdress, calValue); // uncomment this if you want to fetch the value from eeprom
  
  //Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println("Starting...");
  LoadCell.begin();
  long stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
  LoadCell.start(stabilisingtime);
  if(LoadCell.getTareTimeoutFlag()) {
    Serial.println("Tare timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    LoadCell.setCalFactor(calValue); // set calibration value (float)
    Serial.println("Startup + tare is complete");
    delay(3000);
  }
}

void loop() {
  if (!client.connected()) {
    client.subscribe(topicSubscribe);   
    reconnect();
  }
  ///////////////weight////////////////
  
LoadCell.update();
  sprintf(topicw, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"%s\":", VARIABLE_LABEL); // Adds the variable label
  //get smoothed value from data set
  if (millis() > t + 250) {
    float i = LoadCell.getData();
    Serial.print("Load_cell output val: ");
    Serial.println(i);
    t = millis();
     lcd.setCursor(10, 1);
lcd.print(" W=");
lcd.setCursor(12, 1);
lcd.print(i);
dtostrf(i, 4, 2, str_sensor);
//  
  sprintf(payload, "%s {\"value\": %s}}", payload, str_sensor); // Adds the value
  Serial.println("Weight data to Ubidots Cloud");
  client.publish(topicw, payload);
  client.loop();
  delay(1000);
  }

  //receive from serial terminal
  if (Serial.available() > 0) {
    float i;
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  //check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
//  /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/


 sprintf(topict, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"%s\":", VARIABLE_LABEL2);
   
int sensorValue = analogRead(32);
    float temp = sensorValue * (5.0 / 4096.0);
     temp = temp*100 -2;
     Serial.print("T:- ");
  Serial.println(temp);
  lcd.setCursor(6, 1);
lcd.print("T=");
lcd.setCursor(8, 1);
lcd.print(temp);
delay(200);
dtostrf(temp, 4, 2, str_sensor);
//  
  sprintf(payload, "%s {\"value\": %s}}", payload, str_sensor); // Adds the value
  Serial.println("Temp data to Ubidots Cloud");
  client.publish(topict, payload);
  client.loop();
  delay(1000);
  if (temp>45)
{

digitalWrite(pump, 1);
digitalWrite(buz, 1);
delay(200);
digitalWrite(buz, 0);
delay(200);

}
else {

digitalWrite(pump, 0);

}
  

////////////Gas////////////////
sprintf(topicg, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"%s\":", VARIABLE_LABEL1);
   
int gas = analogRead(33);
//moisture=map(moisture,4095,0,0,4095);
Serial.print("Value of gas Sensor is:- ");
Serial.println(gas);
lcd.setCursor(0, 1);
lcd.print("G=");
lcd.setCursor(2, 1);
lcd.print(gas/10);

dtostrf(gas, 4, 2, str_sensor);
 sprintf(payload, "%s {\"value\": %s}}", payload, str_sensor); // Adds the value
  Serial.println("Gas data to Ubidots Cloud");
  client.publish(topicg, payload);
  client.loop();
  delay(1000);
if (gas>300)
{
digitalWrite(buz, 1);
delay(200);
digitalWrite(buz, 0);
delay(200);
digitalWrite(l1, 1);
myservo.write(180);
digitalWrite(l3, 1);

}
else {
digitalWrite(l1, 0);
digitalWrite(l3, 0);
myservo.write(0);
}

}
