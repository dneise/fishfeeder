//letzte Änderung:  if (executed == false && lightVal > 500) von 300 auf 500 gesetzt, da es immer schon bei 400 war. scheint aber wohl netzteil abhängig zu sein.
//letzte Änderung 15.06.22:  Servo beschleunigt
#include <AccelStepper.h>
#include <Servo.h>
#include "WiFi.h"
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>                              // include NTPClient library
#include <time.h>                                   // time() ctime()

// Define NTP properties
#define NTP_ADDRESS  "de.pool.ntp.org"              // change this to whatever pool is closest (see ntp.org)
#define MY_TZ "CET-1CEST,M3.5.0/02,M10.5.0/03"      //Timezone
time_t now;                                         // this is the epoch
tm tm;                                              // the structure tm holds time information in a more convient way
String t = "";

#define dirPin 33
#define stepPin 32
#define motorInterfaceType 1
#define SERVO_PIN 26 // ESP32 pin GIOP26 connected to servo motor
String temp = "";  //temporär
String packet = "";
String received = "";
#define onoffpin 16  //servo on off pin
Servo servoMotor;

//WIFI
const char *ssid     = "secret_ssid";                                //your Wifi SSID
const char *password = "secret_password";
const char* deviceName = "secret_device_name";

String command;

const int MS1 = 14;
const int MS2 = 13;
const int MS3 = 12;
int MOTOR_STEPS = 200;

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

const int buttonPin = 4;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status
#define LED1  18
#define LED2  5
#define LED3  23  //white
const int sensorPin = 35;  //photoelectric pin
int LDRValue;  //LDR Value
int lightInit;  // initial value
int lightVal;   // light reading
bool nulled = false;
bool executed = false;
bool going = false;
bool activated = true;

unsigned long previousMillisLED = 0;                // will store last time LED3 was updated
unsigned long intervalLED = 45000;                   // interval at which to switch off LED3 (milliseconds)


const int g1 = 280;
const int g2 = 175;
const int g3 = 65;
const int g4 = -40;
const int g5 = -145;

int counteryellow = 0;

const int MotorStepyellow[5] = {g1, g2, g3, g4, g5};

const int b1 = -250;
const int b2 = -360;
const int b3 = -460;
const int b4 = -570;
const int b5 = -675;

int counterblue = 0;

const int MotorStepblue[5] = {b1, b2, b3, b4, b5};

const int s1 = -786;
const int s2 = 705;
const int s3 = 600;
const int s4 = 490;
const int s5 = 380;
int counterblack = 0;

// Variables for publish counter via mqtt

String counterblack_str;
String counterblue_str;
String counteryellow_str;
char black[5];
char blue[5];
char yellow[5];

const int MotorStepblack[5] = {s1, s2, s3, s4, s5};

char buffer[40]; // added for monitor reading


const char* mqtt_server = "192.168.178.44";
const int mqttPort = 1883;
const char* mqttUser = "secret_mqtt_user";
const char* mqttPassword = "secret_mqtt_password";
//mqtt ende

// WiFi connect timeout per AP. Increase when connecting takes longer.
const uint32_t connectTimeoutMs = 5000;
unsigned long previousMillis = 0;                   // will store last time updated
const long interval = 30000;                        // interval at which to run   muss 15000 sein
unsigned long previousMillisLED1 = 0;                // will store last time LED1 was updated
unsigned long intervalLED1 = 1000;                   // interval at which to blink (milliseconds)
int LED1State = LOW;


WiFiClient espClient;
PubSubClient client(espClient);


void nullposition() {
  //digitalWrite(LED3, HIGH); // 04062022
  stepper.enableOutputs();
  LDRValue = analogRead(A0); // read analog input pin 0
  lightVal = analogRead(sensorPin);
  Serial.println("check fork light barrier");
  Serial.println(lightVal);
  stepper.setMaxSpeed(200);
  if (executed == false && lightVal > 700) {
    stepper.runToNewPosition(-100);
    Serial.println("detection positive --> repositioning");
    executed = true;
  //  delay(2000);
  }


 lightVal = analogRead(sensorPin);
 LDRValue = analogRead(A0); // read analog input pin 0
  stepper.moveTo(4000);
  while (lightVal < 700 && nulled != true && LDRValue > 3000) {// Full speed up to 300
    stepper.run();
    lightVal = analogRead(sensorPin); // read the current light levels
LDRValue = analogRead(A0); // read analog input pin 0
  }

  stepper.stop(); // Stop as fast as possible: sets new target
  stepper.runToPosition();
  stepper.setCurrentPosition(0);  //set steppercounter to 0 aufter reaching endstop switch
  if (LDRValue > 3000) {
    Serial.println(LDRValue); // prints the value read
      Serial.println("moved on to null position"); // prints the value read
  nulled = true;

    Serial.println("null position set");
  Serial.println(stepper.currentPosition());
  }
  stepper.setMaxSpeed(200);
  stepper.disableOutputs();
LDRValue = analogRead(A0); // read analog input pin 0
  if (LDRValue < 3000) {
   Serial.println("light barrier interrupted");
  Serial.println(LDRValue); // prints the value read
  Serial.println("move to empty position again");
  moveemptyposition();
  stepper.moveTo(4000);
  }

  // delay(10);
}


void setup() {
  Serial.begin(115200);
  servoMotor.attach(SERVO_PIN);  // attaches the servo on ESP32 pin
  lightInit = analogRead(sensorPin);
  Serial.println(lightInit);
  pinMode(onoffpin, OUTPUT);
  digitalWrite(onoffpin, LOW);
  pinMode(buttonPin, INPUT); //initialize the pushbutton pin as an input
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, HIGH);
  stepper.setPinsInverted(false, false, true);
  stepper.setEnablePin(25);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);

  setup_wifi();
  client.setServer(mqtt_server, mqttPort);
  client.setCallback(callback);
  reconnect();
  configTzTime(MY_TZ, NTP_ADDRESS);
  ntp();
  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(200);   //(1000)
  stepper.setAcceleration(800);  //(1000)
  nullposition();

}


void callback(char* topic, byte* payload, unsigned int length) {
  String sTopic = String(topic);
  Serial.print("topic empfangen: ");
  Serial.println(String(topic));


  if (sTopic == "aqua/feeder") {
activated = true;
 digitalWrite(LED3, HIGH);
    temp = "";

    for (int i = 0; i < length; i++) {
      temp += ((char)payload[i]);
    }
    received = temp;
    Serial.println(received);
    moveposition();
  }


}

void setup_wifi() {


  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");
}
void reconnect() {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    void setup_wifi();
  }
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    char clientid[25];
    snprintf(clientid, 25, "WIFI-Display-%08X", "12345"); //this adds the mac address to the client for a unique id
    Serial.print("Client ID: ");
    Serial.println(clientid);
    if (client.connect(clientid)) {
      Serial.println("connected");


      client.subscribe("aqua/feeder");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 1 second before retrying
      delay(1000);
    }
  }
}
void moveposition() {

  if (received == "blue") {
    if ( counterblue < 5 ) {
      MOTOR_STEPS = MotorStepblue[counterblue++];
      sprintf ( buffer, "Counter: %d \t MotoStepValues blue: %d", counterblue, MOTOR_STEPS);
      Serial.println(buffer);
if (counterblue == 5) {
  counterblue = 0;
}
    }
    else {
      Serial.println("bullshit");
    }
  }


  if (received == "yellow") {

    MOTOR_STEPS = MotorStepyellow[counteryellow++];
    sprintf ( buffer, "Counter: %d \t MotoStepValues yellow: %d", counteryellow, MOTOR_STEPS);
    Serial.println(buffer);
if (counteryellow == 5) {
  counteryellow = 0;
}
  }
  if (received == "black") {

    MOTOR_STEPS = MotorStepblack[counterblack++];
    sprintf ( buffer, "Counter: %d \t MotoStepValues black: %d", counterblack, MOTOR_STEPS);
    Serial.println(buffer);
if (counterblack == 5) {
  counterblack = 0;
}
  }
  if (received != "black" && received != "yellow" && received != "blue" ) {
    // nulled = false;
    MOTOR_STEPS = 0;
  }
  nulled = false;
  executed = false;
  nullposition();
  executed = true;

  stepper.enableOutputs();
  stepper.setMaxSpeed(200);   //(1000)
  stepper.setAcceleration(800);  //(1000)
  //MOTOR_STEPS = command.toInt();

  if (nulled == true) {
    Serial.println("fahren");
    stepper.moveTo(MOTOR_STEPS);
    while (stepper.currentPosition() != MOTOR_STEPS) {
      stepper.run();
      //Serial.println(stepper.currentPosition());
    }
    nulled = false;

    //stepper.disableOutputs();
  }
 if (received == "black" || received == "yellow" || received == "blue" ) {
  empty();
 }
 else {
    nulled = false;
    executed = false;
  nullposition();
 }
}

void empty()
{
  digitalWrite(onoffpin, HIGH);
  // rotates from 110 degrees to 0 degrees
  /* for (int pos = 110; pos >= 0; pos -= 1) {
    servoMotor.write(pos);
    delay(20); // waits 15ms to reach the position - last value 20
  }
  for (int pos = 0; pos <= 110; pos += 1) {
    // in steps of 10 degree
    servoMotor.write(pos);
    delay(20); // waits 15ms to reach the position - last value 20
  }
  */
  for (int pos = 110; pos >= -50; pos -= 1) {
    servoMotor.write(pos);
    delay(1); // waits 15ms to reach the position - last value 20
  }

  for (int pos = -50; pos <= 110; pos += 1) {
    // in steps of 10 degree
    servoMotor.write(pos);
    delay(5); // waits 15ms to reach the position - last value 20
  }

   for (int pos = 110; pos >= -50; pos -= 1) {
    servoMotor.write(pos);
    delay(1); // waits 15ms to reach the position - last value 20
  }

  for (int pos = -50; pos <= 110; pos += 1) {
    // in steps of 10 degree
    servoMotor.write(pos);
    delay(5); // waits 15ms to reach the position - last value 20
  }

   for (int pos = 110; pos >= -50; pos -= 1) {
    servoMotor.write(pos);
    delay(3); // waits 15ms to reach the position - last value 20
  }

  for (int pos = -50; pos <= 110; pos += 1) {
    // in steps of 10 degree
    servoMotor.write(pos);
    delay(5); // waits 15ms to reach the position - last value 20
  }
  //delay(5000);

  moveemptyposition();
}

void moveemptyposition() {
   delay(500);
  stepper.enableOutputs();
  stepper.setMaxSpeed(200);   //(1000)
  stepper.setAcceleration(800);  //(1000)
  MOTOR_STEPS = stepper.currentPosition() - 350;
  stepper.moveTo(MOTOR_STEPS);
  while (stepper.currentPosition() != MOTOR_STEPS) {
    stepper.run();
   // Serial.println(stepper.currentPosition());

  }
  nulled = false;
  nullposition();
}

void loop() {

    unsigned long currentMillisLED = millis();

  if ((currentMillisLED - previousMillisLED > intervalLED)&& activated == true) {
    // save the last time you blinked the LED
    previousMillisLED = currentMillisLED;
  digitalWrite(LED3, LOW);
  digitalWrite(onoffpin, LOW);
  activated = false;
    }

  buttonState = digitalRead(buttonPin);
  if (Serial.available()) {

    command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("you have typed: ");
    Serial.println(command);
    if (command.equals("99")) {
      //Serial.println("go to null position");
      nulled = false;
      nullposition();

    }
    else if (command.equals("999")) {
      empty();
    }
    else  {
      moveposition();
    }

  }

  if (WiFi.status() == WL_CONNECTED) {                            //blink blue LED1 while wifi is connected

    unsigned long currentMillisLED1 = millis();

    if (currentMillisLED1 - previousMillisLED1 > intervalLED1) {
      // save the last time you blinked the LED1
      previousMillisLED1 = currentMillisLED1;
      LED1State = !LED1State;
      // if the LED1 is off turn it on and vice-versa:
      if (LED1State == HIGH) {
        digitalWrite(LED1, LOW);
        intervalLED1 = 3000;
      } else {
        digitalWrite(LED1, HIGH);
        intervalLED1 = 100;
      }
    }
  }

  if (buttonState == HIGH) {

counterblack = 0;
counterblue = 0;
counteryellow = 0;

    digitalWrite(LED2, HIGH);

  }
  else {
    digitalWrite(LED2, LOW);

  }


  reconnect();



    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {

      Serial.print("counteryellow jetzt: ");
      Serial.println(counteryellow);
      Serial.print("counterblack jetzt: ");
      Serial.println(counterblack);
      Serial.print("counterblue jetzt: ");
      Serial.println(counterblue);
      Serial.println("publish werte zu smartphone");

      lightVal = analogRead(sensorPin);
        Serial.println("check fork light barrier in loop");
  Serial.println(lightVal);
ntp();
//Preparing for mqtt send

    counteryellow_str = String(counteryellow); //converting ftemp (the float variable above) to a string
    counteryellow_str.toCharArray(yellow, counteryellow_str.length() + 1); //packaging up the data to publish to mqtt whoa...

    counterblack_str = String(counterblack); //converting Humidity (the float variable above) to a string
    counterblack_str.toCharArray(black, counterblack_str.length() + 1); //packaging up the data to publish to mqtt whoa...

    counterblue_str = String(counterblue); //converting Humidity (the float variable above) to a string
    counterblue_str.toCharArray(blue, counterblue_str.length() + 1); //packaging up the data to publish to mqtt whoa...

      client.publish("aqua/feeder/counteryellow", yellow);
      client.publish("aqua/feeder/counterblack", black);
      client.publish("aqua/feeder/counterblue", blue);
     // client.publish("aqua/feeder/zeit", t.c_str());

     previousMillis = currentMillis;
    }


  client.loop();

}


void ntp()
{
  //Seperate the components of the time/date for using it to trigger the event of setzero()".
  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  time(&now);                               // read the current time
  localtime_r(&now, &tm);                   // update the structure tm with the current time
 /* Serial.print("year:");
  Serial.print(tm.tm_year + 1900);          // years since 1900
  Serial.print("\tmonth:");
  Serial.print(tm.tm_mon + 1);              // January = 0 (!)
  Serial.print("\tday:");
  Serial.print(tm.tm_mday);                 // day of month
  Serial.print("\thour:");
  Serial.print(tm.tm_hour);                 // hours since midnight  0-23
  Serial.print("\tmin:");
  Serial.print(tm.tm_min);                  // minutes after the hour  0-59
  Serial.print("\tsec:");
  Serial.print(tm.tm_sec);                  // seconds after the minute  0-61*
  Serial.print("\twday");
  Serial.println(tm.tm_wday);               // days since Sunday 0-6
*/
  //  date = String(tm.tm_mday) + "." + String(tm.tm_mon + 1) + "." + String(tm.tm_year + 1900);
  // savedate = String(tm.tm_mon + 1) + String(tm.tm_year + 1900) + ".csv";
  t = String(tm.tm_hour) + ":" + String(tm.tm_min) + ":" + String(tm.tm_sec);

  if (tm.tm_isdst == 1)                     // Daylight Saving Time flag
    Serial.print("\tDST");
  else
    Serial.print("\tstandard");
  Serial.println();

}
