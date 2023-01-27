//letzte Änderung:  if (executed == false && lightVal > 500) von 300 auf 500 gesetzt, da es immer schon bei 400 war. scheint aber wohl netzteil abhängig zu sein.
//letzte Änderung 15.06.22:  Servo beschleunigt
#include <AccelStepper.h>
#include <Servo.h>
#include "WiFi.h"
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>                              // include NTPClient library
#include <time.h>                                   // time() ctime()

#define dirPin 33
#define stepPin 32
#define stepperEnablePin 25
// MS1 .. MS3 are also stepper pins TODO: explain
// this defined stepper resolution (how many degrees per step)
const int MS1 = 14;
const int MS2 = 13;
const int MS3 = 12;


#define motorInterfaceType 1

#define tippingServoControlPin 26 // ESP32 pin GIOP26 connected to servo motor
#define tippingServoOnOffPin 16  //servo on off pin

String temp = "";  //temporär
String packet = "";
String received = "";

Servo servoMotor;

//WIFI
const char *ssid     = "secret_ssid";                                //your Wifi SSID
const char *password = "secret_password";
const char* deviceName = "secret_device_name";

String command;

int MOTOR_STEPS = 200;

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

const int buttonPin = 4;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status
#define LED1  18  // this is a blue LED
#define LED2  5
// this white LED illuminates the bucket LDR (under bucketPositionSensorPin) to sense the position
#define LED_white  23
const int nullPositionPhotoElectricPin = 35;
#define bucketPositionSensorPin A0
int LDRValue;  //LDR Value
int lightInit;  // initial value
int lightVal;   // light reading

bool nulled = false;
bool executed = false;
bool going = false;
bool activated = true;

unsigned long previousMillisLED = 0;                // will store last time LED_white was updated
unsigned long intervalLED = 45000;                   // interval at which to switch off LED_white (milliseconds)


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
PubSubClient mqtt_client(espClient);

void network_led_on() {
    digitalWrite(LED1, LOW);
}
void network_led_off() {
    digitalWrite(LED1, HIGH);
}

void setup() {
    Serial.begin(115200);
    servoMotor.attach(tippingServoControlPin);
    lightInit = analogRead(nullPositionPhotoElectricPin);
    Serial.println(lightInit);

    pinMode(tippingServoOnOffPin, OUTPUT);
    digitalWrite(tippingServoOnOffPin, LOW);

    pinMode(buttonPin, INPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED_white, OUTPUT);

    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED_white, HIGH);

    stepper.setPinsInverted(false, false, true);
    stepper.setEnablePin(stepperEnablePin);
    pinMode(MS1, OUTPUT);
    pinMode(MS2, OUTPUT);
    pinMode(MS3, OUTPUT);
    digitalWrite(MS1, HIGH);
    digitalWrite(MS2, HIGH);
    digitalWrite(MS3, HIGH);

    wait_until_wifi_is_connected();
    mqtt_client.setServer(mqtt_server, mqttPort);
    mqtt_client.setCallback(callback);
    reconnect_mqtt();

    stepper.setMaxSpeed(200);
    stepper.setAcceleration(800);
    nullposition();
}

void wait_until_wifi_is_connected() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the WiFi network");
}

void reconnect_mqtt() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected!");
        wait_until_wifi_is_connected();
    }

    // Loop until we're reconnected
    while (!mqtt_client.connected()) {
        Serial.print("Attempting MQTT connection...");

        // ---------- TODO -------- this seems static, why not use static string?
        char clientid[25];
        snprintf(clientid, 25, "WIFI-Display-%08X", "12345"); //this adds the mac address to the mqtt_client for a unique id
        Serial.print("Client ID: ");
        Serial.println(clientid);
        // ------------------------------

        if (mqtt_client.connect(clientid)) {
            Serial.println("connected");
            mqtt_client.subscribe("aqua/feeder");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqtt_client.state());
            Serial.println(" try again in 1 seconds"); // TODO use same constant here and below.
            delay(1000);
        }
    }
}

bool is_fork_light_barrier_blocked(){
    lightVal = analogRead(nullPositionPhotoElectricPin);
    if (lightVal > 700){
        return true;
    } else {
        return false;
    }
}

bool is_bucket_up(){
    LDRValue = analogRead(bucketPositionSensorPin);
    if (LDRValue > 3000) {
        return true;
    } else {
        return false;
    }
}

void nullposition() {
    stepper.enableOutputs();
    Serial.print("check fork light barrier. Value: ");
    Serial.println(lightVal);

    stepper.setMaxSpeed(200);
    if (executed == false && is_fork_light_barrier_blocked()) {
        stepper.runToNewPosition(-100);  // move backward, from current position
        Serial.println("detection positive --> repositioning");
        executed = true;
    }

    stepper.moveTo(4000); // move forward from current position
    while (!is_fork_light_barrier_blocked() && nulled != true && is_bucket_up()) {
        stepper.run();
    }

    stepper.stop();
    stepper.runToPosition();  // TODO: find out what this does?
    stepper.setCurrentPosition(0);  //set internal stepper counter to 0

    if (is_bucket_up()) {
        Serial.println("moved on to null position");
        nulled = true;
        Serial.println("null position set");
        Serial.println(stepper.currentPosition());
    }

    stepper.disableOutputs();

    if (!is_bucket_up()) {
        Serial.println("light barrier interrupted");
        Serial.println("bringing bucket up again");
        bring_bucket_into_upright_position();
    }
}

void bring_bucket_into_upright_position() {
    delay(500);  // TODO do we really need this?  maybe this is better closer to the stop of the stepper
    stepper.enableOutputs();
    stepper.setMaxSpeed(200);
    stepper.setAcceleration(800);
    int CONTROL_POSITION = stepper.currentPosition() - 350;
    stepper.moveTo(CONTROL_POSITION);
    while (stepper.currentPosition() != CONTROL_POSITION) {
        stepper.run();
    }
    nulled = false;
    nullposition();
}

void callback(char* topic, byte* payload, unsigned int length) {
    String sTopic = String(topic);
    Serial.print("topic empfangen: ");
    Serial.println(String(topic));

    if (sTopic == "aqua/feeder") {
        activated = true;
        digitalWrite(LED_white, HIGH);
        temp = "";

        for (int i = 0; i < length; i++) {
            temp += ((char)payload[i]);
        }

        received = temp;
        Serial.println(received);
        move_to_position_according_to_command();
    }
}

void move_to_position_according_to_command() {

    if (received == "blue") {
        MOTOR_STEPS = MotorStepblue[counterblue++];
        sprintf ( buffer, "Counter: %d \t MotoStepValues blue: %d", counterblue, MOTOR_STEPS);
        Serial.println(buffer);
        if (counterblue == 5) {
            counterblue = 0;
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
        MOTOR_STEPS = 0;
    }
    nulled = false;
    executed = false;
    nullposition();
    executed = true;

    stepper.enableOutputs();
    stepper.setMaxSpeed(200);
    stepper.setAcceleration(800);

    if (nulled == true) {
        Serial.println("fahren");
        stepper.moveTo(MOTOR_STEPS);
        while (stepper.currentPosition() != MOTOR_STEPS) {
            stepper.run();
        }
        nulled = false;
    }
    if (received == "black" || received == "yellow" || received == "blue" ) {
        empty_bucket();
    } else {
        nulled = false;
        executed = false;
        nullposition();
    }
}

void empty_bucket()
{
    digitalWrite(tippingServoOnOffPin, HIGH);
    for (int iteration = 0; iteration < 3; iteration ++){
        for (int pos = 110; pos >= -50; pos -= 1) {
            servoMotor.write(pos);
            delay(1);
        }
        for (int pos = -50; pos <= 110; pos += 1) {
            servoMotor.write(pos);
            delay(5);
        }
    }
    digitalWrite(tippingServoOnOffPin, LOW);
    bring_bucket_into_upright_position();
}


void loop() {
    unsigned long currentMillisLED = millis();

    if ((currentMillisLED - previousMillisLED > intervalLED)&& activated == true) {
        previousMillisLED = currentMillisLED;
        digitalWrite(LED_white, LOW);
        digitalWrite(onoffpin, LOW);
        activated = false;
    }


    if (Serial.available()) {
        command = Serial.readStringUntil('\n');
        command.trim();
        Serial.print("you have typed: ");
        Serial.println(command);
        if (command.equals("null")) {
            nulled = false;
            nullposition();
        } else if (command.equals("empty")) {
            empty_bucket();
        } else {
            move_to_position_according_to_command();
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

    buttonState = digitalRead(buttonPin);
    if (buttonState == HIGH) {
        counterblack = 0;
        counterblue = 0;
        counteryellow = 0;
        digitalWrite(LED2, HIGH);
    } else {
        digitalWrite(LED2, LOW);
    }
    reconnect_mqtt();

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        Serial.print("counteryellow jetzt: ");
        Serial.println(counteryellow);
        Serial.print("counterblack jetzt: ");
        Serial.println(counterblack);
        Serial.print("counterblue jetzt: ");
        Serial.println(counterblue);
        Serial.println("publish werte zu smartphone");

        lightVal = analogRead(nullPositionPhotoElectricPin);
        Serial.println("check fork light barrier in loop");
        Serial.println(lightVal);

        mqtt_client.publish("aqua/feeder/counteryellow", String(counteryellow).c_str());
        mqtt_client.publish("aqua/feeder/counterblack", String(counterblack).c_str());
        mqtt_client.publish("aqua/feeder/counterblue", String(counterblue).c_str());

        previousMillis = currentMillis;
    }
    mqtt_client.loop();
}