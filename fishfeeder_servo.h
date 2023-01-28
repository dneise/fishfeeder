#ifndef FISHFEEDER_SERVO_H
#define FISHFEEDER_SERVO_H

#include <Servo.h>

int tippingServoControlPin;
int tippingServoOnOffPin;

Servo servoMotor;

void servo_setup(int _tippingServoOnOffPin, int _tippingServoControlPin){

    tippingServoControlPin=_tippingServoControlPin;
    tippingServoOnOffPin=_tippingServoOnOffPin;

    servoMotor.attach(tippingServoControlPin);
    pinMode(tippingServoOnOffPin, OUTPUT);
    digitalWrite(tippingServoOnOffPin, LOW);
}

void servo_empty_bucket(int num_iterations, int start_pos, int end_pos)
{
    int forward_delay = 1;
    int backward_delay = 5;

    digitalWrite(tippingServoOnOffPin, HIGH);
    for (int iteration = 0; iteration < num_iterations; iteration ++){
        for (int pos = start_pos; pos >= end_pos; pos -= 1) {
            servoMotor.write(pos);
            delay(forward_delay);
        }
        for (int pos = end_pos; pos <= start_pos; pos += 1) {
            servoMotor.write(pos);
            delay(backward_delay);
        }
    }
    digitalWrite(tippingServoOnOffPin, LOW);
}


#endif