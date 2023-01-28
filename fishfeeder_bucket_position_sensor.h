#ifndef FISHFEEDER_BUCKET_POSITION_SENSOR_H
#define FISHFEEDER_BUCKET_POSITION_SENSOR_H

// this white LED illuminates the bucket LDR to sense the position
int white_LED_pin;
int bucket_position_light_sensor_pin;
int bucket_LDR_sensor_limit;

void bucket_position_sensor_setup(
        int _white_led_pin,
        int _bucket_position_light_sensor_pin,
        int _bucket_LDR_sensor_limit
){
    white_LED_pin=_white_led_pin;
    bucket_position_light_sensor_pin=_bucket_position_light_sensor_pin;
    bucket_LDR_sensor_limit=_bucket_LDR_sensor_limit;

    pinMode(white_LED_pin, OUTPUT);
    white_led_on();
}

void white_led_on(){
    digitalWrite(white_LED_pin, HIGH);
}

void white_led_off(){
    digitalWrite(white_LED_pin, LOW);
}

bool is_bucket_up(){
    if (analogRead(bucket_position_light_sensor_pin) > bucket_LDR_sensor_limit) {
        return true;
    } else {
        return false;
    }

}




#endif