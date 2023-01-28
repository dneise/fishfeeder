// TODO this does not work yet!!!!! finalize !! you lazy guy !!!
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
}

void white_led_on(){
    std::cout << "white led on" << std::endl;
}

void white_led_off(){
    std::cout << "white led off" << std::endl;
}

bool is_bucket_up(){
    return true;
}