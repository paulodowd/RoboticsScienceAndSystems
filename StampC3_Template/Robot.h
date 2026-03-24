#ifndef ROBOT_H
#define ROBOT_H

#include <Wire.h>
#include "i2c_datatypes.h"

class Robot_c {

  public:
    RobotSensorConfig_t sensor_config;
    RobotEncoders_t     encoders;
    RobotSurface_t      surface;
    RobotMotors_t       motors;
    RobotBuzzer_t       buzzer;
    RobotPose_t         pose;

    Robot_c() {
      memset( &sensor_config, 0, sizeof( sensor_config ));
      memset( &encoders, 0, sizeof( encoders ));
      memset( &surface, 0, sizeof(surface));
      memset( &motors, 0, sizeof( motors ));
      memset( &buzzer, 0, sizeof( buzzer ));
      memset( &pose, 0, sizeof( pose ));  
      
    }

    void initialise() {
      Wire.setPins(9, 8); // SDA, SCL
      Wire.begin();
      Wire.setClock(400000);
      setupLED();
      pinMode(3, INPUT_PULLUP);
      
    }

    bool getButton() {
      return !digitalRead(3);
    }

    void setSensorConfig( uint8_t mode, uint8_t emit ) {

      if( mode > ADC ) return;
      if( emit > EMIT_OFF ) return;

      RequestType_t request;
      request.value = SET_SENSOR_CONFIG;
      Wire.beginTransmission( ROBOT_I2C_ADDR );
      Wire.write( (byte*)&request, sizeof( request ));
      Wire.endTransmission();

      RobotSensorConfig_t new_config;
      new_config.emit = emit;
      new_config.mode = mode;
      
      Wire.beginTransmission( ROBOT_I2C_ADDR );
      Wire.write( (byte*)&new_config, sizeof( new_config ));
      Wire.endTransmission();
    }

    void getSurfaceSensors() {

      RequestType_t request;
      request.value = REQUEST_SURFACE;
      Wire.beginTransmission( ROBOT_I2C_ADDR );
      Wire.write( (byte*)&request, sizeof( request ));
      Wire.endTransmission();

      Wire.requestFrom( ROBOT_I2C_ADDR, sizeof( surface ));
      Wire.readBytes((byte*)&surface, sizeof( surface ) );
    }
    void printAllSensors() {
      // Surface in microseconds

    }

    void getPose() {

      RequestType_t request;
      request.value = REQUEST_POSE;
      Wire.beginTransmission( ROBOT_I2C_ADDR );
      Wire.write( (byte*)&request, sizeof( request ));
      Wire.endTransmission();

      Wire.requestFrom( ROBOT_I2C_ADDR, sizeof( pose ));
      Wire.readBytes((byte*)&pose, sizeof( pose ) );

    }
    void printTelemetry() {

    }

    void setMotorPWM( float l, float r ) {

      RequestType_t request;
      request.value = SET_MOTORS;
      Wire.beginTransmission( ROBOT_I2C_ADDR );
      Wire.write( (byte*)&request, sizeof( request ));
      Wire.endTransmission();

      motors.pwm[1] = (int16_t)l;
      motors.pwm[0] = (int16_t)r;

      Wire.beginTransmission( ROBOT_I2C_ADDR );
      Wire.write( (byte*)&motors, sizeof( motors ));
      Wire.endTransmission();
    }

    void setPose( float x, float y, float theta ) {

      RequestType_t request;
      request.value = SET_POSE;
      Wire.beginTransmission( ROBOT_I2C_ADDR );
      Wire.write( (byte*)&request, sizeof( request ));
      Wire.endTransmission();

      RobotPose_t new_pose;
      new_pose.x = x;
      new_pose.y = y;
      new_pose.theta = theta;
      
      Wire.beginTransmission( ROBOT_I2C_ADDR );
      Wire.write( (byte*)&new_pose, sizeof( new_pose ));
      Wire.endTransmission();
      
    }

    void playTone( uint16_t freq, uint16_t duration_ms ) {

      RequestType_t request;
      request.value = SET_BUZZER;
      Wire.beginTransmission( ROBOT_I2C_ADDR );
      Wire.write( (byte*)&request, sizeof( request ));
      Wire.endTransmission();

      RobotBuzzer_t new_tone;
      new_tone.freq = freq;
      new_tone.duration_ms = duration_ms;
      
      Wire.beginTransmission( ROBOT_I2C_ADDR );
      Wire.write( (byte*)&new_tone, sizeof( new_tone ));
      Wire.endTransmission();
      
    }


    // Paul: I've extracted this from Freenove_WS2812_Lib_for_ESP32
    //       so that students do not need to go through the steps of
    //       installing a library just to run 1 RGB LED
    // https://github.com/Freenove/Freenove_WS2812_Lib_for_ESP32/tree/master
    void setLED( uint8_t r, uint8_t g, uint8_t b, uint8_t brightness ) {
      r = (uint8_t)constrain(r, 0, 255);
      g = (uint8_t)constrain(g, 0, 255);
      b = (uint8_t)constrain(b, 0, 255);

      brightness = constrain(brightness, 0, 255);

      uint8_t  p[3];
      p[LED_rOffset] = r * brightness / 255;
      p[LED_gOffset] = g * brightness / 255;
      p[LED_bOffset] = b * brightness / 255;

      uint32_t color = p[0] << 16 | p[1] << 8 | p[2] ;
      for (int bit = 0; bit < 24; bit++) {
        if (color & (1 << (23 - bit))) {
          led_data[bit].level0 = 1;
          led_data[bit].duration0 = 8;
          led_data[bit].level1 = 0;
          led_data[bit].duration1 = 4;
        } else {
          led_data[bit].level0 = 1;
          led_data[bit].duration0 = 4;
          led_data[bit].level1 = 0;
          led_data[bit].duration1 = 8;
        }
      }
      updateLED();
    }



  private:

    // Paul: I've extracted this from Freenove_WS2812_Lib_for_ESP32
    //       so that students do not need to go through the steps of
    //       installing a library just to run 1 RGB LED
    // https://github.com/Freenove/Freenove_WS2812_Lib_for_ESP32/tree/master
    uint8_t LED_rOffset;
    uint8_t LED_gOffset;
    uint8_t LED_bOffset;

    // type = 0x12 (01 00 10

    rmt_data_t  led_data[24]; // 1 led = 24 bits
    bool setupLED() {

      // Setup for this led type
      LED_rOffset = (0x12 >> 4 ) & 0x03;
      LED_gOffset = (0x12 >> 2 ) & 0x03;
      LED_bOffset = 0x12 & 0x03;
      rmt_reserve_memsize_t rmt_mem = RMT_MEM_NUM_BLOCKS_1;
      if (rmtInit(2, RMT_TX_MODE, rmt_mem, 10000000) == false) {
        return false;
      }

      for ( int i = 0; i < 24; i++ ) {
        led_data[i].level0    = 1;
        led_data[i].duration0 = 4;
        led_data[i].level1    = 0;
        led_data[i].duration1 = 8;
      }

      // We'll set an initial colour, so we can see if the
      // device is turned on
      setLED( 255, 128, 178, 20 );

      return true;
    }

    esp_err_t updateLED() {
      return rmtWrite(2, led_data, 24, RMT_WAIT_FOR_EVER);
    }


};

#endif
