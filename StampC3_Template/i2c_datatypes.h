#ifndef I2C_DATATYPES_H
#define I2C_DATATYPES_H

/* 20/03/26: Starting work on setting up the i2c
 * 
 */

#pragma pack(push,1)

#define ROBOT_I2C_ADDR    0x08

// Flag values to change request context
#define REQUEST_NULL        0
#define REQUEST_SURFACE     1
#define REQUEST_BUMP        2
#define REQUEST_POSE        3
#define SET_POSE            4
#define SET_MOTORS          5
#define SET_BUZZER          6
#define SET_SENSOR_CONFIG   7

//
//
// Size: 1 byte
typedef struct { 
  uint8_t value;
} RequestType_t;

//
//
// Size: 10 byte
typedef struct {                    
  uint16_t  reading[5];          
} RobotSurface_t;

//
//
// Size: 10 byte
typedef struct {                    
  uint16_t  reading[2];             
} RobotBump_t;

//
//
// Size: 12 byte
typedef struct {
  float     x;                   
  float     y;                   
  float     theta;               
} RobotPose_t;

//
//
// Size: 8 byte
typedef struct {                  
  int32_t   count[2];             
} RobotEncoders_t;


//
//
// Size: 4 byte
typedef struct {                  
  int16_t   pwm[2];               
} RobotMotors_t;


//
//
// Size: 2 byte
#define EMIT_AUTO     0
#define EMIT_LINE     1
#define EMIT_BUMP     2
#define EMIT_OFF      3
#define MICROS 0
#define ADC    1
typedef struct { 
  uint8_t   mode;            
  uint8_t   emit;                 
} RobotSensorConfig_t;

//
//
// Size: 4 byte
typedef struct {
  uint16_t freq;
  uint16_t duration_ms;
} RobotBuzzer_t;
#pragma pack(pop)

#endif
