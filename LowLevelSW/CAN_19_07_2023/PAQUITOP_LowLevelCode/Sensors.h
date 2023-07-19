/**
 * Sensors Class to manage CANBUS lowlevel sensors.
 */

#ifndef SENSORS_CLASS_H
#define SENSORS_CLASS_H

#include "FlexCAN_T4.h"

#define CAN_BAUDRATE 125000
#define MY_ID 0xAA
#define REQ_TIMEOUT 200
#define THRESHOLD_SEND_TIMEOUT 5000

enum {
  sensor1   = 0x01,
  sensor2,
  sensor3,
  sensor4,
  sensor5,
  sensor6,
  sensor7,
  sensor8
};

enum {
  SET_ID_CAN = 0x12,
  SET_THRESHOLD,
  DIST_REQUEST,
  ALARM_YELLOW,
  ALARM_RED,
  ALARM_LASER,
  DIST_ANS
};

typedef struct {
  uint16_t yellowThreshold, redThreshold, laserThreshold, alarmTimeout;
} threshold_t;

typedef struct { 
  uint16_t distLaser, distSonar;
  bool error = false;
} dist_t;


typedef void (*callback_t)(uint8_t sensor, uint8_t threshold, uint16_t distance);

class Sensors {
  public:
    Sensors(threshold_t threshold, callback_t c);
    void begin();
    void setThreshold(threshold_t threshold);
    void update();
    dist_t requestDistance(uint8_t sensorId);
    threshold_t getThreshold();
    
  private:
    threshold_t threshold;
    callback_t callback;
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> canBus;
    void sendThreshold();
    long time;

};

#endif
