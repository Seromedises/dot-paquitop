#include "Sensors.h"
#include <stdio.h>

/**
 * Class Sensors constructor.
 */
Sensors::Sensors(threshold_t threshold, callback_t c) 
    : threshold(threshold), callback(c) {}


/**
 * Initialization function.
 * CANBUS setup and sending of thresholds to every sensor.
 */
void Sensors::begin() {  
  canBus.begin();
  canBus.setBaudRate(CAN_BAUDRATE);
  time = millis();
  sendThreshold();
}

/**
 * Update function, to be called for every loop cycle.
 * Checks if any alarm has arrived and calls callback function.
 */
void Sensors::update() {
  CAN_message_t msg;

  if(canBus.read(msg)) {
    if( (msg.buf[1] == ALARM_RED || msg.buf[1] == ALARM_YELLOW || msg.buf[1] == ALARM_LASER)) {
      callback(msg.buf[0], msg.buf[1], msg.buf[2] << 8 | msg.buf[3]);
    }
  }

  long actTime = millis();
  if(actTime - time > THRESHOLD_SEND_TIMEOUT) {
    sendThreshold();
    time = actTime;
  }
}

/**
 * Function to set threshold for all sensors.
 * Updates the private attribute and sends the threshold 
 * to all the sensors.
 */
void Sensors::setThreshold(threshold_t threshold) {
    this->threshold = threshold;

    sendThreshold();
}

/**
 * Private function to send the threshold to a specific sensor.
 */
void Sensors::sendThreshold() {
  CAN_message_t msg;

  msg.buf[0] = SET_THRESHOLD;
  msg.buf[1] = threshold.yellowThreshold >> 8;
  msg.buf[2] = threshold.yellowThreshold;
  msg.buf[3] = threshold.redThreshold >> 8;
  msg.buf[4] = threshold.redThreshold;
  msg.buf[5] = threshold.laserThreshold >> 8;
  msg.buf[6] = threshold.laserThreshold;
  msg.buf[7] = threshold.alarmTimeout / 20;
  msg.len = 8;

  canBus.write(msg);
}

/**
 * Function to request measured distance to a specific sensor.
 * Sends a request message and then waits up to REQ_TIMEOUT milliseconds
 * for the answer, else returns error constant DIST_ERR.
 * 
 * WARNING: 
 * the function is blocking, if alarm messages arrive between the request
 * and the response, they would be discarded.
 */
dist_t Sensors::requestDistance(uint8_t sensorId) {
  CAN_message_t msg;
  dist_t distance;

  msg.id = sensorId;
  msg.buf[0] = DIST_REQUEST;
  msg.len = 1;

  canBus.write(msg);

  long startTime = millis();
  bool flag = true;

  while(flag) {
    if(canBus.read(msg) && msg.id == MY_ID && msg.buf[1] == DIST_ANS && msg.buf[0] == sensorId) {
      flag = false;
    }

    if(millis() - startTime > REQ_TIMEOUT) {
      distance.error = true;
      return distance;
    }
  }

  distance.distLaser = msg.buf[2] << 8 | msg.buf[3];
  distance.distSonar = msg.buf[4] << 8 | msg.buf[5];

  return distance;
}

/**
 * Getter function to return actual threshold.
 */
threshold_t Sensors::getThreshold() { 
    return threshold;
}