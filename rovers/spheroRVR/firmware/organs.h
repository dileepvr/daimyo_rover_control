#pragma once

#include <math.h>
#include <MLX90393.h>
#include "rvrclass.h"


/* BEGIN COMPASS STUFF */
// const int compass_ID = 2;  // Which Sparkfun compass to use

MLX90393 mlx;
MLX90393::txyz mlxdata; //Create a structure, called data, of four floats (t, x, y, and z)
float mxoff;
float mxscale;
float myoff;
float myscale;
float myxscale;
uint8_t readlim = 3;
const float degconv = 180.0/M_PI;

float getorient() {
  float ang = 0.0;
  for(long thisread = 0; thisread < readlim; thisread++) {
    mlx.readData(mlxdata);
    ang += atan2((mlxdata.y-myoff)*myxscale, mlxdata.x-mxoff);
  }
  return degconv*ang/readlim;
}
/* END COMPASS STUFF */

/* BEGIN SONIC RANGE FINDER STUFF */
const uint8_t TRIG_PIN = D7;
const uint8_t ECHO_PIN = D6;
const unsigned int MAX_DIST = 23200; // us (23200 us == 400 cm)
const float meter_conv = 0.01/58.2;  // Convert us round-trip time
float dist_m() {
  unsigned int pulse_width;
  // Returns distance of obstacle in meters
  delay(1);
  /* delayMicroseconds(2); */
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  pulse_width = pulseIn(ECHO_PIN, HIGH, MAX_DIST);
  if(pulse_width > MAX_DIST) { return 4.0; } // 4 meters (out of range)
  else { return pulse_width*meter_conv - 0.06; }

}
/* END SONIC RANGE FINDER STUFF */

/* BEGIN COLLISION SWITCHES */
const uint8_t LEFT_SW = D3;
const uint8_t RIGHT_SW = D4;
unsigned long transition_time = 0;
const uint8_t debouce_time = 5; // milliseconds
bool col_flag = false;
bool col_flag2 = false;
extern sphero_rvr rvr;
extern int cmd_state;
void ICACHE_RAM_ATTR collision() {
  /* if(millis() - transition_time > debouce_time) { */
  /*   transition_time = millis(); */
    col_flag = !(digitalRead(LEFT_SW) && digitalRead(RIGHT_SW));
    if(col_flag) {
      col_flag2 = true;
      cmd_state = 0;
      rvr.drive(64, 0, 1);
    }
  /* } */
}
/* END COLLISION SWITCHES */
