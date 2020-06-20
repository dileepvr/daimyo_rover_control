#ifndef SPHRVR_H
#define SPHRVR_H

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#ifndef UINT16_MAX
#define UINT16_MAX 65535
#endif

#define MESSAGE_START 0x8D
#define MESSAGE_END   0xD8
#define GET_BATT_PERCENT 0x10
#define GET_BT_NAME 0x05
#define ESCAPE   0xAB
#define ESC_ESC  0x23
#define ESC_SOP  0x05
#define ESC_EOP  0x50

#define STREAM_NOTIFY 0x3D
#define LOCATOR           0x2226
#define LOCATOR_DATA_SIZE          0x02
#define LOCATOR_DATA_MIN           -16000.0
#define LOCATOR_DATA_MAX           16000.0
#define LOCATOR_FLAGS_BITMASK_NONE           0x00
#define LOCATOR_FLAGS_BITMASK_AUTO_CALIBRATE 0x01
#define SET_LOCATOR_FLAGS                               0x17

union flag {
    uint8_t allbits;
    struct bits {
        unsigned int is_response:1;
        unsigned int response_requested:1;
        unsigned int response_requested_if_error:1;
        unsigned int is_activity:1;
        unsigned int has_target:1;
        unsigned int has_source:1;
        unsigned int UNUSED:1;
        unsigned int has_more_flags:1;
    } flag_bits;
};

class sphero_rvr{

private:
  Stream *port;
  bool portinit = false;
  bool stream_configed = false;
  byte inpayload[256], inpl_len = 0;
  int mdelay = 1;

  // Power management constants
  byte arr_poweroff[7] =
    {0x1A, 0x01, 0x13, 0x00, 0x88, 0x05, 0x44};
  byte arr_wake[7] =
    {0x3E, 0x11, 0x01, 0x13, 0x0D, 0x00, 0x8F};
  byte arr_sleep[7] =
    {0x3E, 0x11, 0x01, 0x13, 0x01, 0x00, 0x9B};
  byte arr_get_batt[6] =
    {0x1A, 0x01, 0x13, 0x10, 0x00, 0xC1};

  // LED constants
  byte all_leds_head[10] =
    {0x3E, 0x11, 0x01, 0x1A, 0x1A, 0, 0x3F, 0xFF, 0xFF, 0xFF};
  byte all_leds_head_chksm = 0xC0; // Sum of all_leds_head
  byte led_header[6] = {0x3E, 0x11, 0x01, 0x1A, 0x1A, 0x00};
  byte RIGHT_HEADLIGHT[4] = {0x00, 0x00, 0x00, 0x07};
  byte LEFT_HEADLIGHT[4] = {0x00, 0x00, 0x00, 0x38};
  byte LEFT_BRAKELIGHT[4] = {0x07, 0x00, 0x00, 0x00};
  byte RIGHT_BRAKELIGHT[4] = {0x38, 0x00, 0x00, 0x00};
  byte LR_CHKSM1 = 0xBC; byte LR_CHKSM2 = 0x8B;

  // Driving constants
  byte raw_motor_head[6] = {0x3E, 0x12, 0x01, 0x16, 0x01, 0x00};
  byte raw_motor_chksum = 0x68; // Sum of raw_motor_head
  byte drive_head[6] = {0x3E, 0x12, 0x01, 0x16, 0x07, 0x00};
  byte drive_head_chksum = 0x6E; // Sum of drive_head

  // Sensor constants
  byte arr_BT_advert_name[6] =
    {0x1A, 0x02, 0x19, 0x05, 0x00, 0xC5};
  byte arr_reset_yaw[6] =
    {0x18, 0x02, 0x16, 0x06, 0x00, 0xC9};
    //    {0x3E, 0x12, 0x01, 0x16, 0x06, 0x00, 0x92};
  byte arr_reset_xy[6] =
    {0x18, 0x02, 0x18, 0x13, 0x00, 0xBA};
    //    {0x3E, 0x12, 0x01, 0x16, 0x13, 0x00, 0x85};
  byte arr_xy_stream_config[10] =
    {0x18, 0x02, 0x18, 0x39, 0x00, 0x01, 0x00, 0x06, 0x02, 0x8B};
  byte start_xy_stream_head[5] =
    {0x18, 0x02, 0x18, 0x3A, 0x00};
  byte xy_stream_head_chksum = 0x6C; // Sum of start_xy_stream_head
  byte arr_stop_xy_stream[6] =
    {0x18, 0x02, 0x18, 0x3B, 0x00, 0x92};
  byte arr_clear_stream[6] =
    {0x18, 0x02, 0x18, 0x3C, 0x00, 0x91};

public:

  char BTname[256];
  float xpos, ypos, angle, Nangle; // Nangle is north from global x-axis
  float xoff, yoff; // offsets
  //  float theta; // angle (radians) between map axes and rover internal axes
  float sintheta, costheta;
  float Dxy = 0.01; // precision meters
  float Dangle = 0.1; // precision degrees
  float maxvel = 1.2; // 1.2 m/s ~\equiv 255 byte value
  byte defvel = 64; // 100 byte value ~\equiv 0.4 m/s
  bool heartbeat_flag = false;
  bool stream_active = false;
  int heartbeat_period = 500; // milliseconds
  unsigned long lastheart = 0;
  uint8_t state = 0; // {0: st_IDLE, 1:st_MOVE}

  bool init_port(Stream* prt);

  // IO
  byte readbyte();
  byte read_message();
  void handle_message();
  void writebytes(byte* barray, byte size);

  // Power management
  bool poweroff();
  bool wake();
  bool sleep();
  byte get_battery_life();
  bool get_BT_name();

  // LED control
  bool all_leds(byte red, byte green, byte blue);
  bool all_leds_off() { return all_leds(0, 0, 0); }
  bool left_leds(byte red, byte green, byte blue);
  bool right_leds(byte red, byte green, byte blue);

  // Motor control and driving
  bool set_raw_motors(byte lmode, byte lspeed, byte rmode, byte rspeed);
  // dir = 0 (fwd) or 1 (reverse)
  bool drive(byte speed, uint16_t heading, byte dir);
  bool face(uint16_t heading) { return drive(0, heading, 0); }
  bool halt() {
    state = 0;
    return set_raw_motors(0, 0, 0, 0);
  }

  // Sensors
  bool reset_yaw();
  bool reset_xy();
  bool config_xy_stream();
  bool start_xy_stream(uint16_t period);
  bool stop_xy_stream();
  bool clear_stream();
  bool read_xy(); // quick toggle stream service

};

#endif
