#include "rvrclass.h"
#include <ESP8266WiFi.h>

extern WiFiClient debug_client;
extern void tcpsendall(WiFiClient myclient, char* buf);
bool debug = false;

bool sphero_rvr::init_port(Stream* prt) {
  port = prt; portinit = true;
  xpos = 0.0; ypos = 0.0; angle = 0.0;
  sintheta = 0.0; costheta = 1.0;
  Nangle = 0.0;
  sprintf(BTname, "RVR01");
  return true;
}

byte sphero_rvr::readbyte() {
  byte inbyte;
  while(port->available() <= 0) {}
  // delay(mdelay);
  inbyte = port->read();
  if(debug) {
    char bb[4];
    sprintf(bb, "%02X ", inbyte);
    tcpsendall(debug_client, bb);
    // delay(100);
  }
  if(inbyte == ESCAPE) {
    while(port->available() <= 0) {}
    inbyte = port->read();
    inbyte = inbyte | 0x88;
  }
  return inbyte;
}

byte sphero_rvr::read_message() {

    uint8_t bite;
    int checksum = 0;
    uint8_t buffer[256];
    byte cmd_id;
    union flag inflags;

    /* Start the message */
    bite = readbyte();
    if (bite == -1) return NULL;
    while (bite != MESSAGE_START) {
        bite = readbyte();
    }

    /* Flags */
    bite = readbyte();
    inflags.allbits = bite;
    checksum += bite;

    /* Rest of the header */
    if (inflags.flag_bits.has_target == 1) {
      bite = readbyte(); // TargetID
      checksum += bite;
    }

    if (inflags.flag_bits.has_source == 1) {
      bite = readbyte(); // SourceID
      checksum += bite;
    }

    bite = readbyte(); // DeviceID
    checksum += bite;

    bite = readbyte(); // CommandID
    checksum += bite;
    cmd_id = bite;

    bite = readbyte(); // Sequence number
    checksum += bite;

    /* error code byte */
    if (inflags.flag_bits.is_response) {
      bite = readbyte(); // Error code (could return this instead of cid?)
      checksum += bite;
    }

    /* And now the payload */
    uint8_t recv_length = 0;
    bite = readbyte();
    while (bite != MESSAGE_END) {
      buffer[recv_length] = bite;
      checksum += bite;
      recv_length++;
      bite = readbyte();
    }
    recv_length--;   // account for the received checksum

    /* Checksum */
    checksum = checksum - bite & 0xFF ^ 0xFF;
    if (bite != checksum) {
      return 0x00;
    } else {
      inpl_len = recv_length;
      for (int i=0; i<recv_length; i++) {
        inpayload[i] = buffer[i];
      }
      return cmd_id;
    }
}

void sphero_rvr::handle_message() {
  if(portinit && port->available()) {
    byte in_cid = read_message();
    if(in_cid == STREAM_NOTIFY) { // Location xy
      uint32_t integer; // ipayload[0] is config token number (0x01)
      integer = inpayload[1] << 24 | inpayload[2] << 16 |
        inpayload[3] << 8 | inpayload[4];
      // normalize
      float xtemp, ytemp;
      xtemp =
        (integer - 0.5*UINT32_MAX)/UINT32_MAX;
      xtemp = xtemp * (LOCATOR_DATA_MAX - LOCATOR_DATA_MIN);
      // xpos += LOCATOR_DATA_MIN;
      if(inpl_len == 9) {
        integer = inpayload[5] << 24 | inpayload[6] << 16 |
        inpayload[7] << 8 | inpayload[8];
        // normalize
        ytemp =
          (integer - 0.5*UINT32_MAX)/UINT32_MAX;
        ytemp = ytemp * (LOCATOR_DATA_MAX - LOCATOR_DATA_MIN);
        // ypos += LOCATOR_DATA_MIN;
      }
      xpos = costheta*xtemp + sintheta*ytemp;
      ypos = costheta*ytemp - sintheta*xtemp;
      xpos += xoff; ypos += yoff;
    }
  }
}

void sphero_rvr::writebytes(byte* barray, byte size) {
  for(int ii = 0; ii < size; ii++) {
    if(barray[ii] == MESSAGE_START) {
      port->write(ESCAPE); // delay(mdelay);
      port->write(ESC_SOP); // delay(mdelay);
    } else if(barray[ii] == MESSAGE_END) {
      port->write(ESCAPE); // delay(mdelay);
      port->write(ESC_EOP); // delay(mdelay);
    } else if(barray[ii] == ESCAPE) {
      port->write(ESCAPE); // delay(mdelay);
      port->write(ESC_ESC); // delay(mdelay);
    } else {
      port->write(barray[ii]); // delay(mdelay);
    }
    if(debug) {
      char bb[4];
      sprintf(bb, "%02X ", barray[ii]);
      tcpsendall(debug_client, bb);
      // delay(100);
    }
  }
}

bool sphero_rvr::poweroff() {
  if(portinit) {
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(arr_poweroff, 7);
    port->write(MESSAGE_END); // delay(mdelay);
    return true;
  } else { return false; }
}

bool sphero_rvr::wake() {
  if(portinit) {
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(arr_wake, 7);
    port->write(MESSAGE_END); // delay(mdelay);
    return true;
  } else { return false; }
}

bool sphero_rvr::sleep() {
  if(portinit) {
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(arr_sleep, 7);
    port->write(MESSAGE_END); // delay(mdelay);
    return true;
  } else { return false; }
}

byte sphero_rvr::get_battery_life() {
  if(portinit) {
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(arr_get_batt, 6);
    port->write(MESSAGE_END); // delay(mdelay);
    while(read_message() != GET_BATT_PERCENT) {}
    return inpayload[0];
  } else { return 0x00; }
}

bool sphero_rvr::get_BT_name() {
  if(portinit) {
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(arr_BT_advert_name, 6);
    port->write(MESSAGE_END); // delay(mdelay);
    while(read_message() != GET_BT_NAME) {}
    inpayload[inpl_len] = 0x00;
    sprintf(BTname, "%s", inpayload+1);
    return true;
  } else { return false; }
}

bool sphero_rvr::all_leds(byte red, byte green, byte blue) {
  if(portinit) {
    byte checksum = all_leds_head_chksm;
    byte RGB[3] = {red, green, blue};
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(all_leds_head, 10);
    for(int ii = 0; ii<10; ii++) {
      writebytes(RGB, 3);
    }
    checksum += 10*(red+green+blue);
    checksum = checksum & 0xFF ^ 0xFF;
    port->write(checksum); // delay(mdelay);
    port->write(MESSAGE_END);
    return true;
  } else { return false; }
}

bool sphero_rvr::left_leds(byte red, byte green, byte blue) {
  if(portinit) {
    byte RGB[3] = {red, green, blue};
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(led_header, 6);
    writebytes(LEFT_HEADLIGHT, 4);
    writebytes(RGB, 3);
    byte checksum = (LR_CHKSM1 + red + green + blue) & 0xFF ^ 0xFF;
    port->write(checksum); // delay(mdelay);
    port->write(MESSAGE_END); // delay(mdelay);
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(led_header, 6); writebytes(LEFT_BRAKELIGHT, 4);
    writebytes(RGB, 3);
    checksum = (LR_CHKSM2 + red + green + blue) & 0xFF ^ 0xFF;
    port->write(checksum); // delay(mdelay);
    port->write(MESSAGE_END);
    return true;
  } else { return false; }
}

bool sphero_rvr::right_leds(byte red, byte green, byte blue) {
  if(portinit) {
    byte RGB[3] = {red, green, blue};
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(led_header, 6); writebytes(RIGHT_HEADLIGHT, 4);
    writebytes(RGB, 3);
    byte checksum = (LR_CHKSM2 + red + green + blue) & 0xFF ^ 0xFF;
    port->write(checksum); // delay(mdelay);
    port->write(MESSAGE_END); // delay(mdelay);
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(led_header, 6); writebytes(RIGHT_BRAKELIGHT, 4);
    writebytes(RGB, 3);
    checksum = (LR_CHKSM1 + red + green + blue) & 0xFF ^ 0xFF;
    port->write(checksum); // delay(mdelay);
    port->write(MESSAGE_END);
    return true;
  } else { return false; }
}

bool sphero_rvr::set_raw_motors(byte lmode, byte lspeed,
                                byte rmode, byte rspeed) {
  // The motors timeout after two seconds. Renew command!
  if(portinit) {
    if(lmode > 2) lmode = 0;
    if(rmode > 2) rmode = 0;
    byte checksum =
      (raw_motor_chksum + lmode + lspeed + rmode + rspeed) & 0xFF ^ 0xFF;
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(raw_motor_head, 6);
    byte motor_modes[4] = {lmode, lspeed, rmode, rspeed};
    writebytes(motor_modes, 4);
    port->write(checksum); // delay(mdelay);
    port->write(MESSAGE_END);
    return true;
  } else { return false; }
}

bool sphero_rvr::reset_yaw() {
  if(portinit) {
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(arr_reset_yaw, 6);
    port->write(MESSAGE_END); // delay(mdelay);
    return true;
  } else { return false; }
}

bool sphero_rvr::reset_xy() {
  if(portinit) {
    // xpos = xoff; ypos = yoff;
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(arr_reset_xy, 6);
    port->write(MESSAGE_END); // delay(mdelay);
    return true;
  } else { return false; }
}

bool sphero_rvr::drive(byte speed, uint16_t heading, byte dir) {
  if(portinit) {
    byte heading_MSB = heading >> 8;
    byte heading_LSB = heading & 0xFF;
    byte checksum =
      (drive_head_chksum + speed
       + heading_MSB + heading_LSB + dir) & 0xFF ^ 0xFF;
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(drive_head, 6);
    byte rest_payload[5] =
      {speed, heading_MSB, heading_LSB, dir, checksum};
    writebytes(rest_payload, 5);
    port->write(MESSAGE_END); // delay(mdelay);
    return true;
  } else { return false; }
}

bool sphero_rvr::config_xy_stream() {
  if(portinit) {
    if(stream_active) { stop_xy_stream(); }
    clear_stream();
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(arr_xy_stream_config, 10);
    port->write(MESSAGE_END); // delay(mdelay);
    stream_configed = true;
    return true;
  } else { return false; }
}

bool sphero_rvr::start_xy_stream(uint16_t period) {
  if(portinit && !stream_active) {
    if(!stream_configed) { config_xy_stream(); }
    byte msbits = (period & 0xFF00) >> 8;
    byte lsbits = (period & 0x00FF);
    byte checksum =
      (xy_stream_head_chksum + msbits + lsbits) & 0xFF ^ 0xFF;
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(start_xy_stream_head, 5);
    byte rest_payload[3] = {msbits, lsbits, checksum};
    writebytes(rest_payload, 3);
    port->write(MESSAGE_END); // delay(mdelay);
    stream_active = true;
    return true;
  } else { return false; }
}

bool sphero_rvr::stop_xy_stream() {
  if(portinit && stream_active) {
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(arr_stop_xy_stream, 6);
    port->write(MESSAGE_END); // delay(mdelay);
    stream_active = false;
    return true;
  } else { return false; }
}

bool sphero_rvr::clear_stream() {
  if(portinit && !stream_active) {
    port->write(MESSAGE_START); // delay(mdelay);
    writebytes(arr_clear_stream, 6);
    port->write(MESSAGE_END); // delay(mdelay);
    return true;
  } else { return false; }
}

bool sphero_rvr::read_xy() {
  if(portinit && !stream_active) {
    start_xy_stream(5); // ms
    // Read twice to clear buffer
    while(read_message() != STREAM_NOTIFY) {}
    while(read_message() != STREAM_NOTIFY) {}
    stop_xy_stream();
    uint32_t integer; // ipayload[0] is config token number (0x01)
    integer = inpayload[1] << 24 | inpayload[2] << 16 |
      inpayload[3] << 8 | inpayload[4];
    // normalize
    float xtemp, ytemp;
    xtemp =
      (integer - 0.5*UINT32_MAX)/UINT32_MAX;
    xtemp = xtemp * (LOCATOR_DATA_MAX - LOCATOR_DATA_MIN);
    if(inpl_len == 9) {
      integer = inpayload[5] << 24 | inpayload[6] << 16 |
        inpayload[7] << 8 | inpayload[8];
      // normalize
      ytemp =
        (integer - 0.5*UINT32_MAX)/UINT32_MAX;
      ytemp = ytemp * (LOCATOR_DATA_MAX - LOCATOR_DATA_MIN);
    }
    xpos = costheta*xtemp + sintheta*ytemp;
    ypos = costheta*ytemp - sintheta*xtemp;
    xpos += xoff; ypos += yoff;
    return true;
  } else { return false; }
}
