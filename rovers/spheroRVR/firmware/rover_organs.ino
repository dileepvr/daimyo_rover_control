#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include "organs.h"
#include "rvrclass.h"
#include <math.h>
#include <EEPROM.h>
#include <stdlib.h>

sphero_rvr rvr;

ESP8266WiFiMulti wifiMulti;     // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'
WiFiClient client;
WiFiServer debug_server(8082);
WiFiClient debug_client;
int daimyo_port = 8081;
char daimyo_server[32] = "192.168.0.17";

char daimyo_server[32] = "192.168.4.1";
const char ssid[32] = "rover_wifi";
const char password[64] = "rover_wifi_password";

float radconv = M_PI/180.0; // Convert degrees to radians
byte targvel = 128;
float targdist = 0.0;
float targang = 0.0;
int targdir = 0; // 0: CCW, 1: CW
int cmd_state = 0; // 1:FWD, 2:BWD, 3:TURN, 4:ATURN, 5:GOTO, 6:CFWD, 7:CBWD, 8:CTURN, 9:OBS, 10:POBS
int goto_state = 0; // 1: first turn, 2: traverse, 3: final turn
int pobs_state = 0; // 1: first turn, 2: traverse
float targx = 0.0, targy = 0.0, targang2 = 0.0; // for GOTO
float olddistsq = 0.0; // for GOTO
float gotopres = 0.0005;
float diffangpres = 3.0;

byte defangvel = 120;

char indebugdata[256];
char outdebugdata[256];
char indaimyodata[256];
char outdaimyodata[256];
uint8_t incoming_indx = 0;
uint8_t indaimyo_indx = 0;
bool incoming_flag = false;
bool outdebug_flag = false;
bool indaimyo_flag = false;
bool outdaimyo_flag = false;

bool calib_flag = false;
unsigned long freshjointime = millis();
int IDsend = 0;

bool OTA_flag = true;

char rovername[18] = "Ronin01";
char roverversion[18] = "0";

int compass_ID;

float xmin = 100.0, xmax = -100.0, ymin = 100.0, ymax = -100.0, zmin, zmax;

volatile float angle, distobs;

struct {
  char rovername[16] = "";
  char roverversion[16] = "";
  int compassID;
  float xmin, xmax, ymin, ymax;
} eeprom_data;


void setup() {

  uint addr = 0;

  EEPROM.begin(512);
  EEPROM.get(addr, eeprom_data); // Both EEPROM.get() and EEPROM.put() need to be preceeded by an EEPROM.begin(512) each
  // eeprom_data.compassID = 4;
  // EEPROM.put(addr, eeprom_data);
  // EEPROM.commit();

  strcpy(rovername, eeprom_data.rovername);
  strcpy(roverversion, eeprom_data.roverversion);
  compass_ID = eeprom_data.compassID;
  mxoff = (eeprom_data.xmax+eeprom_data.xmin)*0.5;
  mxscale = 2.0/(eeprom_data.xmax-eeprom_data.xmin);
  myoff = (eeprom_data.ymax+eeprom_data.ymin)*0.5;
  myscale = 2.0/(eeprom_data.ymax-eeprom_data.ymin);
  myxscale = myscale/mxscale;

  Serial.begin(115200);
  delay(10);

  rvr.init_port(&Serial);

  // BEGIN COMPASS
  Wire.begin(D2, D1); /* join i2c bus with SDA=D2 and SCL=D1 of NodeMCU */
  byte status = mlx.begin(0, 0, D5);
  delay(10);
  angle = getorient();
  // END COMPASS

  // BEGIN ULTRASONIC RANGE FINDER
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  // END ULTRASONIC RANGE FINDER

  // BEGIN COLLISION SWITCHES
  pinMode(LEFT_SW, INPUT_PULLUP);
  pinMode(RIGHT_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_SW), collision, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SW), collision, CHANGE);
  // END COLLISION SWITCHES

  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(ssid, password);

  int i = 0;
  while (wifiMulti.run() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(250);
  }
  ArduinoOTA.setHostname(rovername);
  ArduinoOTA.setPassword("bluemodethorlabs");

  ArduinoOTA.onStart([]() {
  });
  ArduinoOTA.onEnd([]() {
      all_leds_red();
      if(client.connected()) { // Disconnect from daimyo
        tcpsendall(client, "<BYE>");
        client.stop();
      }
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  debug_server.begin();
  if (!client.connect(daimyo_server, daimyo_port)) {
    //    Serial.println("connection failed");
    all_leds_red();
    return;
  }
  else {
    all_leds_blue();
    IDsend = 1;
    freshjointime = millis();
  }

  mlx.readData(mlxdata); //Read the values from the sensor
  xmin = 100.0; xmax = -100.0;
  ymin = 100.0; ymax = -100.0;
  zmin = mlxdata.z; zmax = mlxdata.z;

  /* rvr.wake(); */
  /* rvr.all_leds(0x40, 0, 0); */

}

unsigned long previousTime = millis();

const unsigned long interval = 100;


void loop() {

  if(IDsend > 0) { // Freshly joined daimyo. Send ID and POS
    if(millis() - freshjointime > 1000) {
      if(IDsend == 1) {
        IDsend = 2;
        freshjointime = millis();
        sprintf(outdaimyodata, "<MYID,%s,%s>", rovername, roverversion);
        tcpsendall(client, outdaimyodata);
      } else if(IDsend == 2) {
        IDsend = 0;
        rvr.angle = rvr.Nangle + getorient();
        if(rvr.angle < 0.0) { rvr.angle += 360; }
        sprintf(outdaimyodata, "<MYPOS,%.3f,%.3f,%.1f>",
                rvr.xpos, rvr.ypos, rvr.angle);
        tcpsendall(client, outdaimyodata);
      }
    }
  }

  if(col_flag2) { // front limit switches triggered
    col_flag2 = false;
    //    cmd_state = 0;
    //    rvr.drive(64, 0, 1);
    rvr.read_xy();
    all_leds_red();
    if(client.connected()) {
      sprintf(outdaimyodata, "<COL,%.3f,%.3f>", rvr.xpos, rvr.ypos);
      tcpsendall(client, outdaimyodata);
    }
    if(cmd_state == 10 && targdist > 0.2) {
      delay((int)((targdist-0.2)*4/1.2)*100);
    } else { delay(100); }
    rvr.drive(0, 0, 0); // rvr.halt(); // roll stop
    rvr.state = 0;
    if(client.connected()) {
      tcpsendall(client, "<ACK,0>");
      rvr.angle = rvr.Nangle + getorient();
      if(rvr.angle < 0.0) { rvr.angle += 360; }
      sprintf(outdaimyodata, "<MYPOS,%.3f,%.3f,%.1f>",
              rvr.xpos, rvr.ypos, rvr.angle);
      tcpsendall(client, outdaimyodata);
    }
    cmd_state = 0;
  }

  rvr.handle_message();
  if(OTA_flag) { ArduinoOTA.handle(); }
  unsigned long curtime = millis();
  if(curtime - previousTime > interval) {
    TCPdebughandle();
    TCPdaimyohandle();
    previousTime = curtime;
  }

  if(rvr.heartbeat_flag && client.connected()) {
    if(curtime - rvr.lastheart > rvr.heartbeat_period) {
      rvr.lastheart = curtime;
      rvr.angle = rvr.Nangle + getorient();
      if(rvr.angle < 0.0) { rvr.angle += 360; }
      if(!rvr.stream_active) { rvr.read_xy(); };
      sprintf(outdaimyodata, "<MYPOS,%.3f,%.3f,%.1f>",
              rvr.xpos, rvr.ypos, rvr.angle);
      tcpsendall(client, outdaimyodata);
    }
  }

  if(cmd_state != 0) { cmd_exec(); }

  if(calib_flag) { // Digital compass MLX90393 calibration
    cmd_state = 0;
    rvr.halt();
    rvr.reset_yaw();
    rvr.face(5);
    delay(100);
    mag_calib();
  }
}

bool daimyo_debug_flag = false;

void TCPdaimyohandle() {
  if(client.connected()) {
    while(client.available() > 0) {
      char inchar = client.read();
      if(inchar == '>') {
        indaimyo_flag = true;
        indaimyodata[indaimyo_indx] = '\0';
        indaimyo_indx = 0;
        if(daimyo_debug_flag && debug_client) {
          strcpy(outdebugdata, indaimyodata);
          tcpsendall(debug_client, outdebugdata);
        }
        break;
      } else if(inchar == '<') {
        indaimyo_indx = 0;
      } else {
        indaimyo_flag = false;
        indaimyodata[indaimyo_indx++] = inchar;
        if (indaimyo_indx >= 256) { indaimyo_indx = 0; }
      }
    }
    if(indaimyo_flag) {
      indaimyo_flag = false;
      if(strcmp(indaimyodata, "DIE") == 0) {
        outdaimyo_flag = false;
        tcpsendall(client, "<BYE>");
        rvr.halt();
        all_leds_red();
        client.stop();
      } else if(strcmp(indaimyodata, "ID") == 0) {
        sprintf(outdaimyodata, "<MYID,%s,%s>", rovername, roverversion);
        outdaimyo_flag = true;
      } else if(strcmp(indaimyodata, "POS") == 0) {
        rvr.read_xy();
        rvr.angle = rvr.Nangle + getorient();
        if(rvr.angle < 0.0) { rvr.angle += 360; }
        sprintf(outdaimyodata, "<MYPOS,%.3f,%.3f,%.1f>",
                rvr.xpos, rvr.ypos, rvr.angle);
        outdaimyo_flag = true;
      } else if(strcmp(indaimyodata, "PRES") == 0) {
        sprintf(outdaimyodata, "<MYPRES,%.3f,%.1f>",
                rvr.Dxy, rvr.Dangle);
        outdaimyo_flag = true;
      } else if(strcmp(indaimyodata, "MAXVEL") == 0) {
        sprintf(outdaimyodata, "<MYMAXV,%.3f>", rvr.maxvel);
        outdaimyo_flag = true;
      } else if(strcmp(indaimyodata, "SILENT") == 0) {
        rvr.heartbeat_flag = false;
        sprintf(outdaimyodata, "<ACK,>");
        outdaimyo_flag = true;
      } else if(strcmp(indaimyodata, "HALT") == 0) {
        rvr.halt();
        all_leds_blue();
        cmd_state = 0;
        sprintf(outdaimyodata, "<ACK,0>"); // st_IDLE state
        outdaimyo_flag = true;
      } else {
        char* separator = strchr(indaimyodata, ',');
        if(separator == 0) { // No fields
        } else { // with fields
          *separator = '\0'; ++separator;
          if(strcmp(indaimyodata, "SETPOS") == 0) { // set position
            if(rvr.state != 0) { rvr.halt(); }
            char* inx = separator; separator = strchr(inx, ',');
            *separator = '\0'; ++separator; rvr.xoff = atof(inx);
            char* iny = separator; separator = strchr(iny, ',');
            *separator = '\0'; ++separator; rvr.yoff = atof(iny);
            char* inNang = separator;
            if(*inNang != '\0') { rvr.Nangle = atof(inNang); }
            rvr.reset_yaw(); rvr.reset_xy();
            float theta = fmod(90.0 - rvr.Nangle - getorient(), 360.0);
            theta = theta*radconv;
            rvr.sintheta = sin(theta); rvr.costheta = cos(theta);
            rvr.read_xy();
            rvr.angle = rvr.Nangle + getorient();
            if(rvr.angle < 0.0) { rvr.angle += 360; }
            sprintf(outdaimyodata, "<ACK,0><MYPOS,%.3f,%.3f,%.1f>",
                    rvr.xpos, rvr.ypos, rvr.angle);
            outdaimyo_flag = true;
          } else if(strcmp(indaimyodata, "SETPRES") == 0) { // set precision
            char* inDxy = separator; separator = strchr(inDxy, ',');
            *separator = '\0'; ++separator; rvr.Dxy = atof(inDxy);
            char* inDang = separator;
            if(*inDang != '\0') { rvr.Dangle = atof(inDang); }
            sprintf(outdaimyodata, "<ACK,%d>", rvr.state);
            outdaimyo_flag = true;
          } else if(strcmp(indaimyodata, "HEART") == 0) { // turn on heartbeat
            rvr.heartbeat_flag = true;
            rvr.lastheart = 0;
            char* inprd = separator;
            if(*inprd != '\0') { rvr.heartbeat_period = atoi(inprd); }
            else { rvr.heartbeat_period = 500; } // ms
            sprintf(outdaimyodata, "<ACK,>");
            outdaimyo_flag = true;
          } else if(strcmp(indaimyodata, "FWD") == 0) { // forward a distance
            char* indist = separator; separator = strchr(indist, ',');
            *separator = '\0'; ++separator; targdist = atof(indist);
            char* invel = separator; separator = strchr(invel, ',');
            if(separator > invel) {
              separator = '\0'; float fvel = 255*atof(invel)/1.2;
              if(fvel <=255.0) { targvel = (int)fvel & 0xFF; }
              else { targvel = 255; }
            } else { targvel = rvr.defvel; } // rover default velocity
            targdist = targdist - 12.8/targvel; // overshoot
            if(targdist < 0.0) { targdist = 0.0; }
            // ignoring timeout field completely
            cmd_state = 1; // FWD
            rvr.halt(); reset_rvr_offsets();
            if(rvr.stream_active) {
              rvr.stop_xy_stream(); rvr.start_xy_stream(50); // 50 ms
            }
            rvr.state = 1; // st_MOVING
            all_leds_green();
            rvr.drive(targvel, 0, 0); // FWD
            sprintf(outdaimyodata, "<ACK,1>");
            outdaimyo_flag = true;
          } else if(strcmp(indaimyodata, "BWD") == 0) { // backward a distance
            char* indist = separator; separator = strchr(indist, ',');
            *separator = '\0'; ++separator; targdist = (-1.0)*atof(indist);
            char* invel = separator; separator = strchr(invel, ',');
            if(separator > invel) {
              separator = '\0'; float fvel = 255*atof(invel)/1.2;
              if(fvel <=255.0) { targvel = (int)fvel & 0xFF; }
              else { targvel = 255; }
            } else { targvel = rvr.defvel; }
            targdist = targdist + 12.8/targvel; // overshoot
            if(targdist > 0.0) { targdist = 0.0; }
            // ignoring timeout field completely
            cmd_state = 2; // BWD
            rvr.halt(); reset_rvr_offsets();
            if(rvr.stream_active) {
              rvr.stop_xy_stream(); rvr.start_xy_stream(50); // 50 ms
            }
            rvr.state = 1; // st_MOVING
            all_leds_green();
            rvr.drive(targvel, 0, 1); // BWD
            sprintf(outdaimyodata, "<ACK,1>");
            outdaimyo_flag = true;
          } else if(strcmp(indaimyodata, "CFWD") == 0) { // continuous forward
            char* invel = separator; separator = strchr(invel, ',');
            if(separator > invel) {
              separator = '\0'; float fvel = 255*atof(invel)/1.2;
              if(fvel <=255.0) { targvel = (int)fvel & 0xFF; }
              else { targvel = 255; }
            } else { targvel = rvr.defvel; }
            // ignoring timeout field completely
            cmd_state = 0;
            rvr.halt(); reset_rvr_offsets();
            rvr.state = 1; // st_MOVING
            all_leds_green();
            rvr.drive(targvel, 0, 0); // CFWD
            sprintf(outdaimyodata, "<ACK,1>");
            outdaimyo_flag = true;
          } else if(strcmp(indaimyodata, "CBWD") == 0) { // cont. backward
            char* invel = separator; separator = strchr(invel, ',');
            if(separator > invel) {
              separator = '\0'; float fvel = 255*atof(invel)/1.2;
              if(fvel <=255.0) { targvel = (int)fvel & 0xFF; }
              else { targvel = 255; }
            } else { targvel = rvr.defvel; }
            // ignoring timeout field completely
            cmd_state = 0;
            rvr.halt(); reset_rvr_offsets();
            rvr.state = 1; // st_MOVING
            all_leds_green();
            rvr.drive(targvel, 0, 1); // CBWD
            sprintf(outdaimyodata, "<ACK,1>");
            outdaimyo_flag = true;
          } else if(strcmp(indaimyodata, "TURN") == 0) { // TURN absolute
            char* inang = separator; separator = strchr(inang, ',');
            *separator = '\0'; ++separator;
            targang = fmod(atof(inang), 360.0);
            /* if(targang > 180.0) { targang -= 360.0;  } */
            if(targang < 0.0) { targang += 360.0; }
            if(abs(targang) == 180.0) { targang = 180.0; }
            // ignoring timeout field completely
            cmd_state = 3;
            rvr.halt(); reset_rvr_offsets();
            rvr.state = 1; // st_MOVING
            all_leds_green();
            sprintf(outdaimyodata, "<ACK,%d>", rvr.state);
            outdaimyo_flag = true;
          } else if(strcmp(indaimyodata, "ATURN") == 0) { // TURN relative
            char* indir = separator; separator = strchr(indir, ',');
            *separator = '\0'; ++separator; targdir = atoi(indir);
            char* inang = separator; separator = strchr(inang, ',');
            *separator = '\0';
            targang = fmod(atof(inang), 360.0);
            /* if(targang > 180.0) { targang -= 360.0;  } */
            if(targang < 0.0) { targang += 360.0; }
            if(abs(targang) == 180.0) { targang = 180.0; }
            // ignoring timeout field completely
            cmd_state = 4;
            rvr.halt(); rvr.read_xy();
            rvr.xoff = rvr.xpos; rvr.yoff = rvr.ypos;
            rvr.reset_yaw(); rvr.reset_xy();
            float theta = fmod(90.0 - 2*rvr.Nangle + rvr.angle, 360.0);
            theta = theta*radconv;
            rvr.sintheta = sin(theta); rvr.costheta = cos(theta);
            rvr.angle = fmod(rvr.Nangle + getorient(), 360.0);
            if(rvr.angle < 0.0) { rvr.angle += 360; }
            if(targdir == 0) { // CCW
              targang = fmod(rvr.angle + targang, 360.0);
            } else { // CW
              targang = fmod(rvr.angle - targang, 360.0);
            }
            /* if(targang > 180.0) { targang -= 360.0;  } */
            if(targang < 0.0) { targang += 360.0; }
            if(abs(targang) == 180.0) { targang = 180.0; }
            rvr.state = 1; // st_MOVING
            all_leds_green();
            sprintf(outdaimyodata, "<ACK,%d>", rvr.state);
            outdaimyo_flag = true;
          } else if(strcmp(indaimyodata, "CTURN") == 0) { // continuous TURN
            char* indir = separator; separator = strchr(indir, ',');
            *separator = '\0'; ++separator; targdir = atoi(indir);
            // ignoring timeout field completely
            cmd_state = 8;
            rvr.halt();
            rvr.state = 1; // st_MOVING
            all_leds_green();
            sprintf(outdaimyodata, "<ACK,%d>", rvr.state);
            outdaimyo_flag = true;
          } else if(strcmp(indaimyodata, "GOTO") == 0) { // GOTO
            char* inx = separator; separator = strchr(inx, ',');
            *separator = '\0'; ++separator; targx = atof(inx);
            char* iny = separator; separator = strchr(iny, ',');
            *separator = '\0'; ++separator; targy = atof(iny);
            rvr.halt(); rvr.read_xy();
            targang = degconv*atan2(targy-rvr.ypos, targx-rvr.xpos);
            targang = fmod(targang, 360.0);
            if(targang < 0.0) { targang+= 360.0; }
            char* invel = separator; separator = strchr(invel, ',');
            /* if(separator > invel) { */
            /*   separator = '\0'; float fvel = 255*atof(invel)/1.2; */
            /*   if(fvel <=255.0) { targvel = (int)fvel & 0xFF; } */
            /*   else { targvel = 255; } */
            /* } else { targvel = rvr.defvel; } */
            targvel = rvr.defvel;
            ++separator;
            char* inang = separator; separator = strchr(inang, ',');
            if(separator > inang) {
              separator = '\0'; targang2 = fmod(atof(inang), 360.0);
            } else { targang2 = targang; }
            /* if(targang2 > 180.0) { targang2 -= 360.0;  } */
            if(targang2 < 0.0) { targang2 += 360.0; }
            if(abs(targang2) == 180.0) { targang2 = 180.0; }
            // ignoring timeout field completely
            cmd_state = 5; // GOTO
            goto_state = 1; // GOTO_first_turn
            reset_rvr_offsets();
            if(rvr.stream_active) {
              rvr.stop_xy_stream(); rvr.start_xy_stream(50); // 50 ms
            }
            rvr.state = 1; // st_MOVING
            all_leds_green();
            sprintf(outdaimyodata, "<ACK,1>");
            outdaimyo_flag = true;
          } else if(strcmp(indaimyodata, "OBS") == 0) { // obstacle dist
            rvr.halt();
            rvr.angle = fmod(rvr.Nangle + getorient(), 360.0);
            if(rvr.angle < 0.0) { rvr.angle += 360; }
            char* inang = separator; separator = strchr(inang, ',');
            if(separator > inang) {
              separator = '\0'; targang = fmod(atof(inang), 360.0);
            } else { targang = rvr.angle; }
            /* if(targang > 180.0) { targang -= 360.0;  } */
            if(targang < 0.0) { targang += 360.0; }
            if(abs(targang) == 180.0) { targang = 180.0; }
            // ignoring timeout field completely
            cmd_state = 9; // OBS
            all_leds_green();
            rvr.state = 1;
            sprintf(outdaimyodata, "<ACK,1>");
            outdaimyo_flag = true;
          } else if(strcmp(indaimyodata, "POBS") == 0) { // move to obstacle
            rvr.halt();
            char* indist = separator; separator = strchr(indist, ',');
            *separator = '\0'; ++separator; targdist = atof(indist);
            rvr.angle = fmod(rvr.Nangle + getorient(), 360.0);
            if(rvr.angle < 0.0) { rvr.angle += 360; }
            char* inang = separator; separator = strchr(inang, ',');
            if(separator > inang) {
              separator = '\0'; targang = fmod(atof(inang), 360.0);
            } else { targang = rvr.angle; }
            /* if(targang > 180.0) { targang -= 360.0;  } */
            if(targang < 0.0) { targang += 360.0;  }
            if(abs(targang) == 180.0) { targang = 180.0; }
            // ignoring timeout field completely
            cmd_state = 10; // POBS
            pobs_state = 1; // first turn
            all_leds_green();
            rvr.state = 1;
            sprintf(outdaimyodata, "<ACK,1>");
            outdaimyo_flag = true;
          }
        } // with fields
      }
    } // indaimyo_flag
    if(outdaimyo_flag) {
      outdaimyo_flag = false;
      tcpsendall(client, outdaimyodata);
    }
  }
}

void TCPdebughandle() {
  if(debug_client) {
    while(debug_client.available() > 0) {
      char inchar = debug_client.read();
      if(inchar == '>') {
        incoming_flag = true;
        indebugdata[incoming_indx] = '\0';
        incoming_indx = 0;
        break;
      } else if(inchar == '<') {
        incoming_indx = 0;
      } else {
        incoming_flag = false;
        indebugdata[incoming_indx++] = inchar;
        if (incoming_indx >= 256) { incoming_indx = 0; }
      }
    }
    if(incoming_flag) {
      incoming_flag = false;
      if(strcmp(indebugdata,"reset")==0) {
        if(client.connected()) {
          tcpsendall(client, "<BYE>");
          client.stop();
        }
        // RECONNECT
        if (!client.connect(daimyo_server, daimyo_port)) {
          all_leds_red();
          strcpy(outdebugdata, "connection failed");
          outdebug_flag = true;
          return;
        } else {
          all_leds_blue();
          IDsend = 1;
          freshjointime = millis();
        }
      } else if (strcmp(indebugdata, "help") == 0) {
        strcpy(outdebugdata, "Some useful commands:\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<reset>:\tAttempts to rejoin daimyo control server.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<calib>:\tLists MLX90393 compass calibration.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<calib_on>:\tStarts calibrating compass (rotates rover in place).\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<calib_off>:\tStops and records compass calibration to EEPROM.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<mlx_raw>:\tDisplays MLX90393 raw xyz values.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<dist>:\t\tDisplays SR-04 range finder measurement.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<col>:\t\tShows state of collision limit switches.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<red>:\t\tTurns LEDs red.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<blue>:\t\tTurns LEDs blue.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<poweroff>:\tDisconnects from daimyo and shuts down in 5 seconds.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<battery>:\tChecks battery life.\n");
        tcpsendall(debug_client, outdebugdata);
        delay(100);
        strcpy(outdebugdata, "<turn,angle>:\tTurns to face direction of (uint)angle (b/w 0 and 359).\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<fwd,speed>:\tDrives forward at (byte)speed for 2 seconds.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<bwd,speed>:\tDrives backward at (byte)speed for 2 seconds.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<daimyo_debug>:\tToggles daimyo debug flag so received messages are displayed here.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<server>:\tDisplays daimyo IP and port settings.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "<set_server,IP,port>:\tSets daimyo IP and port values.\n");
        tcpsendall(debug_client, outdebugdata);
        strcpy(outdebugdata, "quit:\t\tCloses this debug tcp connection.\n");
        tcpsendall(debug_client, outdebugdata);

      } else if (strcmp(indebugdata, "daimyo_debug") == 0) {
        daimyo_debug_flag = !daimyo_debug_flag;
        sprintf(outdebugdata, "daimyo_debug_flag now %d.",
                daimyo_debug_flag);
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "server") == 0) {
        sprintf(outdebugdata, "daimyo server IP: %s, Port: %d.",
                daimyo_server, daimyo_port);
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "OTA_ON") == 0) {
        OTA_flag = true;
        strcpy(outdebugdata, "Entered OTA mode");
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "OTA_OFF") == 0) {
        OTA_flag = false;
        strcpy(outdebugdata, "Left OTA mode");
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "calib_on") == 0) {
        xmax = -100.0; ymax = -100.0; xmin = 100.0; ymin = 100.0;
        calib_flag = true;
        strcpy(outdebugdata, "Entered calib mode");
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "calib_off") == 0) {
        mxoff = (xmax+xmin)*0.5;
        mxscale = 2.0/(xmax-xmin);
        myoff = (ymax+ymin)*0.5;
        myscale = 2.0/(ymax-ymin);
        myxscale = myscale/mxscale;
        eeprom_data.xmin = xmin; eeprom_data.xmax = xmax;
        eeprom_data.ymin = ymin; eeprom_data.ymax = ymax;
        uint addr = 0;
        EEPROM.begin(512);
        EEPROM.put(addr, eeprom_data);
        EEPROM.commit();
        calib_flag = false;
        sprintf(outdebugdata,
               "Left calib mode. x: (%.2f, %.2f), y: (%.2f, %.2f)",
               xmin, xmax, ymin, ymax);
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "north") == 0) {
        angle = getorient();
        sprintf(outdebugdata, "North: %.1f deg.", angle);
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "calib") == 0) {
        sprintf(outdebugdata,
                "#: %d, x: (%f, %f), y: (%f, %f).",
                compass_ID, eeprom_data.xmin, eeprom_data.xmax,
                eeprom_data.ymin, eeprom_data.ymax);
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "mlx_raw") == 0) {
        mlx.readData(mlxdata);
        sprintf(outdebugdata, "%f, %f, %f",
                mlxdata.x, mlxdata.y, mlxdata.z);
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "dist") == 0) {
        distobs = dist_m();
        sprintf(outdebugdata, "Dist: %.3f m.", distobs);
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "col") == 0) {
        sprintf(outdebugdata, "Collision: %d.", col_flag);
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "halt") == 0) {
        rvr.halt();
        sprintf(outdebugdata, "halted.");
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "red") == 0) {
        rvr.all_leds(0x40, 0, 0);
        sprintf(outdebugdata, "All leds now red.");
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "blue") == 0) {
        rvr.all_leds(0, 0, 0x40);
        sprintf(outdebugdata, "All leds now blue.");
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "poweroff") == 0) {
        all_leds_red();
        if(client.connected()) { // Disconnect from daimyo
          tcpsendall(client, "<BYE>");
          client.stop();
        }
        rvr.poweroff();
        sprintf(outdebugdata, "Powering off in 5 seconds.");
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "battery") == 0) {
        sprintf(outdebugdata, "Battery life is %d percent.",
                rvr.get_battery_life());
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "start_xy") == 0) {
        /* rvr.reset_xy(); */
        rvr.start_xy_stream(250); // 250 ms
        sprintf(outdebugdata, "XY locator stream service started.");
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "reset_xy") == 0) {
        rvr.reset_xy();
        sprintf(outdebugdata, "XY reset.");
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "stop_xy") == 0) {
        rvr.stop_xy_stream();
        sprintf(outdebugdata, "XY locator stream service stopped.");
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "get_xy") == 0) {
        sprintf(outdebugdata, "X = %f, Y = %f.", rvr.xpos, rvr.ypos);
        outdebug_flag = true;
      } else if (strcmp(indebugdata, "read_xy") == 0) {
        rvr.read_xy();
        sprintf(outdebugdata, "X = %f, Y = %f.", rvr.xpos, rvr.ypos);
        outdebug_flag = true;
      } else {
        char* separator = strchr(indebugdata, ',');
        if(separator == 0) { // No fields
        }
        else { // with fields
          *separator = '\0'; ++separator;
          if(strcmp(indebugdata, "all_leds") == 0) { //
            char* red = separator; separator = strchr(red, ',');
            *separator = '\0'; ++separator; byte ired = atoi(red);
            char* green = separator; separator = strchr(green, ',');
            *separator = '\0'; ++separator; byte igreen = atoi(green);
            char* blue = separator; separator = strchr(blue, ',');
            *separator = '\0'; ++separator; byte iblue = atoi(blue);
            rvr.all_leds(ired, igreen, iblue);
            sprintf(outdebugdata,
                    "All leds set to RGB = (%d, %d, %d).",
                    ired, igreen, iblue);
            outdebug_flag = true;
          } else if(strcmp(indebugdata, "set_server") == 0) { //
            char* inIP = separator; separator = strchr(inIP, ',');
            *separator = '\0'; ++separator;
            sprintf(daimyo_server, "%s", inIP);
            char* inport = separator; separator = strchr(inport, ',');
            if(separator > 0) { *separator = '\0'; ++separator; }
            daimyo_port = atoi(inport);
            sprintf(outdebugdata,
                    "New daimyo server = %s:%d.",
                    daimyo_server, daimyo_port);
            outdebug_flag = true;
          } else if(strcmp(indebugdata, "diffangpres") == 0) { //
            char* dap = separator; separator = strchr(dap, ',');
            if(separator > 0) { *separator = '\0'; ++separator; }
            diffangpres = atof(dap);
            sprintf(outdebugdata,
                    "Angle precision now %.1f.", diffangpres);
            outdebug_flag = true;
          } else if(strcmp(indebugdata, "lturn") == 0) { //
            char* speed = separator; separator = strchr(speed, ',');
            if(separator > 0) { *separator = '\0'; ++separator; }
            byte ispeed = atoi(speed) & 0xFF;
            rvr.set_raw_motors(2, ispeed, 1, ispeed);
            sprintf(outdebugdata,
                    "Turning left at speed %d.", ispeed);
            outdebug_flag = true;
          } else if(strcmp(indebugdata, "rturn") == 0) { //
            char* speed = separator; separator = strchr(speed, ',');
            if(separator > 0) { *separator = '\0'; ++separator; }
            byte ispeed = atoi(speed) & 0xFF;
            rvr.set_raw_motors(1, ispeed, 2, ispeed);
            sprintf(outdebugdata,
                    "Turning right at speed %d.", ispeed);
            outdebug_flag = true;
          } else if(strcmp(indebugdata, "turn") == 0) { //
            char* dangle = separator; separator = strchr(dangle, ',');
            if(separator > 0) { *separator = '\0'; ++separator; }
            uint16_t idangle = atoi(dangle);
            rvr.angle = 1.0*idangle;
            rvr.angle = fmod(rvr.angle, 360.0);
            if(rvr.angle < 0.0) { rvr.angle += 360; }
            /* rvr.reset_yaw(); */
            rvr.face(idangle);
            sprintf(outdebugdata,
                    "Turning by %d.", idangle);
            outdebug_flag = true;
          } else if(strcmp(indebugdata, "fwd") == 0) { //
            char* speed = separator; separator = strchr(speed, ',');
            if(separator > 0) { *separator = '\0'; ++separator; }
            byte ispeed = atoi(speed) & 0xFF;
            /* rvr.reset_yaw(); */
            rvr.drive(ispeed, (uint16_t)rvr.angle, 0);
            delay(2000);
            rvr.drive(0, (uint16_t)rvr.angle, 0);
            sprintf(outdebugdata, "Forward at speed %d.", ispeed);
            outdebug_flag = true;
          } else if(strcmp(indebugdata, "bwd") == 0) { //
            char* speed = separator; separator = strchr(speed, ',');
            if(separator > 0) { *separator = '\0'; ++separator; }
            byte ispeed = atoi(speed) & 0xFF;
            /* rvr.reset_yaw(); */
            rvr.drive(ispeed, (uint16_t)rvr.angle, 1);
            delay(2000);
            rvr.drive(0, (uint16_t)rvr.angle, 0);
            sprintf(outdebugdata, "Backward at speed %d.", ispeed);
            outdebug_flag = true;
          }
        }
      }
    } // incoming_flag
    if(outdebug_flag) {
      outdebug_flag = false;
      tcpsendall(debug_client, outdebugdata);
    }
  } else {
    debug_client = debug_server.available();
  }
}

void tcpsendall(WiFiClient myclient, char* buf) {
  uint8_t bufsize = strlen(buf);
  uint8_t sentnum = myclient.write(buf, bufsize);
  while(sentnum < bufsize) {
    sentnum += myclient.write(buf+sentnum, bufsize-sentnum);
  }
  buf[0] = '\0';
}

void mag_calib() {
  delay(1);
  mlx.readData(mlxdata); //Read the values from the sensor
  if (mlxdata.x < xmin) { xmin = mlxdata.x; }
  if (mlxdata.y < ymin) { ymin = mlxdata.y; }
  if (mlxdata.z < zmin) { zmin = mlxdata.z; }
  if (mlxdata.x > xmax) { xmax = mlxdata.x; }
  if (mlxdata.y > ymax) { ymax = mlxdata.y; }
  if (mlxdata.z > zmax) { zmax = mlxdata.z; }
}

void cmd_exec() {
  switch(cmd_state) {
  case 1: { // FWD
    rvr.read_xy();
    float netdist = (rvr.xpos - rvr.xoff)*rvr.sintheta +
      (rvr.ypos - rvr.yoff)*rvr.costheta;
    if(netdist >= targdist) {
      rvr.drive(0, 0, 0); // rvr.halt(); // roll stop for smooth turning
      rvr.state = 0;
      all_leds_blue();
      if(client.connected()) {
        tcpsendall(client, "<ACK,0>");
        rvr.read_xy();
        rvr.angle = rvr.Nangle + getorient();
        if(rvr.angle < 0.0) { rvr.angle += 360; }
        sprintf(outdaimyodata, "<MYPOS,%.3f,%.3f,%.1f>",
                rvr.xpos, rvr.ypos, rvr.angle);
        tcpsendall(client, outdaimyodata);
      } else { all_leds_red(); }
      cmd_state = 0;
    } else { rvr.drive(targvel, 0, 0); }
  } break;
  case 2: { // BWD
    rvr.read_xy();
    float netdist = (rvr.xpos - rvr.xoff)*rvr.sintheta +
      (rvr.ypos - rvr.yoff)*rvr.costheta;
    if(netdist <= targdist) { // it is negative
      rvr.drive(0, 0, 1); // rvr.halt();
      rvr.state = 0;
      all_leds_blue();
      if(client.connected()) {
        tcpsendall(client, "<ACK,0>");
        rvr.read_xy();
        rvr.angle = rvr.Nangle + getorient();
        if(rvr.angle < 0.0) { rvr.angle += 360; }
        sprintf(outdaimyodata, "<MYPOS,%.3f,%.3f,%.1f>",
                rvr.xpos, rvr.ypos, rvr.angle);
        tcpsendall(client, outdaimyodata);
      } else { all_leds_red(); }
      cmd_state = 0;
    } else { rvr.drive(targvel, 0, 1); }
  } break;
  case 3: { // TURN
    rvr.angle = fmod(rvr.Nangle + getorient(), 360.0);
    if(rvr.angle < 0.0) { rvr.angle += 360; }
    float diffang = targang - rvr.angle;
    if(abs(diffang) < diffangpres) {
      rvr.halt(); cmd_state = 0; rvr.state = 0;
      all_leds_blue();
      if(client.connected()) {
        tcpsendall(client, "<ACK,0>");
        rvr.read_xy();
        rvr.angle = rvr.Nangle + getorient();
        if(rvr.angle < 0.0) { rvr.angle += 360; }
        sprintf(outdaimyodata, "<MYPOS,%.3f,%.3f,%.1f>",
                rvr.xpos, rvr.ypos, rvr.angle);
        tcpsendall(client, outdaimyodata);
      } else { all_leds_red(); }
    } else {
      reset_rvr_offsets();
      int turnby = (int)fmod((-1.0)*diffang, 360.0);
      if(turnby > 180) { turnby -= 360; }
      if(turnby < -180) { turnby += 360; }
      if(turnby > 0) { rvr.face(turnby % 360); }
      else { rvr.face(360+(turnby % 360));  }
      delay(300 + abs(turnby));
    }
  } break;
  case 4: { // ATURN
    rvr.angle = fmod(rvr.Nangle + getorient(), 360.0);
    if(rvr.angle < 0.0) { rvr.angle += 360; }
    float diffang = targang - rvr.angle;
    if(abs(diffang) < diffangpres) {
      rvr.halt(); cmd_state = 0; rvr.state = 0;
      all_leds_blue();
      if(client.connected()) {
        tcpsendall(client, "<ACK,0>");
        rvr.read_xy();
        rvr.angle = rvr.Nangle + getorient();
        if(rvr.angle < 0.0) { rvr.angle += 360; }
        sprintf(outdaimyodata, "<MYPOS,%.3f,%.3f,%.1f>",
                rvr.xpos, rvr.ypos, rvr.angle);
        tcpsendall(client, outdaimyodata);
      } else { all_leds_red(); }
    } else {
      reset_rvr_offsets();
      int turnby = (int)fmod((-1.0)*diffang, 360.0);
      if(turnby > 180) { turnby -= 360; }
      if(turnby < -180) { turnby += 360; }
      if(turnby > 0) { rvr.face(turnby % 360); }
      else { rvr.face(360+(turnby % 360));  }
      delay(300 + abs(turnby));
    }
  }  break;
  case 5: { // GOTO
    switch(goto_state) {
    case 1: { // First turn
      rvr.angle = fmod(rvr.Nangle + getorient(), 360.0);
      if(rvr.angle < 0.0) { rvr.angle += 360; }
      float diffang = targang - rvr.angle;
      if(abs(diffang) < diffangpres) {
        rvr.halt(); goto_state = 2; // traverse
        reset_rvr_offsets();
        targdist = sqrt((targx-rvr.xpos)*(targx-rvr.xpos) +
                        (targy-rvr.ypos)*(targy-rvr.ypos)) - 12.8/targvel;
        if(targdist < 0.0) { targdist = 0.0; }
        rvr.drive(targvel, 0, 0);
        if(client.connected()) {
          rvr.read_xy();
          rvr.angle = rvr.Nangle + getorient();
          if(rvr.angle < 0.0) { rvr.angle += 360; }
          sprintf(outdaimyodata, "<MYPOS,%.3f,%.3f,%.1f>",
                  rvr.xpos, rvr.ypos, rvr.angle);
          tcpsendall(client, outdaimyodata);
        }
        olddistsq = (targx-rvr.xpos)*(targx-rvr.xpos) +
          (targy-rvr.ypos)*(targy-rvr.ypos);
      } else {
        reset_rvr_offsets();
        int turnby = (int)fmod((-1.0)*diffang, 360.0);
        if(turnby > 180) { turnby -= 360; }
        if(turnby < -180) { turnby += 360; }
        if(turnby > 0) { rvr.face(turnby % 360); }
        else { rvr.face(360+(turnby % 360));  }
        delay(300 + abs(turnby));
      }
    } break;
    case 2: { // traverse
      rvr.read_xy();
      float netdist = (rvr.xpos - rvr.xoff)*rvr.sintheta +
        (rvr.ypos - rvr.yoff)*rvr.costheta;
      if(netdist >= targdist) { // reached
        rvr.drive(0, 0, 0); // rvr.halt();
        rvr.read_xy();
        if(targang == targang2) {
          goto_state = 0;
          rvr.state = 0; all_leds_blue();
          cmd_state = 0;
          if(client.connected()) {
            rvr.angle = rvr.Nangle + getorient();
            if(rvr.angle < 0.0) { rvr.angle += 360; }
            sprintf(outdaimyodata, "<ACK,0><MYPOS,%.3f,%.3f,%.1f>",
                    rvr.xpos, rvr.ypos, rvr.angle);
            tcpsendall(client, outdaimyodata);
          } else { all_leds_red(); }
        } else { goto_state = 3; } // second turn
      } else { rvr.drive(targvel, 0, 0); }
    } break;
    case 3: { // Second turn
      rvr.angle = fmod(rvr.Nangle + getorient(), 360.0);
      if(rvr.angle < 0.0) { rvr.angle += 360; }
      float diffang = targang2 - rvr.angle;
      if(abs(diffang) < diffangpres) {
        rvr.halt(); rvr.read_xy(); goto_state = 0;
        rvr.state = 0; all_leds_blue();
        cmd_state = 0;
        if(client.connected()) {
          rvr.angle = rvr.Nangle + getorient();
          if(rvr.angle < 0.0) { rvr.angle += 360; }
          sprintf(outdaimyodata, "<ACK,0><MYPOS,%.3f,%.3f,%.1f>",
                  rvr.xpos, rvr.ypos, rvr.angle);
          tcpsendall(client, outdaimyodata);
        } else { all_leds_red(); }
      } else {
        reset_rvr_offsets();
        int turnby = (int)fmod((-1.0)*diffang, 360.0);
        if(turnby > 180) { turnby -= 360; }
        if(turnby < -180) { turnby += 360; }
        if(turnby > 0) { rvr.face(turnby % 360); }
        else { rvr.face(360+(turnby % 360));  }
        delay(300 + abs(turnby));
      }
    } break;
    }
  } break;
  case 6: { // CFWD
    rvr.drive(targvel, 0, 0);
  } break;
  case 7: { // CBWD
    rvr.drive(targvel, 0, 1);
  } break;
  case 8: { // CTURN
    if(targdir == 0) { rvr.set_raw_motors(2, defangvel, 1, defangvel); }
    else { rvr.set_raw_motors(1, defangvel, 2, defangvel); }
    rvr.angle = fmod(rvr.Nangle + getorient(), 360.0);
    if(rvr.angle < 0.0) { rvr.angle += 360; }
  } break;
  case 9: { // OBS
    rvr.angle = fmod(rvr.Nangle + getorient(), 360.0);
    if(rvr.angle < 0.0) { rvr.angle += 360; }
    float diffang = targang - rvr.angle;
    if(abs(diffang) < diffangpres) {
      rvr.halt(); cmd_state = 0; rvr.state = 0;
      all_leds_blue();
      if(client.connected()) {
        rvr.angle = rvr.Nangle + getorient();
        if(rvr.angle < 0.0) { rvr.angle += 360; }
        sprintf(outdaimyodata, "<ACK,0><MYPOS,%.3f,%.3f,%.1f>",
                rvr.xpos, rvr.ypos, rvr.angle);
        tcpsendall(client, outdaimyodata);
        float dist_obs = dist_m();
        sprintf(outdaimyodata, "<DOBS,%.3f>", dist_obs);
        tcpsendall(client, outdaimyodata);
      } else { all_leds_red(); }
    } else {
      reset_rvr_offsets();
      int turnby = (int)fmod((-1.0)*diffang, 360.0);
      if(turnby > 180) { turnby -= 360; }
      if(turnby < -180) { turnby += 360; }
      if(turnby > 0) { rvr.face(turnby % 360); }
      else { rvr.face(360+(turnby % 360));  }
      delay(300 + abs(turnby));
    }
  } break;
  case 10: { // POBS
    switch(pobs_state) {
    case 1: { // first turn
      rvr.angle = fmod(rvr.Nangle + getorient(), 360.0);
      if(rvr.angle < 0.0) { rvr.angle += 360; }
      float diffang = targang - rvr.angle;
      if(abs(diffang) < diffangpres) {
        rvr.halt(); pobs_state = 2; // traverse
        reset_rvr_offsets();
        if(client.connected()) {
          rvr.read_xy();
          rvr.angle = rvr.Nangle + getorient();
          if(rvr.angle < 0.0) { rvr.angle += 360; }
          sprintf(outdaimyodata, "<MYPOS,%.3f,%.3f,%.1f>",
                  rvr.xpos, rvr.ypos, rvr.angle);
          tcpsendall(client, outdaimyodata);
        }
      } else {
        reset_rvr_offsets();
        int turnby = (int)fmod((-1.0)*diffang, 360.0);
        if(turnby > 180) { turnby -= 360; }
        if(turnby < -180) { turnby += 360; }
        if(turnby > 0) { rvr.face(turnby % 360); }
        else { rvr.face(360+(turnby % 360));  }
        delay(300 + abs(turnby));
      }
    } break;
    case 2: { // traverse
        float curdist = dist_m();
        if(curdist > targdist) {
          targdist = curdist - targdist - 0.1;
          rvr.drive(32, 0, 0);
          cmd_state = 1; // FWD
        }
        else {
          targdist = targdist - curdist - 0.1;
          rvr.drive(32, 0, 1);
          cmd_state = 2; // BWD
        }
        if(targdist < 0.0) { targdist = 0.0; }
        targvel = 32;
    } break;
    }
  } break;
  }
}

void reset_rvr_offsets() {
  rvr.read_xy();
  rvr.xoff = rvr.xpos; rvr.yoff = rvr.ypos;
  rvr.reset_yaw(); rvr.reset_xy();
  float theta = fmod(90.0 - rvr.Nangle - getorient(), 360.0);
  theta = theta*radconv;
  rvr.sintheta = sin(theta); rvr.costheta = cos(theta);
}

void all_leds_red() {
  rvr.all_leds(0x40, 0, 0);
}

void all_leds_green() {
  rvr.all_leds(0, 0x40, 0);
}

void all_leds_blue() {
  rvr.all_leds(0, 0, 0x40);
}
