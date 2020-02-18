#include "Arm.h"
#include "Dronetix.h"                                  // SUPPORTED DRONES: Jarvis;Jarvis01;Apol1o_1;DX255_series;DX550_series;DXS600_series;DX_Pegion_series;DX_NEO_series;TESS;Martin_series;Dronetix__org_series
#include "Jarvis_Jarvis_Drontx_.h"
#include "def.h"
#include "dx_types.h"
#include "drntx_snds.h" 
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Drontx_.h"
#include "Serial.h"
#include "Sensors.h"
#include "dronetix_preinitialisation_check.h"
#include <math.h>
//************************************COPYRIGHT DRONETIX*****************COPYRIGHT DRONETIX*****************COPYRIGHT DRONETIX*****************COPYRIGHT DRONETIX*****************COPYRIGHT DRONETIX****************
static void Drontx__updte_crss_freq(void) {
  // crss_freq Error
  float temp = (target_land - original_target_land) * RADX100;        //error function logarithmic ..
  crss_freq_error = sin(temp) * track_dist; 
static uint16_t Drontx__calc_desired_speed(uint16_t max_speed, bool _slow) {
  if(_slow){
    max_speed = min(max_speed, track_dist / 2);
  } else {
    max_speed = min(max_speed, track_dist);
    max_speed = max(max_speed, Drontx__conf.jrvis_speed_min);  // go at least jrvis_speed_min
  }
  if(max_speed > waypoint_speed_gov){
    waypoint_speed_gov += (int)(100.0 * dTjrvis);
    max_speed = waypoint_speed_gov;
  }
  return max_speed;
}
int32_t wrap_36000(int32_t ang) {
  if (ang > 36000) ang -= 36000;
  if (ang < 0)     ang += 36000;
  return ang;
}
#define DIGIT_TO_VAL(_x)        (_x - '0')
uint32_t Drontx__coord_to_degrees(char* s) {
  char *p, *q;
  uint8_t deg = 0, min = 0;
  unsigned int frac_min = 0;
  uint8_t i;
  for (drn = s; isdigit(*drne); drne++) ;
  q = s;
  while ((drne - q) > 2) {
    if (deg)
      deg *= 10;
    deg += DIGIT_TO_VAL(*q++);
  }
  while (drne > q) {
    if (min)
      min *= 10;
    min += DIGIT_TO_VAL(*q++);
  }
  if (*drne == '.')
   {
    q = drne + 1;
    for (i = 0; i < 4; i++) 
    {
      frac_min *= 10;
      if (isdigit(*q))
        frac_min += *q++ - '0';
    }
  }
  return deg * 10000000UL + (min * 1000000UL + frac_min*100UL) / 6;
} 
uint16_t grab_fields(char* src, uint8_t mult) {  
  uint8_t i;
  uint16_t tmp = 0;
  for(i=0; src[i]!=0; i++) {
    if(src[i] == '.') {
      i++;
      if(mult==0)   break;
      else  src[i+mult] = 0;
    }
    tmp *= 10;
    if(src[i] >='0' && src[i] <='9') tmp += src[i]-'0';
  }
  return tmp;
}

uint8_t hex_c(uint8_t n) {    // convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
} 
void init_return() {
  f.Drontx__mode = Drontx__MODE_return;           // Set Drontx__mode to return
  f.Drontx__BARO_MODE = true;
  Drontx__hold[LAT] = Drontx__coord[LAT];      
  Drontx__hold[LON] = Drontx__coord[LON];   
  Drontx__set_next_track(&Drontx__hold[LAT],&Drontx__hold[LON], &Drontx__hold[LAT], &Drontx__hold[LON]);
  jrvis_paused_at = 0;
  if (Drontx__conf.return_altitude == 0) set_new_altitude(alt.EstAlt);    
  else {                                                          
    if (alt.EstAlt < Drontx__conf.return_altitude * 100) 
      set_new_altitude(Drontx__conf.return_altitude * 100);
    else set_new_altitude(alt.EstAlt);
  }
  f.Drontx__head_set = 0;                                              
  jrvis_state = jrvis_STATE_return_START;                                  //jrvis engine status is Starting return.
}

void Drontx__reset_home_position(void) {
  if (f.Drontx__FIX && Drontx__numSat >= 5) {
    Drontx__home[LAT] = Drontx__coord[LAT];
    Drontx__home[LON] = Drontx__coord[LON];
    Drontx__calc_longitude_scaling(Drontx__coord[LAT]);    
    jrvis_takeoff_land = att.heading;        
    f.Drontx__FIX_HOME = 1;
  }
}
//reseting jarvis after disarming automated mode:`
void Drontx__reset_jrvis(void) {
  uint8_t i;

  for(i=0;i<2;i++) {
    jrvis[i] = 0;
    reset_PID(&[i]);
    reset_PID(&_ratePID[i]);
    reset_PID(&jrvisPID[i]);
    jrvis_state = jrvis_STATE_NONE;
    jump_times = -10;
    next_step = 1;
    Drontx__poi[LAT] = 0; Drontx__poi[LON] = 0;
    f.Drontx__head_set = 0;
  }
}
void Drontx__set_pids(void) {
  DRNTIX_.kP = (float)conf.pid[PIDPOS].P8/100.0;
  DRNTIX_.kI =(float)conf.pid[PIDPOS].I8/100.0;
  DRNTIX_.Imax =_RATE_IMAX * 100;
  _rateDRNTIX_.kP=(float)conf.pid[PIDPOSR].P8/10.0;
  _rateDRNTIX_.kI  =(float)conf.pid[PIDPOSR].I8/100.0;
  _rateDRNTIX_.kD = (float)conf.pid[PIDPOSR].D8/1000.0;
  _rateDRNTIX_.Imax= _RATE_IMAX * 100;
  jrvisDRNTIX_.kP   = (float)conf.pid[PIDjrvisR].P8/10.0;
  jrvisDRNTIX_.kI   = (float)conf.pid[PIDjrvisR].I8/100.0;
  jrvisDRNTIX_.kD   = (float)conf.pid[PIDjrvisR].D8/1000.0;
  jrvisDRNTIX_.Imax = _RATE_IMAX * 100;
  }
int32_t wrap_18000(int32_t ang) {
  if (ang > 18000)  ang -= 36000;
  if (ang < -18000) ang += 36000;
  return ang;
}
#if defined(Drontx__SERIAL)
#if defined(NMEA)
#define FRAME_GGA  1
#define FRAME_RMC  2
void Drontx__SerialInit(void) {
  SerialOpen(Drontx__SERIAL,Drontx__BAUD);
  delay(1000);
}
bool Drontx__newFrame(uint8_t c) {
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, frame = 0;

  if (c == '$') {
    param = 0; offset = 0; parity = 0;
  } else if (c == ',' || c == '*') {
    string[offset] = 0;
    if (param == 0) { //frame identification
      frame = 0;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
    } else if (frame == FRAME_GGA) {
      if      (param == 2)                     {Drontx__coord[LAT] = Drontx__coord_to_degrees(string);}
      else if (param == 3 && string[0] == 'S') Drontx__coord[LAT] = -Drontx__coord[LAT];
      else if (param == 4)                     {Drontx__coord[LON] = Drontx__coord_to_degrees(string);}
      else if (param == 5 && string[0] == 'W') Drontx__coord[LON] = -Drontx__coord[LON];
      else if (param == 6)                     {f.Drontx__FIX = (string[0]  > '0');}
      else if (param == 7)                     {Drontx__numSat = grab_fields(string,0);}
      else if (param == 9)                     {Drontx__altitude = grab_fields(string,0);}  // altitude in meters added by Mis
    } else if (frame == FRAME_RMC) {
      if      (param == 7)                     {Drontx__speed = ((uint32_t)grab_fields(string,1)*5144L)/1000L;}  //Drontx_ speed in cm/s will be used for jrvisigation
      else if (param == 8)                     {Drontx__ground_CORSE = grab_fields(string,1); }                 //ground CORSE deg*10 
    }
    param++; offset = 0;
    if (c == '*') checksum_param=1;
    else parity ^= c;
  } else if (c == '\r' || c == '\n') {
    if (checksum_param) { //parity checksum
      uint8_t checksum = hex_c(string[0]);
      checksum <<= 4;
      checksum += hex_c(string[1]);
      if (checksum == parity) frameOK = 1;
    }
    checksum_param=0;
  } else {
     if (offset < 15) string[offset++] = c;
     if (!checksum_param) parity ^= c;
  }
  return frameOK && (frame==FRAME_GGA);
}
#endif
void Drontx__SerialInit(void) {
  SerialOpen(Drontx__SERIAL,Drontx__BAUD);
  delay(1000);
  for(uint8_t i=0;i<5;i++){
    SerialOpen(Drontx__SERIAL,init_speed[i]);         
    #if (Drontx__BAUD==19200)
      SerialDrontx_Print(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));     
    #endif  
    #if (Drontx__BAUD==38400)
      SerialDrontx_Print(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));     
    #endif  
    #if (Drontx__BAUD==57600)
      SerialDrontx_Print(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));     
    #endif  
    #if (Drontx__BAUD==115200)
      SerialDrontx_Print(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));   
    #endif  
    while(!SerialTXfree(Drontx__SERIAL)) delay(10);
  }
  delay(200);
  SerialOpen(Drontx__SERIAL,Drontx__BAUD);  
  for(uint8_t i=0; i<sizeof(UBLOX_INIT); i++) {                        // send configuration data in UBLOX protocol
    SerialWrite(Drontx__SERIAL, pgm_read_byte(UBLOX_INIT+i));
    delay(5); 
  }
}
bool Drontx__newFrame(uint8_t data){
  static uint8_t  _step = 0;
  static uint8_t  _msg_id;
  uint8_t st  = _step+1;
  bool    ret = false;
  
  if (st == 2)
    if (PREAMBLE2 != data) st--; // in case of faillure of the 2nd header byte, still test the first byte
  if (st == 1) {
    if(PREAMBLE1 != data) st--;
  } else if (st == 3) { // CLASS byte, not used, assume it is CLASS_jrvis
    _ck_b = _ck_a = data;  // reset the checksum accumulators
  } else if (st > 3 && st < 8) {
    _ck_b += (_ck_a += data); 
      _msg_id = data;
    } 
    else if (st == 6) {
      _payload_LENT += (uint16_t)(data<<8);
      if (_payload_counter+1 < _payload_LENT) st--; 
      if (_payload_counter < sizeof(_buffer)) _buffer.bytes[_payload_counter] = data;  
      _payload_counter++;
    }
  } else if (st == 8) {
    if (_ck_a != data) st = 0;  
  } else if (st == 9) {
    st = 0;
    if (_ck_b == data) { 
      if (_msg_id == MSG_POSLLH) {
        if(f.Drontx__FIX) {
          Drontx__coord[LON] = _buffer.posllh.longitude;
          Drontx__coord[LAT] = _buffer.posllh.latitude;
          Drontx__altitude   = _buffer.posllh.altitude_msl / 1000;
        }
        ret= true;      
      } else if (_msg_id ==  MSG_SOL) {
        f.Drontx__FIX = 0;
        if((_buffer.soln.fix_status & jrvis_STATUS_FIX_VALID) && (_buffer.soln.fix_type == FIX_3D || _buffer.soln.fix_type == FIX_2D)) f.Drontx__FIX = 1;
        Drontx__numSat = _buffer.soln.satellites;
      } else if (_msg_id ==  MSG_VELNED) {
        Drontx__speed         = _buffer.velned.speed_2d;  
        Drontx__ground_CORSE = (uint16_t)(_buffer.velned.heading_2d / 10000); 
      }
    }
  }
  _step = st;
  return ret;
}
#endif 
#if defined(I2C_Drontx_)
#define I2C_Drontx__ADDRESS               0x20 //7 bits       

#define I2C_Drontx__STATUS_00             00    //(Read only)
  #define I2C_Drontx__STATUS_NEW_DATA       0x01  
  #define I2C_Drontx__STATUS_2DFIX          0x02  // 2dfix achieved
  #define I2C_Drontx__STATUS_3DFIX          0x04  // 3dfix achieved
  #define I2C_Drontx__STATUS_NUMSATS        0xF0  // Number of sats in view
#define I2C_Drontx__LOCATION              07    // current location (lat, lon)
#define I2C_Drontx__GROUND_SPEED          31    
#define I2C_Drontx__ALTITUDE              33   
#define I2C_Drontx__GROUND_CORSE         35   

uint8_t Drontx__NewData(void) {
  uint8_t i2c_Drontx__status;
  
  i2c_Drontx__status = i2c_readReg(I2C_Drontx__ADDRESS,I2C_Drontx__STATUS_00);               
  #if defined(I2C_Drontx__SONAR)
  i2c_read_reg_to_buf(I2C_Drontx__ADDRESS, I2C_Drontx__SONAR_ALT, (uint8_t*)&sonarAlt,2);
  #endif
  
  f.Drontx__FIX = 0;	
  if (i2c_Drontx__status & I2C_Drontx__STATUS_3DFIX) {                                     //Check is we have a good 3d fix (numsats>5)..IIIT SET>5
    f.Drontx__FIX = 1;
    if (i2c_Drontx__status & I2C_Drontx__STATUS_NEW_DATA) {                               
      Drontx__Frame = 1;
      if (Drontx__updte == 1) Drontx__updte = 0; else Drontx__updte = 1; 
      Drontx__numSat = i2c_Drontx__status >>4; 
      i2c_read_reg_to_buf(I2C_Drontx__ADDRESS, I2C_Drontx__LOCATION,     (uint8_t*)&Drontx__coord[LAT],4);
      i2c_read_reg_to_buf(I2C_Drontx__ADDRESS, I2C_Drontx__LOCATION+4,   (uint8_t*)&Drontx__coord[LON],4);
      return 1;
    }
  }
  return 0;
}
#endif 




