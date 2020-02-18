#include <avr/io.h>
#include "Arm.h"
#include "Dronetix.h"                                  // SUPPORTED DRONES: Jarvis;Jarvis01;Apol1o_1;DX255_series;DX550_series;DXS600_series;DX_Pegion_series;DX_NEO_series;TESS;Martin_series;Dronetix__org_series
#include "Jarvis_Jarvis_dronetix_gps.h"
#include "jr_JARVIS_CONFIG.h"
#include "def.h"
#include "dx_types.h"
#include "drntx_snds.h"      
#include "Arduino.h"
#include "config.h"
#include "Output.h"
#include "RX.h"
#include "Sensors.h"
#include "Serial.h"
#include "Protocol.h"
#include <avr/pgmspace.h>
//************************************COPYRIGHT DRONETIX*****************COPYRIGHT DRONETIX*****************COPYRIGHT DRONETIX*****************COPYRIGHT DRONETIX*****************COPYRIGHT DRONETIX****************
const char drntix_pid[] PROGMEM =
  "AD;"
  "WS;"
  "YAW;"
  "ALT;"
  "Pos;"
  "LOC;"
  "krish_;"
  "LEVEL;"
  "MAG_DRN;"
;
const char DRNTIX_ARRY[] PROGMEM = // CODES OF DYNAMICALLY GEENRATED GUI VALUES
  "ARM;"
  #if ACC
    "USER_FRIENDLYY;"
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    "BARO;"
  #endif
  #ifdef VARIOMETER
    "VARIO;"
  #endif
  "MAG;"
#endif
#if defined(SERVO_TILT)|| defined(GIMBAL)||defined(SERVO_MIX_TILT) //FOR PHOTOGRAPHY DRONES OF DRONETIX
  "camstable;"
#endif
#if dronetix_gps
  "dronetix_gps HOME;"
  "dronetix_gps HOLD;"
#endif
#if defined(LED_FLASHER)  //passed over to 328p
  "LEDMAX;"
  "LEDLOW;"
#endif
#if defined(LANDING_LIGHTS_DDR)
  "LLIGHTS;"
#endif
#ifdef injarvis__acc_calib
  "CALIB;"
#endif
#ifdef governor
  "GOVERNOR;"
#endif
#ifdef DRNTIX_CUSTOM_OSD
  "OSD ;"
#endif
#if dronetix_gps
  "TRACK;"
  "LAND;"
#endif;
const uint8_t boxids[] PROGMEM = {
  0, 
  #if ACC
    1, //"USER_FRIENDLYY;"
    2, //"DRNTIX_POS_HLD;"
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    3, //"BARO;"
  #endif
  #ifdef VARIOMETER
    4, //"VARIO;"
  #endif
  5, //"MAG;"
#endif
#if defined(SERVO_TILT)||defined(GIMBAL)||defined(SERVO_MIX_TILT)
  6, //"camstable;"
#endif
#if dronetix_gps
  10, //"dronetix_gps HOME;"
  11, //"dronetix_gps HOLD;"
#endif
#ifdef injarvis__acc_calib
  17, //"CALIB;"
#endif
#ifdef governor
  18, //"GOVERNOR;"
#endif
#ifdef DRNTIX_USTOM_OSD
  19, //"DRNTIX_USTOM_OSD;"
#endif
#if dronetix_gps
  20, //"TRACK;"
  21, //"LAND;"
#endif
  };
uint32_t CRNTtime = 0;
uint16_t PREtime = 0;
uint16_t JARVIS_RUN_LOOP = 0;     
int16_t  magHold,headFreeModeHold; 
uint8_t  vbatMin = VBATNOMINAL; 
uint8_t  DRONETOX_RC_OPT[CHECKBOXITEMS];
int32_t  AltHold; 
int16_t  BaroPID = 0;
int16_t  erroralt_jrI = 0;
int16_t gyroZero[3] = {0,0,0};
imu_t imu;
analog_t analog;
alt_t alt;
att_t att;
#if defined(ARMEDtimeWARNING)
  uint32_t  ArmedtimeWarningMicroSeconds = 0;
#endif
int16_t  debug[4];
flags_struct_t f;
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
  uint16_t JARVIS_RUN_LOOPMax = 0;      
  uint16_t JARVIS_RUN_LOOPMin = 65535;   
  int32_t  BAROaltMax;            
  uint16_t dronetix_gps_speedMax = 0;       
  #ifdef POWERMETER_HARD
    uint16_t powerValueMax = 0;
  #endif
  #if defined(WATTS)
    uint16_t jrvis_max_crrnt = 0;
  #endif
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDtimeWARNING) || defined(LOG_PERMANENT)
  uint32_t armedtime = 0;
#endif

int16_t  i2c_errors_count = 0;

#if defined(JARVIS_THRTTLE_CRRECT)
  int16_t jarvis_thrttle_crrect = 0; 
  int8_t  cosZ = 100;              
#endif
#if defined(injarvis__acc_calib)
  uint16_t Injarvis_calibratingA = 0;
  int16_t AccInjarvis_CalibrationArmed;
  uint16_t AccInjarvis_CalibrationSavetodronetix_preinitialisation_check = 0;
  uint16_t AccInjarvis_CalibrationActive = 0;
#endif
#if defined(POWERMETER) || ( defined(LOG_VALUES) && (LOG_VALUES >= 3) )
  uint32_t pMeter[PMOTOR_SUM + 1]; 
  uint8_t pMeterV;                                 
  uint16_t powerValue = 0;          
#endif
uint16_t intPowerTrigger1;
#if defined(LCD_TELEMETRY)
  uint8_t telemetry = 0;
  uint8_t telemetry_JRVIS = 0;
#endif
#ifdef LCD_TELEMETRY_STEP
  char telemetryStepSequence []  = LCD_TELEMETRY_STEP;
  uint8_t telemetryStepIndex = 0;
#endif
#define AD_LO  (1<<(2*AD))
#define AD_CE  (3<<(2*AD))
#define AD_HI  (2<<(2*AD))
#define WS_LO  (1<<(2*WS))
#define WS_CE  (3<<(2*WS))
#define WS_HI  (2<<(2*WS))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))
int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;
int16_t RC_DRONETIX_DATAS[RC_CHANS];    
int16_t rcSerial[8];        
uint8_t rcSerialCount = 0;   
int16_t lookupWSADRC[5];
uint16_t lookupThrottleRC[11];
#if defined(SERIAL_RX)
  volatile uint8_t  JRVIS_FrameFlags;
  volatile uint32_t JRVIS_timeLast;
  uint8_t  JRVIS_FrameDone;
#endif
int16_t axsPID[3];
int16_t motor[8];
int16_t servo[8] = {1500,1500,1500,1500,1500,1500,1500,1000};  //SUCCESSOVE PROVIDABLE BANDWIDTH
static uint8_t dynP8[2], dynD8[2];
global_conf_t global_conf;
conf_t conf;
#ifdef LOG_PERMANENT
  jarvis1_t jarvis1;
#endif
#if dronetix_gps
  dronetix_gps_conf_struct dronetix_gps_conf;
#endif
  int16_t  dronetix_gps_USER_FRIENDLY_MODE_DRNTIX[2] = { 0, 0};                      // the USER_FRIENDLY_MODE_DRNTIXs that must be applied for dronetix_gps correction
  int32_t  dronetix_gps_coord[2];
  int32_t  dronetix_gps_home[2];
  int32_t  dronetix_gps_hold[2];

  int32_t  dronetix_gps_prev[2];                              
  int32_t  dronetix_gps_poi[2];
  uint8_t  dronetix_gps_numSat;
  uint16_t dronetix_gps_distanceToHome;                        
  int16_t  dronetix_gps_directionToHome;                       
  int32_t  dronetix_gps_directionToPoi;
  uint16_t dronetix_gps_alt_jr;                         
  uint16_t dronetix_gps_speed;                                  
  uint8_t  dronetix_gps_update = 0;                             
  uint16_t dronetix_gps_ground_course = 0;               
  uint8_t next_step = 1;                 
  int16_t jump_times = -10;
#if dronetix_gps
  TRACK_step_struct TRACK_step;
#endif
  int16_t  krish_[2];
  int16_t  krish__rated[2];   
  int32_t original_alt_jr;
  int32_t target_alt_jr;
  int32_t alt_to_hold;

  uint32_t alt_change_timer;
  int8_t alt_change_flag;
  uint32_t alt_change;   
#if BARO
  int32_t baroPressure;
  int16_t barotemp;
  int32_t baroPressureSum;
#endif
  static uint8_t anlg_redr =0;
  switch (anlg_redr++ % (3+VBAT_CELLS_NUM)) {
  case 0:
  {
    #if defined(POWERMETER_HARD)
      static uint32_t lastRead = CRNTtime;
      static uint8_t ind = 0;
      static uint16_t pvec[PSENSOR_SMOOTH], psum;
      uint16_t p =  analogRead(PSENSORPIN);
      powerValue = ( conf.psensornull > p ? conf.psensornull - p : p - conf.psensornull); 
      analog.jrvis_hunger = ((uint32_t)powerValue * conf.pint2ma) / 100; // [100mA]   
      pMeter[PMOTOR_SUM] += ((CRNTtime-lastRead) * (uint32_t)((uint32_t)powerValue*conf.pint2ma))/100000;
      lastRead = CRNTtime;
    #endif 
    break;
  }
  case 1:
  {
    #if defined(VBAT) && !defined(VBAT_CELLS)
      static uint8_t ind = 0;
      static uint16_t vvec[VBAT_SMOOTH], vsum;
      uint16_t v = analogRead(V_BATPIN);
      #if VBAT_SMOOTH == 1
        analog.vbat = (v*VBAT_PRESCALER) / conf.vbatscale + VBAT_OFFSET; 
        vsum += v;
        vsum -= vvec[ind];
        vvec[ind++] = v;
        ind %= VBAT_SMOOTH;
        #if VBAT_SMOOTH == VBAT_PRESCALER
          analog.vbat = vsum / conf.vbatscale + VBAT_OFFSET; 
        #elif VBAT_SMOOTH < VBAT_PRESCALER
          analog.vbat = (vsum * (VBAT_PRESCALER/VBAT_SMOOTH)) / conf.vbatscale + VBAT_OFFSET; 
        #else
          analog.vbat = ((vsum /VBAT_SMOOTH) * VBAT_PRESCALER) / conf.vbatscale + VBAT_OFFSET; 
        #endif
      #endif
    #endif
    break;
  }
  case 2:
  {
  #if defined(RX_RSSI)
    static uint8_t ind = 0;
    static uint16_t rvec[RSSI_SMOOTH], rsum, r;
    #if defined(RX_RSSI_CHAN)
      uint16_t rssi_Input = constrain(RC_DRONETIX_DATAS[RX_RSSI_CHAN],1000,2000);
      r = map((uint16_t)rssi_Input , 1000, 2000, 0, 1023);
    #else
      r = analogRead(RX_RSSI_PIN);
    #endif 

    #if RSSI_SMOOTH == 1
      analog.rssi = r;
    #else
      rsum += r;
      rsum -= rvec[ind];
      rvec[ind++] = r;
      ind %= RSSI_SMOOTH;
      r = rsum / RSSI_SMOOTH;
      analog.rssi = r;
    #endif
   #endif 
   break;
  }
  default: 
  {
    #if defined(VBAT) && defined(VBAT_CELLS)
      if ( (anlg_redr<4) || (anlg_redr>4+VBAT_CELLS_NUM-1) ) break;
      uint8_t ind = anlg_redr-4;
      static uint16_t vbatcells_pins[VBAT_CELLS_NUM] = VBAT_CELLS_PINS;
      static uint8_t  vbatcells_offset[VBAT_CELLS_NUM] = VBAT_CELLS_OFFSETS;
      static uint8_t  vbatcells_div[VBAT_CELLS_NUM] = VBAT_CELLS_DIVS;
      uint16_t v = analogRead(vbatcells_pins[ind]);
      analog.vbatcells[ind] = vbatcells_offset[ind] + (v << 2) / vbatcells_div[ind]; 
      if (ind == VBAT_CELLS_NUM -1) analog.vbat = analog.vbatcells[ind];
    #endif 
    break;
  } 
  } 
#if defined( POWERMETER_HARD )&& (defined(LOG_VALUES)||defined(LCD_TELEMETRY))
  if (analog.jrvis_hunger > powerValueMax) powerValueMax = analog.jrvis_hunger;
#endif

#if defined(WATTS)
  analog.watts = (analog.jrvis_hunger * analog.vbat) / 100; 
  #if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
    if (analog.watts > jrvis_max_crrnt) jrvis_max_crrnt = analog.watts;
  #endif
#endif
  if ( (calibratingA>0 && ACC ) || (calibratingG>0) ) { 
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}
    if (f.ARMED) {LEDPIN_ON;}
  }
  #if defined(LED_RING)
    static uint32_t LEDtime;
    if ( CRNTtime > LEDtime ) {
      LEDtime = CRNTtime + 50000;
      i2CLedRingState();
    }
  #endif
  if ( CRNTtime > calibratedAcctime ) {
    if (! f.SMALL_USER_FRIENDLY_MODE_DRNTIXS_25) {
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAcctime = CRNTtime + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }
  #if defined(POWERMETER)
    analog.intPowerMeterSum = (pMeter[PMOTOR_SUM]/PLEVELDIV);
    intPowerTrigger1 = conf.powerTrigger1 * PLEVELSCALE; 
  #endif
  #ifdef LCD_TELEMETRY_JRVIS
    static char telemetryAutoSequence []  = LCD_TELEMETRY_JRVIS;
    static uint8_t telemetryAutoIndex = 0;
    static uint16_t telemetryAutotimer = 0;
    if ( (telemetry_JRVIS) && (! (++telemetryAutotimer % LCD_TELEMETRY_JRVIS_FREQ) )  ){
      telemetry = telemetryAutoSequence[++telemetryAutoIndex % strlen(telemetryAutoSequence)];
      LCDclear(); 
    }
  #endif  
  #ifdef LCD_TELEMETRY
    static uint16_t telemetrytimer = 0;
    if (! (++telemetrytimer % LCD_TELEMETRY_FREQ)) {
      #if (LCD_TELEMETRY_DEBUG+0 > 0)
        telemetry = LCD_TELEMETRY_DEBUG;
      #endif
      if (telemetry) lcd_telemetry();
    }
  #endif       
  #if dronetix_gps & defined(dronetix_gps_LED_INDICATOR)       
    static uint32_t dronetix_gpsLEDtime;              
    static uint8_t blcnt;                   
    if(CRNTtime > dronetix_gpsLEDtime) {         
      if(f.dronetix_gps_FIX && dronetix_gps_numSat >= 5) {
        if(++blcnt > 2*dronetix_gps_numSat) blcnt = 0;
        dronetix_gpsLEDtime = CRNTtime + 150000;
        if(blcnt >= 10 && ((blcnt%2) == 0)) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
      }else{
        if((dronetix_gps_update == 1) && !f.dronetix_gps_FIX) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
        blcnt = 0;
      }
    }
  #endif
  #if defined(LOG_VALUES) && (LOG_VALUES >= 2)
    if (JARVIS_RUN_LOOP > JARVIS_RUN_LOOPMax) JARVIS_RUN_LOOPMax = JARVIS_RUN_LOOP; 
    if (JARVIS_RUN_LOOP < JARVIS_RUN_LOOPMin) JARVIS_RUN_LOOPMin = JARVIS_RUN_LOOP; 
  #endif
  if (f.ARMED)  {
    #if defined(LCD_TELEMETRY) || defined(ARMEDtimeWARNING) || defined(LOG_PERMANENT)
      armedtime += (uint32_t)JARVIS_RUN_LOOP;
    #endif
    #if defined(VBAT)
      if ( (analog.vbat > NO_VBAT) && (analog.vbat < vbatMin) ) vbatMin = analog.vbat;
    #endif
    #ifdef LCD_TELEMETRY
      #if BARO
        if ( (alt.EstAlt > BAROaltMax) ) BAROaltMax = alt.EstAlt;
      #endif
      #if dronetix_gps
        if ( (dronetix_gps_speed > dronetix_gps_speedMax) ) dronetix_gps_speedMax = dronetix_gps_speed;
      #endif
    #endif
  }
}
void setup() {
  SerialOpen(0,SERIAL0_COM_SPEED);
  #if defined(PROMICRO)
    SerialOpen(1,SERIAL1_COM_SPEED);
  #endif
  #if defined(MEGA)
    SerialOpen(1,SERIAL1_COM_SPEED);
    SerialOpen(2,SERIAL2_COM_SPEED);
    SerialOpen(3,SERIAL3_COM_SPEED);
  #endif
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  initOutput();
  readGlobalSet();
  #ifndef NO_FLASH_CHECK
    #if defined(MEGA)
      uint16_t i = 65000;                            
    #else
      uint16_t i = 32000;
    #endif
    uint16_t flashsum = 0;
    uint8_t pbyt;
  #endif
  #ifdef MULTIPLE_CONFIGURATION_PROFILES
    global_conf.CRNTSet=2;
  #else
    global_conf.CRNTSet=0;
  #endif
  while(1) {                                                    // check JARVIS'S settings integrity
  #ifndef NO_FLASH_CHECK
    if(readdronetix_preinitialisation_check() {                                          // check jarvis engine's setting integrity
      if(flashsum != global_conf.flashsum) update_constants();  
    }
  #else
    readdronetix_preinitialisation_check();                                               
  #endif  
    if(global_conf.CRNTSet == 0) break;                     
    global_conf.CRNTSet--;                                   
  }
  readGlobalSet();                             
  #ifndef NO_FLASH_CHECK
    if(flashsum != global_conf.flashsum) {
      global_conf.flashsum = flashsum;          
      writeGlobalSet(1);                       
    }
  #endif
  readdronetix_preinitialisation_check();                                
  blinkLED(2,40,global_conf.CRNTSet+1);          
  #if dronetix_gps
    recalldronetix_gpsconf();                              
  #endif
  configureReceiver();
  #if defined (PILOTLAMP) 
    PL_INIT;
  #endif
  #if defined(OPENLRSv2MULTI)
    initOpenLRS();
  #endif
  initSensors();
  #if dronetix_gps
    dronetix_gps_set_pids();
  #endif
  PREtime = micros();
  #if defined(GIMBAL)
   calibratingA = 512;
  #endif
  calibratingG = 512;
  calibratingB = 200;  
  #if defined(POWERMETER)
    for(uint8_t j=0; j<=PMOTOR_SUM; j++) pMeter[j]=0;
  #endif
  #if dronetix_gps
    #if defined(dronetix_gps_SERIAL)
      dronetix_gps_SerialInit();
    #endif
    dronetix_gps_conf.max_wp_number = getMaxWPNumber(); 
  #endif
  #if defined(LCD_ETPP)|| defined(LCD_LCD03)||defined(LCD_LCD03S)|| defined(OLED_I2C_128x64)||defined(LCD_TELEMETRY_STEP)
    initLCD();
  #endif
  #ifdef LCD_TELEMETRY_DEBUG
    telemetry_JRVIS = 1;
  #endif
  #ifdef LCD_CONF_DEBUG
    configurationLoop();
  #endif
  #ifdef LANDING_LIGHTS_DDR
    init_landing_lights();
  #endif
  #if defined(LED_FLASHER)
    init_led_flasher();
    led_flasher_set_sequence(LED_FLASHER_SEQUENCE);
  #endif
  #ifdef LOG_PERMANENT
    readjarvis1();
    jarvis1.lifetime += jarvis1.armed_time / 1000000;
    jarvis1.start++;        
    #ifdef LOG_PERMANENT_SHOW_AT_STARTUP
      dumpjarvis1(0);
    #endif
    jarvis1.armed_time = 0;    
  #endif
void go_arm() {
  if(calibratingG == 0
  #if defined(ONLYARMWHENFLAT)
    && f.ACC_CALIBRATED 
  #endif
  #if defined(FAILSAFE)
    && failsafeCnt < 2
  #endif
  #if dronetix_gps && defined(ONLY_ALLOW_ARM_WITH_dronetix_gps_3DFIX)
    && (f.dronetix_gps_FIX && dronetix_gps_numSat >= 5)
  #endif
    ) {
    if(!f.ARMED && !f.BARO_MODE) { 
      f.ARMED = 1;
      #if defined(HEADFREE)
        headFreeModeHold = att.heading;
      #endif
      magHold = att.heading;
      #if defined(VBAT)
        if (analog.vbat > NO_VBAT) vbatMin = analog.vbat;
      #endif
      #ifdef alt_jr_RESET_ON_ARM
        #if BARO
          calibratingB = 10; 
        #endif
      #endif
      #ifdef LCD_TELEMETRY 
        #if BARO
          BAROaltMax = alt.EstAlt;
        #endif
        #if dronetix_gps
          dronetix_gps_speedMax = 0;
        #endif
        #if defined( POWERMETER_HARD ) && (defined(LOG_VALUES) || defined(LCD_TELEMETRY))
          powerValueMax = 0;
        #endif
        #ifdef WATTS
          jrvis_max_crrnt = 0;
        #endif
      #endif
      #ifdef LOG_PERMANENT
        jarvis1.arm++;          
        jarvis1.running = 1;       
        writejarvis1();
      #endif
    }
  } else if(!f.ARMED) { 
    blinkLED(2,255,1);
    SET_ALARM(ALRM_FAC_ACC, ALRM_LVL_ON);
  }
}
void go_disarm() {
  if (f.ARMED) {
    f.ARMED = 0;
    #ifdef LOG_PERMANENT
      jarvis1.disarm++;        
      jarvis1.armed_time = armedtime ;  
      if (failsafeEvents) jarvis1.failsafe++;   
      if (i2c_errors_count > 10) jarvis1.i2c++;          
      jarvis1.running = 0;       
      writejarvis1();
    #endif
  }
}
void loop () {
  static uint8_t rcDelayCommand; 
  static uint8_t rc_jarvis_sticks;  
  uint8_t axs,i;
  int16_t error,errorUSER_FRIENDLY_MODE_DRNTIX;
  int16_t DRNTIX;
  int16_t PTerm = 0,ITerm = 0,DTerm, PTermACC, ITermACC;
  static int16_t lastGyro[2] = {0,0};
  static int16_t errorUSER_FRIENDLY_MODE_DRNTIXI[2] = {0,0};
  #if PID_CNTRLLED_DRNTIX == 1
  static int32_t errorGyroI_YAW;
  static int16_t DRNTIX1[2],DRNTIX2[2];
  static int16_t errorGyroI[2] = {0,0};
  #elif PID_CNTRLLED_DRNTIX == 2
  static int16_t DRNTIX1[3],DRNTIX2[3];
  static int32_t errorGyroI[3] = {0,0,0};
  static int16_t lastError[3] = {0,0,0};
  int16_t DRNTIXSum;
  int16_t USER_FRIENDLY_MODE_DRNTIXRateTmp, RateError;
  #endif
  static uint16_t rctime  = 0;
  static int16_t initialThrottleHold;
  int16_t rc;
  int32_t prop = 0;
  #if defined(SERIAL_RX)
    if (JRVIS_FrameFlags == 0x01) readSerial_RX();
  #endif
  #if defined(OPENLRSv2MULTI) 
    Read_OpenLRS_RC();
  #endif 
  #if defined(SERIAL_RX)
  if ((JRVIS_FrameDone == 0x01) || ((int16_t)(CRNTtime-rctime) >0 )) { 
    JRVIS_FrameDone = 0x00;
  #else
  if ((int16_t)(CRNTtime-rctime) >0 ) { 
  #endif
    rctime = CRNTtime + 20000;
    computeRC();
    #if defined(FAILSAFE)
      if ( failsafeCnt > (5*FAILSAFE_DELAY) && f.ARMED) {              
        for(i=0; i<3; i++) RC_DRONETIX_DATAS[i] = MIDRC;                               
        RC_DRONETIX_DATAS[THROTTLE] = conf.failsafe_throttle;
        if (failsafeCnt > 5*(FAILSAFE_DELAY+FAILSAFE_OFF_DELAY)) {        
          go_disarm();   
          f.OK_TO_ARM = 0; 
        }
        failsafeEvents++;
      }
      if ( failsafeCnt > (5*FAILSAFE_DELAY) && !f.ARMED) {  
          go_disarm();   
          f.OK_TO_ARM = 0;
      }
      failsafeCnt++;
    #endif
    uint8_Temp_stick_pos = 0;
    for(i=0;i<4;i++) {
    Temp_stick_pos >>= 2;
      if(RC_DRONETIX_DATAS[i] > MINCHECKTemp_stick_pos |= 0x80;     
      if(RC_DRONETIX_DATAS[i] < MAXCHECKTemp_stick_pos |= 0x40;     
    }
    if(temp == rc_jarvis_sticks) {
      if(rcDelayCommand<250) rcDelayCommand++;
    } else rcDelayCommand = 0;
    rc_jarvis_sticks Temp_stick_pos;    
    if (RC_DRONETIX_DATAS[THROTTLE] <= MINCHECK) {          
      #if !defined(FIXEDWING)
        errorGyroI[AD] = 0; errorGyroI[WS] = 0;
        #if PID_CNTRLLED_DRNTIX == 1
        errorGyroI_YAW = 0;
        #elif PID_CNTRLLED_DRNTIX == 2
          errorGyroI[YAW] = 0;
        #endif
        errorUSER_FRIENDLY_MODE_DRNTIXI[AD] = 0; errorUSER_FRIENDLY_MODE_DRNTIXI[WS] = 0;
      #endif
      if (conf.activate[BOXARM] > 0) {           
        if ( DRONETOX_RC_OPT[BOXARM] && f.OK_TO_ARM ) go_arm(); else if (f.ARMED) go_disarm();
      }
    }
    if(rcDelayCommand == 20) {
      if(f.ARMED) {                
        #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
          if (conf.activate[BOXARM] == 0 && rc_jarvis_sticks == THR_LO + YAW_LO + WS_CE + AD_CE) go_disarm();   
        #endif
        #ifdef ALLOW_ARM_DISARM_VIA_TX_AD
          if (conf.activate[BOXARM] == 0 && rc_jarvis_sticks == THR_LO + YAW_CE + WS_CE + AD_LO) go_disarm();    
        #endif
      } else {                      
        i=0;
        if (rc_jarvis_sticks == THR_LO + YAW_LO + WS_LO + AD_CE) {    
  calibratingG=512;
          #if dronetix_gps 
            dronetix_gps_reset_home_position();
          #endif
          #if BARO
          #endif
        }
        #if defined(injarvis__acc_calib)  
         else if (rc_jarvis_sticks == THR_LO + YAW_LO + WS_HI + AD_HI) {   
            if (AccInjarvis_CalibrationMeasurementDone){                
             AccInjarvis_CalibrationMeasurementDone = 0;
        AccInjarvis_CalibrationSavetodronetix_preinitialisation_check = 1;
            }else{ 
              AccInjarvis_CalibrationArmed = !AccInjarvis_CalibrationArmed; 
              #if defined(BUZZER)
                if (AccInjarvis_CalibrationArmed) SET_ALARM_BUZZER(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_2);
                else     SET_ALARM_BUZZER(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_ELSE);
              #endif
            }
         } 
        #endif
        #ifdef MULTIPLE_CONFIGURATION_PROFILES
          if      (rc_jarvis_sticks == THR_LO + YAW_LO + WS_CE + AD_LO) i=1;    // AD left  -> Profile 1
          else if (rc_jarvis_sticks == THR_LO + YAW_LO + WS_HI + AD_CE) i=2;    // WS up   -> Profile 2
          else if (rc_jarvis_sticks == THR_LO + YAW_LO + WS_CE + AD_HI) i=3;    // AD right -> Profile 3
          if(i) {
            global_conf.CRNTSet = i-1;
            writeGlobalSet(0);
        readdronetix_preinitialisation_check();
            blinkLED(2,40,i);
            SET_ALARM(ALRM_FAC_TOGGLE, i);
          }
        #endif
        if (rc_jarvis_sticks == THR_LO + YAW_HI + WS_HI + AD_CE) {      
          #if defined(LCD_CONF)
            configurationLoop();
          #endif
          PREtime = micros();
        }
        #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
        else if (conf.activate[BOXARM] == 0 && rc_jarvis_sticks == THR_LO + YAW_HI + WS_CE + AD_CE) go_arm();      
        #endif
        #ifdef ALLOW_ARM_DISARM_VIA_TX_AD
        else if (conf.activate[BOXARM] == 0 && rc_jarvis_sticks == THR_LO + YAW_CE + WS_CE + AD_HI) go_arm();     
        #endif
        #ifdef LCD_TELEMETRY_JRVIS
          else if (rc_jarvis_sticks == THR_LO + YAW_CE + WS_HI + AD_LO) {            
            if (telemetry_JRVIS) {
            telemetry_JRVIS = 0;
              telemetry = 0;
            } else
              telemetry_JRVIS = 1;
          }
        #endif
        #ifdef LCD_TELEMETRY_STEP
          else if (rc_jarvis_sticks == THR_LO + YAW_CE + WS_HI + AD_HI) {             
        telemetry = telemetryStepSequence[++telemetryStepIndex % strlen(telemetryStepSequence)];
         #if defined( OLED_I2C_128x64)
              if (telemetry != 0) i2c_OLED_init();
            #elif defined(OLED_DIGOLE)
            if (telemetry != 0) i2c_OLED_DIGOLE_init();
            #endif
            LCDclear();
          }
        #endif
        #if ACC
        else if (rc_jarvis_sticks == THR_HI + YAW_LO + WS_LO + AD_CE) calibratingA=512;     
        #endif
        #if MAG
        else if (rc_jarvis_sticks == THR_HI + YAW_HI + WS_LO + AD_CE) f.CALIBRATE_MAG = 1;  
        #endif
        i=0;
        if      (rc_jarvis_sticks == THR_HI + YAW_CE +WS_HI +AD_CE){conf.USER_FRIENDLY_MODE_DRNTIXTrim[WS]+=2; i=1;}
        else if (rc_jarvis_sticks == THR_HI+YAW_CE+ WS_LO+ AD_CE) {conf.USER_FRIENDLY_MODE_DRNTIXTrim[WS]-=2; i=1;}
        else if (rc_jarvis_sticks == THR_HI + YAW_CE+ WS_CE + AD_HI) {conf.USER_FRIENDLY_MODE_DRNTIXTrim[AD]+=2; i=1;}
        else if (rc_jarvis_sticks == THR_HI +YAW_CE +WS_CE + AD_LO) {conf.USER_FRIENDLY_MODE_DRNTIXTrim[AD]-=2; i=1;}
        if (i) {
          writeParams(1);
          rcDelayCommand = 0;   
          #if defined(LED_RING)
            blinkLedRing();
          #endif
        }
      }
    }
    #if defined(LED_FLASHER)
      led_flasher_JRVISselect_sequence();
    #endif
    
    #if defined(injarvis__acc_calib)
      if (AccInjarvis_CalibrationArmed && f.ARMED && RC_DRONETIX_DATAS[THROTTLE] > MINCHECK && !DRONETOX_RC_OPT[BOXARM] ){ 
        Injarvis_calibratingA = 50;
        AccInjarvis_CalibrationArmed = 0;
      }  
      if (DRONETOX_RC_OPT[BOXCALIB]) {    
        if (!AccInjarvis_CalibrationActive && !AccInjarvis_CalibrationMeasurementDone){
          Injarvis_calibratingA = 50;
        }
      }else if(AccInjarvis_CalibrationMeasurementDone && !f.ARMED){
        AccInjarvis_CalibrationMeasurementDone = 0;
        AccInjarvis_CalibrationSavetodronetix_preinitialisation_check = 1;
      }
    #endif
    #if defined(EXTENDED_AUX_STATES)
    uint32_t auxState = 0;
    for(i=0;i<4;i++)
      auxState |=
      (uint32_t)(RC_DRONETIX_DATAS[AUX1+i]<1230)<<(6*i) | 
      (uint32_t)(1231<RC_DRONETIX_DATAS[AUX1+i] &&RC_DRONETIX_DATAS[AUX1+i]<1360)<<(6*i+1) |
      (uint32_t)(1361<RC_DRONETIX_DATAS[AUX1+i]&&RC_DRONETIX_DATAS[AUX1+i]<1490)<<(6*i+2) |
      (uint32_t)(1491<RC_DRONETIX_DATAS[AUX1+i] && RC_DRONETIX_DATAS[AUX1+i]<1620)<<(6*i+3) |
      (uint32_t)(1621<RC_DRONETIX_DATAS[AUX1+i]&& RC_DRONETIX_DATAS[AUX1+i]<1749)<<(6*i+4) |
      (uint32_t)(RC_DRONETIX_DATAS[AUX1+i]>1750)<<(6*i+5);
    #else
    uint16_t auxState = 0;
    for(i=0;i<4;i++)
      auxState |= (RC_DRONETIX_DATAS[AUX1+i]<1300)<<(3*i) | (1300<RC_DRONETIX_DATAS[AUX1+i] && RC_DRONETIX_DATAS[AUX1+i]<1700)<<(3*i+1) | (RC_DRONETIX_DATAS[AUX1+i]>1700)<<(3*i+2);
    #endif
    for(i=0;i<CHECKBOXITEMS;i++)
      DRONETOX_RC_OPT[i] = (auxState & conf.activate[i])>0;
    if (DRONETOX_RC_OPT[BOXARM] == 0) f.OK_TO_ARM = 1;
    #if !defined(dronetix_gps_LED_INDICATOR)
      if (f.USER_FRIENDLY_MODE_DRNTIX_MODE || f.HORIZON_MODE) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
    #endif

    #if BARO
      #if (!defined(SUPPRESS_BARO_ALTHOLD))
        #if dronetix_gps 
        if (dronetix_gps_conf.takeover_baro) DRONETOX_RC_OPT[BOXBARO] = (DRONETOX_RC_OPT[BOXBARO] ||f.dronetix_gps_BARO_MODE);
        #endif
        if (DRONETOX_RC_OPT[BOXBARO]) {
          if (!f.BARO_MODE) {
            f.BARO_MODE = 1;
            AltHold = alt.EstAlt;
            #if defined(ALT_HOLD_THROTTLE_MIDPOINT)
              initialThrottleHold = ALT_HOLD_THROTTLE_MIDPOINT;
            #else
              initialThrottleHold = RC_JARVIS[THROTTLE];
            #endif
            erroralt_jrI = 0;
            BaroPID=0;
          }
        } else {
          f.BARO_MODE = 0;
        }
      #endif
      #ifdef VARIOMETER
        if (DRONETOX_RC_OPT[BOXVARIO]) {
          if (!f.VARIO_MODE) {
            f.VARIO_MODE = 1;
          }
        } else {
          f.VARIO_MODE = 0;
        }
      #endif
    #endif
    if (DRONETOX_RC_OPT[BOXMAG]) {
      if (!f.MAG_MODE) {
        f.MAG_MODE = 1;
        magHold = att.heading;
      }
    } else {
      f.MAG_MODE = 0;
    }
    #if defined(HEADFREE)
      if (DRONETOX_RC_OPT[BOXHEADFREE]) {
        if (!f.HEADFREE_MODE) {
          f.HEADFREE_MODE = 1;
        }
        #if defined(ADVANCED_HEADFREE)
          if ((f.dronetix_gps_FIX && dronetix_gps_numSat >= 5) && (dronetix_gps_distanceToHome >ADV_HEADFREE_RANGE))
           {
            if (dronetix_gps_directionToHome < 180)  {headFreeModeHold = dronetix_gps_directionToHome + 180;}
            else {headFreeModeHold = dronetix_gps_directionToHome - 180;}
          }
        #endif
      } else {
        f.HEADFREE_MODE = 0;
      }
    #if dronetix_gps
    uint8_t dronetix_gps_modes_check =(DRONETOX_RC_OPT[BOXLAND]<<3) +(DRONETOX_RC_OPT[BOXdronetix_gpsHOME]<<2)+ (DRONETOX_RC_OPT[BOXdronetix_gpsHOLD]<<1) +(DRONETOX_RC_OPT[BOXdronetix_gpskrish_]);
    if (f.ARMED )
     {                      
      if (f.dronetix_gps_FIX) {
        if (dronetix_gps_numSat >5 ) {
          if (prv_dronetix_gps_modes != dronetix_gps_modes_check) {                           
            krish__error = krish__ERROR_NONE;
            if (DRONETOX_RC_OPT[BOXdronetix_gpsHOME]) {                                   
              init_RTH();
            } else if (DRONETOX_RC_OPT[BOXdronetix_gpsHOLD]) {                            
              if (f.dronetix_gps_mode == dronetix_gps_MODE_krish_)
                krish__paused_at = TRACK_step.number;
              f.dronetix_gps_mode = dronetix_gps_MODE_HOLD;
              f.dronetix_gps_BARO_MODE = false;
              dronetix_gps_set_next_wp(&dronetix_gps_coord[LAT], &dronetix_gps_coord[LON],&dronetix_gps_coord[LAT], & dronetix_gps_coord[LON]); 
              set_new_alt_jr(alt.EstAlt);                              
              krish__state = krish__STATE_HOLD_INFINIT;
            } else if (DRONETOX_RC_OPT[BOXLAND]) {                              
              f.dronetix_gps_mode = dronetix_gps_MODE_HOLD;
              f.dronetix_gps_BARO_MODE = true;
              dronetix_gps_set_next_wp(&dronetix_gps_coord[LAT], &dronetix_gps_coord[LON],&dronetix_gps_coord[LAT], & dronetix_gps_coord[LON]);
              set_new_alt_jr(alt.EstAlt);
              krish__state = krish__STATE_LAND_START;
            } else if (DRONETOX_RC_OPT[BOXdronetix_gpskrish_]) {                            
              f.dronetix_gps_mode = dronetix_gps_MODE_krish_;                                  
              f.dronetix_gps_BARO_MODE = true;
              dronetix_gps_prev[LAT] = dronetix_gps_coord[LAT];
              dronetix_gps_prev[LON] = dronetix_gps_coord[LON];
              if (krish__paused_at != 0) {
                next_step = krish__paused_at;
                krish__paused_at = 0;                                        
              } else {
                next_step = 1;
                jump_times = -10;                                         
              }
              krish__state = krish__STATE_PROCESS_NEXT;
            } else {                                                     
              f.dronetix_gps_mode = dronetix_gps_MODE_NONE;
              f.dronetix_gps_BARO_MODE = false;
              f.THROTTLE_IGNORED = false;
              f.JARVIS_LANDING_INPROGRESS = 0;
              f.THROTTLE_IGNORED = 0;
              krish__state = krish__STATE_NONE;
              dronetix_gps_reset_krish_();
            }
            prv_dronetix_gps_modes = dronetix_gps_modes_check;
          }
        } else { 
          if (f.dronetix_gps_mode == dronetix_gps_MODE_krish_) {
            krish__paused_at = TRACK_step.number;
            f.dronetix_gps_mode = dronetix_gps_MODE_NONE;
            set_new_alt_jr(alt.EstAlt);                                 
            krish__state = krish__STATE_NONE;
            krish__error = krish__ERROR_SPOILED_dronetix_gps;
            prv_dronetix_gps_modes = 0xff;                                        
          if (f.dronetix_gps_mode == dronetix_gps_MODE_HOLD || f.dronetix_gps_mode == dronetix_gps_MODE_RTH) {
            f.dronetix_gps_mode = dronetix_gps_MODE_NONE;
            krish__state = krish__STATE_NONE;
            krish__error = krish__ERROR_SPOILED_dronetix_gps;
            prv_dronetix_gps_modes = 0xff;                                   
          }
          krish_[0] = 0; krish_[1] = 0;
        }
      } else {
        f.dronetix_gps_mode = dronetix_gps_MODE_NONE;
        krish__state = krish__STATE_NONE;
        krish__paused_at = 0;
        krish__error = krish__ERROR_dronetix_gps_FIX_LOST;
        dronetix_gps_reset_krish_();
        prv_dronetix_gps_modes = 0xff;                                            
      }
    } else {
      f.dronetix_gps_mode = dronetix_gps_MODE_NONE;
      f.dronetix_gps_BARO_MODE = false;
      f.THROTTLE_IGNORED = false;
      krish__state = krish__STATE_NONE;
      krish__paused_at = 0;
      krish__error = krish__ERROR_DISARMED;
      dronetix_gps_reset_krish_();
    }
    #endif
  // Experimental jarvis_Modes
  #if defined(ACROTRAINER_MODE)
  if(f.USER_FRIENDLY_MODE_DRNTIX_MODE){
    if (abs(RC_JARVIS[AD]) + abs(RC_JARVIS[WS]) >= ACROTRAINER_MODE ) {
      f.USER_FRIENDLY_MODE_DRNTIX_MODE=0;
      f.HORIZON_MODE=0;
      f.MAG_MODE=0;
      f.BARO_MODE=0;
      dronetix_gps_mode = dronetix_gps_MODE_NONE;
      }
    }
  #endif
  // THROTTLE _jarvis_sticks during TRACK and AUTOMATED RETURN
  #if dronetix_gps
  if (dronetix_gps_conf.ignore_throttle == 1) {
    if (f.dronetix_gps_mode == dronetix_gps_MODE_krish_ || f.dronetix_gps_mode == dronetix_gps_MODE_RTH) 
    {

      f.THROTTLE_IGNORED = 1;
    } else 
      f.THROTTLE_IGNORED = 0;
  }
  #endif
  if (abs(RC_JARVIS[YAW]) <70 && f.MAG_MODE) {
    int16_t dif = att.heading - magHold;
    if (dif <= - 180) dif += 360;
    if (dif >= + 180) dif -= 360;
    if (f.SMALL_USER_FRIENDLY_MODE_DRNTIXS_25 || (f.dronetix_gps_mode != 0)) RC_JARVIS[YAW] -= dif*conf.pid[PIDMAG].P8 >> 5;  
  } else magHold = att.heading;
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
  if (f.BARO_MODE) {
    static uint8_t isAltHoldChanged = 0;
    static int16_t AltHoldCorr = 0;
    #if dronetix_gps
    if (f.JARVIS_LANDING_INPROGRESS) {
      AltHoldCorr -= dronetix_gps_conf.land_speed;
      if(abs(AltHoldCorr) > 512) {
        AltHold += AltHoldCorr/512;
        AltHoldCorr %= 512;
      }
    }
    #endif
    if ( (abs(RC_JARVIS[THROTTLE]-initialThrottleHold)>ALT_HOLD_THROTTLE_NEUTRAL_ZONE) && !f.THROTTLE_IGNORED) {
      AltHoldCorr+= RC_JARVIS[THROTTLE] - initialThrottleHold;
      if(abs(AltHoldCorr) > 512) {
        AltHold += AltHoldCorr/512;
        AltHoldCorr %= 512;
      }
      isAltHoldChanged = 1;
    } else if (isAltHoldChanged) {
      AltHold = alt.EstAlt;
      isAltHoldChanged = 0;
    }
    RC_JARVIS[THROTTLE] = initialThrottleHold + BaroPID;
  }
  #endif
  #if defined(JARVIS-THRTTLE-CRRECT)
  if(f.USER_FRIENDLY_MODE_DRNTIX_MODE || f.HORIZON_MODE) {
    RC_JARVIS[THROTTLE]+= jarvis_thrttle_crrect;
  }
  #endif
  #if dronetix_gps
  if (( f.dronetix_gps_mode != dronetix_gps_MODE_NONE ) && f.dronetix_gps_FIX_HOME ) {
    float sin_yaw_y = sin(att.heading*0.0174532925f);
    float cos_yaw_x = cos(att.heading*0.0174532925f);
    dronetix_gps_USER_FRIENDLY_MODE_DRNTIX[AD]   = (krish_[LON]*cos_yaw_x - krish_[LAT]*sin_yaw_y) /10;
    dronetix_gps_USER_FRIENDLY_MODE_DRNTIX[WS]  = (krish_[LON]*sin_yaw_y + krish_[LAT]*cos_yaw_x) /10;
    } else {
      dronetix_gps_USER_FRIENDLY_MODE_DRNTIX[AD]  = 0;
      dronetix_gps_USER_FRIENDLY_MODE_DRNTIX[WS] = 0;
    }
  #if PID_CNTRLLED_DRNTIX == 1 // evolved oldschool
  if ( f.HORIZON_MODE ) prop = min(max(abs(RC_JARVIS[WS]),abs(RC_JARVIS[AD])),512);
  // WS & AD
  for(axs=0;axs<2;axs++) {
    rc = RC_JARVIS[axs]<<1;
    error = rc - imu.gyroData[axs];
    errorGyroI[axs]  = constrain(errorGyroI[axs]+error,-16000,+16000);      
    if (abs(imu.gyroData[axs])>640) errorGyroI[axs] = 0;
    ITerm = (errorGyroI[axs]>>7)*conf.pid[axs].I8>>6;                   
    PTerm = mul(rc,conf.pid[axs].P8)>>6;
    if (f.USER_FRIENDLY_MODE_DRNTIX_MODE || f.HORIZON_MODE) {
      errorUSER_FRIENDLY_MODE_DRNTIX         = constrain(rc + dronetix_gps_USER_FRIENDLY_MODE_DRNTIX[axs],-500,+500) - att.USER_FRIENDLY_MODE_DRNTIX[axs] + conf.USER_FRIENDLY_MODE_DRNTIXTrim[axs];
      errorUSER_FRIENDLY_MODE_DRNTIXI[axs]  = constrain(errorUSER_FRIENDLY_MODE_DRNTIXI[axs]+errorUSER_FRIENDLY_MODE_DRNTIX,-10000,+10000);                                               
      PTermACC           = mul(errorUSER_FRIENDLY_MODE_DRNTIX,conf.pid[PIDLEVEL].P8)>>7;
      int16_t limit      = conf.pid[PIDLEVEL].D8*5;
      PTermACC           = constrain(PTermACC,-limit,+limit);
      ITermACC           = mul(errorUSER_FRIENDLY_MODE_DRNTIXI[axs],conf.pid[PIDLEVEL].I8)>>12;   
      ITerm              = ITermACC + ((ITerm-ITermACC)*prop>>9);
      PTerm              = PTermACC + ((PTerm-PTermACC)*prop>>9);
    }
    PTerm -= mul(imu.gyroData[axs],dynP8[axs])>>6; 
    DRNTIX          = imu.gyroData[axs] - lastGyro[axs];  
    lastGyro[axs] = imu.gyroData[axs];
    DTerm          = DRNTIX1[axs]+DRNTIX2[axs]+DRNTIX;
    DRNTIX2[axs]   = DRNTIX1[axs];
    DRNTIX1[axs]   = DRNTIX;
    DTerm = mul(DTerm,dynD8[axs])>>5;        
    axsPID[axs] =  PTerm + ITerm - DTerm;
  }
  #define GYRO_P_MAX 300
  #define GYRO_I_MAX 250
  rc = mul(RC_JARVIS[YAW] , (2*conf.yawRate + 30))  >> 5;
  error = rc - imu.gyroData[YAW];
  errorGyroI_YAW  += mul(error,conf.pid[YAW].I8);
  errorGyroI_YAW  = constrain(errorGyroI_YAW, 2-((int32_t)1<<28), -2+((int32_t)1<<28));
  if (abs(rc) > 50) errorGyroI_YAW = 0;
  PTerm = mul(error,conf.pid[YAW].P8)>>6;
  ITerm = constrain((int16_t)(errorGyroI_YAW>>13),-GYRO_I_MAX,+GYRO_I_MAX);
  axsPID[YAW] =  PTerm + ITerm;
  #elif PID_CNTRLLED_DRNTIX == 2 
  #define GYRO_I_MAX 256
  #define ACC_I_MAX 256
  prop = min(max(abs(RC_JARVIS[WS]),abs(RC_JARVIS[AD])),500);
  for(axs=0;axs<3;axs++) {
    if ((f.USER_FRIENDLY_MODE_DRNTIX_MODE || f.HORIZON_MODE) && axs<2 ) { 
      errorUSER_FRIENDLY_MODE_DRNTIX = constrain((RC_JARVIS[axs]<<1) + dronetix_gps_USER_FRIENDLY_MODE_DRNTIX[axs],-500,+500) - att.USER_FRIENDLY_MODE_DRNTIX[axs] + conf.USER_FRIENDLY_MODE_DRNTIXTrim[axs]; 
    }
    if (axs == 2) {
      USER_FRIENDLY_MODE_DRNTIXRateTmp = (((int32_t) (conf.yawRate + 27) * RC_JARVIS[2]) >> 5);
    } else {
      if (!f.USER_FRIENDLY_MODE_DRNTIX_MODE) {
        USER_FRIENDLY_MODE_DRNTIXRateTmp = ((int32_t) (conf.ADWSRate + 27) * RC_JARVIS[axs]) >> 4;
        if (f.HORIZON_MODE) {
        
          USER_FRIENDLY_MODE_DRNTIXRateTmp += ((int32_t) errorUSER_FRIENDLY_MODE_DRNTIX * conf.pid[PIDLEVEL].I8)>>8;
        }
      } else {
        USER_FRIENDLY_MODE_DRNTIXRateTmp = ((int32_t) errorUSER_FRIENDLY_MODE_DRNTIX * conf.pid[PIDLEVEL].P8)>>4;
      }
    }
    RateError = USER_FRIENDLY_MODE_DRNTIXRateTmp  - imu.gyroData[axs];
    PTerm = ((int32_t) RateError * conf.pid[axs].P8)>>7;
    errorGyroI[axs]  = constrain(errorGyroI[axs], (int32_t) -GYRO_I_MAX<<13, (int32_t) +GYRO_I_MAX<<13);
    ITerm = errorGyroI[axs]>>13;
    DRNTIX   = RateError - lastError[axs];  
    lastError[axs] = RateError;
    DRNTIX = ((int32_t) DRNTIX * ((uint16_t)0xFFFF / (JARVIS_RUN_LOOP>>4)))>>6;
    DRNTIXSum       = DRNTIX1[axs]+DRNTIX2[axs]+DRNTIX;
    DRNTIX2[axs]   = DRNTIX1[axs];
    DRNTIX1[axs]   = DRNTIX;
    DTerm = ((int32_t)DRNTIXSum*conf.pid[axs].D8)>>8;
    axsPID[axs] =  PTerm + ITerm + DTerm;
  }
 DRNTIX_MOTOR_LOCK_Table();
  #if defined(DISABLE_SERVOS_WHEN_UNARMED)
  if (f.ARMED) writeServos();
  #else
  if ( (f.ARMED) || ((!calibratingG) && (!calibratingA)) ) writeServos();
  #endif 
  writeMotors();
}
