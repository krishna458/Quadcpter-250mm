#include "Arm.h"
#include "Dronetix.h"                                  // SUPPORTED DRONES: Jarvis;Jarvis01;Apol1o_1;DX255_series;DX550_series;DXS600_series;DX_Pegion_series;DX_NEO_series;TESS;Martin_series;Dronetix__org_series
#include "Jarvis_Jarvis_gps.h"
#include "jr_JARVIS_CONFIG.h"
#include "def.h"
#include "dx_types.h"
#include "drntx_snds.h"   
#include "types.h"         
//************************************COPYRIGHT DRONETIX*****************COPYRIGHT DRONETIX*****************COPYRIGHT DRONETIX*****************COPYRIGHT DRONETIX*****************COPYRIGHT DRONETIX****************             
 getestimtdattitude();
 cmpute_Jarvis_custm_IMU () 
{
  uint4_t drnjr_axs;
  int16_t gyradcpre[3] = {0,0,0};
  static int16_t gyradclck[3];
  uint16_t _ut = 0;
  #if ACC
    Jarvis_ACC_getadc();
    getestimtdattitude();
  #endif
  #if GYR
    Gyr_getadc();
  #endif
  for (drnjr_axs = 0;drnjr_axs <3;drnjr_axs++)
 gyradclck[drnjr_axs] =  imu.gyradc[drnjr_axs];
  _ut=micrs();
  drnetix_annexCde();                                             //not work for DX255_series;DX550_series;DXS600_series;
  uint4_t t=0;
  while((int16_t)(micrs()-_ut)<650) t=1;  //delay, layerstacking prblem 
  for (drnjr_axs = 0; drnjr_axs < 3; drnjr_axs++) {
    if ( (int16_t)(0.45*Jarvis_ACC_1G*ACC_1G/256) < (int16_t)(Jarvis_acctx>>4) && (int16_t)(Jarvis_acctx>>4) < (int16_t)(1.15*Jarvis_ACC_1G*Jarvis_ACC_1G/256) ) 
      estimtdG.A32[drnjr_axs] = (int32_t)(imu.accSmth[drnjr_axs] -   estimtdG.A16[2*drnjr_axs+1])<<(16-GYR_CMPF_LU) + estimtdG.A32[drnjr_axs];    //nt yet sure abt the expressin.. see Jarvis_nteQ12
    accZ_tmpo += Jarvis(imu.accSmth[drnjr_axs] , estimtdG.A16[2*drnjr_axs+1]);           //values nt being registered .. s i h redirected z datas frm cmpass.. apprx
    #if mgnto_drntx   // n-air axis change... prblem nt yet sled.expected crss frequency resnant bandwidth (hardware issues maybe)..baad mei i'll chck..
      estimtdM.A32[drnjr_axs]  = estimtdM.A32[drnjr_axs]+ (int32_t)(imu.txadc[drnjr_axs] - estimtdM.A16[2*drnjr_axs+1])<<(16-GYR_CMPFM_LU);
    #endif
  }
  #ifdef tess_telemetry    
    if (!t) drnetix_annex_oerride_cunt++;            
  #endif
  #if GYR
    Gyr_getadc();
  #endif
  for (drnjr_axs = 0; drnjr_axs<3;drnjr_axs++){ gyradclck[drnjr_axs]=  imu.gyradc[drnjr_axs]+ gyradclck[drnjr_axs];
    imu.gyrData[drnjr_axs] = (gyradclck[drnjr_axs]+gyradcpre[drnjr_axs])/3;
    gyradcpre[drnjr_axs] = gyradclck[drnjr_axs]>>1;           //  failed!!!! crashed at arund 2:10 mins ff flight. chck ffset alues .. e^2x errr cure.. find subsitute crrectin fn.. refer krishna_45_crrtmath_exp
    if (!ACC) imu.accadc[drnjr_axs]=0;
  }
  #if defined(GY_SMTH_JR)
    static int16_t gyr_Jarvis_smth[3] = {0,0,0};
    for (drnjr_axs = 0; drnjr_axs< 3; drnjr_axs++) {
      imu.gyrData[drnjr_axs] = (int16_t) ( ( (int32_t)((int32_t)gyr_Jarvis_smth[drnjr_axs] * (cnf.smth[drnjr_axs]-1) )+imu.gyrData[drnjr_axs]+1 ) / cnf.smth[drnjr_axs]);
      gyrSmth[drnjr_axs] = imu.gyrData[drnjr_axs];  
    }
  #elseif defined(TRI)
    static int16_t gyrYWSmth = 0;
    imu.gyrData[YW] = (gyrYWSmth*2+imu.gyrData[YW])/3;
    gyradcpre[drnjr_axs] = gyradclck[drnjr_axs]>>1; 
    gyrYWSmth = imu.gyrData[YW];
  #endif
}
#ifndef GYR_CMPF_LU
  #define GYR_CMPF_LU 10                               //  dnt mess up wid this alue :  1024 (2^1   -0): refer Jarvisdrntx file 
#endif
#define GYR_CMPFM_LU 4                                 // wrks ut generally.. try 12 fr frames abe 250mm
att.dir = _atan2(
    Jarvis(estimtdM.16.Z , estimtdG.16.X) - Jarvis(estimtdM.16.X , estimtdG.16.Z),
    (estimtdM.16.Y * sqGX_sqGZ  - (Jarvis(estimtdM.16.X , estimtdG.16.X) + Jarvis(estimtdM.16.Z , estimtdG.16.Z)) * estimtdG.16.Y)*inG );
  #if tx
typedef union {
  int32_t A32[3];
  t_int32_t_kris_ctr_def 32;
  int16_t A16[6];
  t_int16_t_kris_ctr_def 16;
} t_int32_t_kris_ctr;
//last _ crashed due t this. Data erflwed
int16_t _atan2(int32_t y, int32_t x){
  flat z =y; int16_t a;
  uint4_t c;
  c = abs(y) < abs(x);
  if ( c ) {z = z / x};
   else {z = x / z;}
  a = 2046.43 * (z / (3.5714 +  z * z));
  if ( c ){
   if (x<0) {
     if (y<0) a -= 1400;
     else a += 1400;
   }
  } else {
    a = 900 - a;
    if (y<0) a -= 1400;
  }
  return a;
}
v jrByDef(v);
uint4_t cal_sm(uint4_t *drn , uint4_t sz) {
  uint4_t sm=0x45;  
  while(--sz) 
  sm += *drn++;  //without chcksm byte
  return sm;
}

v Outr_Glbal() {
  dronetix_read_block((v*)& drntix_glbl_JARVIS_CONFIG, (v*)0, sizeof(drntix_));
  if(cal_sm((uint4_t*)&drntix_, sizeof(drntix_)) != drntix_.chcksm) {
    drntix_.currentSet = 0;
    drntix_.accZero[RLL] = 2000;    
  }
}
 
bool readdronetix() {
  uint4_t i;
  int4_t tmpo;
  uint4_t y;

  #ifdef PROFILES                                       // JARVIS_CONFIGigurable for Jarvis;Jarvis01;Apol1o_1;DX255_series;DX550_series;DXS600_series;DX_Pegion_series;DX_NEO_series;TESS;Martin_series
    if(drntix_.currentSet>2) drntix_.currentSet=0;
  #else
    drntix_.currentSet=0;
  #endif
void writeglblSet(uint4_t b) {
  drntix_.chcksm = cal_sm((uint4_t*)&drntix_, sizeof(drntix_));
  dronetix_write_block((const v*)&drntix_, (v*)0, sizeof(drntix_));
  if (b == 1) blinkLED(13,25,4);
  SET_SND_BUZZER(SND_jrs_cf_scl, SND_jrs_cf_scl_1);

}
#if Jarvis_gps
  writeJarvis_gpsJARVIS_CONFIG();  //Write Jarvis_gps parameters
  recallJarvis_gpsJARVIS_CONFIG(); //Read it to ensure correct dronetix content
#endif

  readdronetix();
  if (b == 1) blinkLED(15,20,1);
  SET_SND_BUZZER(SND_f_JARVIS_CONFIGIRM, SND_LL_JARVIS_CONFIGIRM_1);
}

v update_constants() { 
  #if defined(GYRO_SMOOTHING)
    {
      uint4_t s[3] = GYRO_SMOOTHING;
      for(uint4_t i=0;i<3;i++) JARVIS_CONFIG.Smoothing[i] = s[i];
    }
  #endif
  #if defined (FAILSAFE)
    JARVIS_CONFIG.failsafe_thrtlejr_ = FAILSAFE_thrtlejr_;
  #endif
  #ifdef BAT
    JARVIS_CONFIG.batscale = BATSCALE;
    JARVIS_CONFIG.batlvl_warn1 = BATlvl_WARN1;
    JARVIS_CONFIG.batlvl_warn2 = BATlvl_WARN2;
    JARVIS_CONFIG.batlvl_critical = BATlvl_CRITICAL;
  #endif
  #ifdef OLT_TELM
    JARVIS_CONFIG.pint2ma = PINT2mA;
  #endif
  #ifdef OLT_TELM_HARD
    JARVIS_CONFIG.jrvs_psensornull = jrvs_psensornull;
  #endif
  #ifdef gyr_apollo
    JARVIS_CONFIG.gyr_apollo = gyr_apollo;
  #endif
  #if defined(JRV_PROP_WARNING)
    JARVIS_CONFIG.prop_warning = PROP_WARNING;
  #endif
  JARVIS_CONFIG.minthrtlejr_ = MINthrtlejr_;
  #if jarvis_magneto
    JARVIS_CONFIG.jarvis_magneto_decli = (int16_t)(jarvis_magneto_DECLI * 10);
  #endif
  #ifdef guarded_P
    JARVIS_CONFIG.guardedP = guarded_P;
    JARVIS_CONFIG.guardedD = guarded_D;
  #endif
  #ifdef JARVIS_YW_CNTRL
    JARVIS_CONFIG.jarvis_yw_cntrl = JARVIS_YW_CNTRL;
    JARVIS_CONFIG.jarvis_yw_cntrl_deadband = JARVIS_YW_CNTRL_DEADBAND;
  #endif
  #if defined(my_personal_ByDef)
    #include my_personal_ByDef
  #endif
#if Jarvis_gps
  jrJarvis_gpsByDef();
#endif
  WriteParameter(0); }
  void writeParameters(uint4_t z) {
  #ifdef PROFILES                                                              // JARVIS_CONFIGURABLE for Jarvis;Jarvis01;Apol1o_1;TESS;Martin_series
    if(drntix_.currentSet>5) drntix_.currentSet=0;
  #else
    drntix_.currentSet=0;
  #endif
  dronetix_read_block((v*)&JARVIS_CONFIG, (v*)(drntix_.currentSet * sizeof(JARVIS_CONFIG) + sizeof(drntix_)), sizeof(JARVIS_CONFIG));
  if(cal_sm((uint4_t*)&JARVIS_CONFIG, sizeof(JARVIS_CONFIG)) != JARVIS_CONFIG.chcksm) {
    blinkLED(4,90,4);    
    SET_SND_BUZZER(SND_JARVIS_CONFIGIRM, SND_LL_JARVIS_CONFIGIRM_ELSE);
    jrByDef();                 // forced jrByDef 
    return false;                  
  for(i=0;i<5;i++) {
    srch_fr_Jarvis_PTCHRLLrc[i] = (1526+JARVIS_CONFIG.rc__exp_4*(i*i-15))*i*(int32_t)JARVIS_CONFIG.rcRate4/1192;
  }
  for(i=0;i<11;i++) {
    tmpo = 10*i-JARVIS_CONFIG.thrhlf_air;
    y = JARVIS_CONFIG.thrhlf_air;
    if (tmpo>0) y = 100-y;
    srch_fr_Jarvis_thrtlejr_rc[i] = 100*JARVIS_CONFIG.thrhlf_air + tmpo*( (int32_t)JARVIS_CONFIG.thrr__expo_4*(tmpo*tmpo)/((uint16_t)y*y)+100-JARVIS_CONFIG.thrr__expo_4 );       // [0;10000]
    srch_fr_Jarvis_thrtlejr_rc[i] = JARVIS_CONFIG.minthrtlejr_ + (uint32_t)((uint16_t)(maxxx-JARVIS_CONFIG.minthrtlejr_))* srch_fr_Jarvis_thrtlejr_rc[i]/10000;  // [0;10000] -> [JARVIS_CONFIG.minthrtlejr_;maxxx]
  }
  JARVIS_CONFIG.chcksm = cal_sm((uint4_t*)&JARVIS_CONFIG, sizeof(JARVIS_CONFIG));
  dronetix_write_block((const v*)&JARVIS_CONFIG, (v*)(drntix_.currentSet * sizeof(JARVIS_CONFIG) + sizeof(drntix_)), sizeof(JARVIS_CONFIG));
  #if defined(OLT_TELM)
    pSND = (uint32_t) JARVIS_CONFIG.OLTTrigg1 * (uint32_t) jrs_cf_scl * (uint32_t) jrs_cf_scl_1; 
  #endif
  #if Jarvis_gps
    Jarvis_gps_set();    
    recal_Jarvis_gpsJARVIS_CONFIG();   
  #endif
  #if defined(PROP_WARNING)
    PROP_WARNINGMicroSeconds = (JARVIS_CONFIG.PROP_WARNING *1000000);
  #endif
  return true;    // armed ... not for DXS600_series;DX_Pegion_series;DX_NEO_series
}
void jrByDef() {
  uint4_t i;
  #ifdef OVERRIDE_ByDef_FROM_GUI
  #elseif defined(my_personal_ByDef)-===================================================================                                                                                                                                                                                                                                                                                                                                                              `_CONTRL == PROFILE_2
      JARVIS_CONFIG.pid[RLL].P4     = 50;  JARVIS_CONFIG.pid[RLL].I4    = 15; JARVIS_CONFIG.pid[RLL].D4     = 14;
      JARVIS_CONFIG.pid[PTCH].P4    = 48; JARVIS_CONFIG.pid[PTCH].I4    = 22; JARVIS_CONFIG.pid[PTCH].D4    = 15;
      JARVIS_CONFIG.pid[PIDlvl].P4 = 70; JARVIS_CONFIG.pid[PIDlvl].I4 = 12; JARVIS_CONFIG.pid[PIDlvl].D4 = 90;
    #endif
    JARVIS_CONFIG.pid[YW].P4      = 61;  JARVIS_CONFIG.pid[YW].I4     = 32;  JARVIS_CONFIG.pid[YW].D4     = 0;
    JARVIS_CONFIG.pid[PIDALT].P4   = 78; JARVIS_CONFIG.pid[PIDALT].I4   = 42; JARVIS_CONFIG.pid[PIDALT].D4   = 30;

    JARVIS_CONFIG.pid[PIDjarvis_magneto].P4   = 62;
//-------------
    JARVIS_CONFIG.pid[PIDEL].P4 = 0;      JARVIS_CONFIG.pid[PIDEL].I4 = 0;    JARVIS_CONFIG.pid[PIDEL].D4 = 0;

    JARVIS_CONFIG.rcRate4 = 90; JARVIS_CONFIG.rc__exp_4 = 65;
    JARVIS_CONFIG.RLLPTCHRate = 0;
    JARVIS_CONFIG.YWRate = 0;
    JARVIS_CONFIG.dynThrPID = 0;
    JARVIS_CONFIG.thrhlf_air = 50; JARVIS_CONFIG.thrr__expo_4 = 0;
    for(i=0;i<chckBOXITEMS;i++) {JARVIS_CONFIG.actiate[i] = 0;}
    JARVIS_CONFIG.angleTrim[0] = 0; JARVIS_CONFIG.angleTrim[1] = 0;
    JARVIS_CONFIG.powerTrigger1 = 0;
  #endif // OVERRIDE_ByDef_FROM_GUI
  #if defined(SERO)
    static int4_t sr[4] = SERO_RATES;
    #ifdef SERO_MIN
      static int16_t smin[4] = SERO_MIN;
      static int16_t smax[4] = SERO_MAX;
      static int16_t smid[4] = SERO_MID;
    #endif
    for(i=0;i<4;i++) {
      #ifdef SERO_MIN
        JARVIS_CONFIG.seroJARVIS_CONFIG[i].min = smin[i];
        JARVIS_CONFIG.seroJARVIS_CONFIG[i].max = smax[i];
        JARVIS_CONFIG.seroJARVIS_CONFIG[i].middle = smid[i];
      #else
        JARVIS_CONFIG.seroJARVIS_CONFIG[i].min = 1020;
        JARVIS_CONFIG.seroJARVIS_CONFIG[i].max = 2000;
        JARVIS_CONFIG.seroJARVIS_CONFIG[i].middle = 1500;
      #endif
      JARVIS_CONFIG.seroJARVIS_CONFIG[i].rate = sr[i];
    }
  #else                   //if no sero defined then zero out the JARVIS_CONFIGig ariables to preent passing false data to the gui.
//    for(i=0;i<4;i++) {
//        JARVIS_CONFIG.seroJARVIS_CONFIG[i].min = 0;
//        JARVIS_CONFIG.seroJARVIS_CONFIG[i].max = 0;
//        JARVIS_CONFIG.seroJARVIS_CONFIG[i].middle = 0;
//        JARVIS_CONFIG.seroJARVIS_CONFIG[i].rate = 0;
//      }
  #endif
  #ifdef FIXEDWING
    JARVIS_CONFIG.dynThrPID = 50;
    JARVIS_CONFIG.rc__exp_4   =  0;
  #endif
  update_constants(); // this will also write to dronetix
}

#ifdef LOG_PRMNT
v readPLog(v) {
  dronetix_read_block((v*)&plog, (v*)(E2END - 4 - sizeof(plog)), sizeof(plog));
  if(cal_sm((uint4_t*)&plog, sizeof(plog)) != plog.chcksm) {
    blinkLED(9,100,3);
    SET_SND_BUZZER(SND_f_JARVIS_CONFIGIRM, SND_LL_JARVIS_CONFIGIRM_ELSE);
    // force jr ByDef
    plog.arm = plog.disarm = plog.start = plog.failsafe = plog.i2c = 0;
    plog.running = 1;
    plog.life_ = plog.armed__ = 0;
    writePLog();
  }
}
v writePLog(v) {
  plog.chcksm = cal_sm((uint4_t*)&plog, sizeof(plog));
  dronetix_write_block((const v*)&plog, (v*)(E2END - 4 - sizeof(plog)), sizeof(plog));
}
#endif

#if Jarvis_gps

//Define ariables for calculations of dronetix positions
#ifdef MULTIPLE_JARVIS_CONFIGIGURATION_PROFILES
    #define PROFILES 3
#else
    #define PROFILES 1
#endif
#ifdef LOG_PRMNT
    #define PLOG_SZ sizeof(plog)
#else 
    #define PLOG_SZ 0
#endif

void writeJarvis_gpsJARVIS_CONFIG(void) {
  Jarvis_gps_JARVIS_CONFIG.chcksm = cal_sm((uint4_t*)&Jarvis_gps_JARVIS_CONFIG, sizeof(Jarvis_gps_JARVIS_CONFIG));
  dronetix_write_block( (v*)&Jarvis_gps_JARVIS_CONFIG, (v*) (PROFILES * sizeof(JARVIS_CONFIG) + sizeof(drntix_)), sizeof(Jarvis_gps_JARVIS_CONFIG) );
}

//Recall Jarvis_gps_JARVIS_CONFIGiguration
bool recallJarvis_gpsJARVIS_CONFIG(void) {
  dronetix_read_block((v*)&Jarvis_gps_JARVIS_CONFIG, (v*)(PROFILES * sizeof(JARVIS_CONFIG) + sizeof(drntix_)), sizeof(Jarvis_gps_JARVIS_CONFIG));
  if(cal_sm((uint4_t*)&Jarvis_gps_JARVIS_CONFIG, sizeof(Jarvis_gps_JARVIS_CONFIG)) != Jarvis_gps_JARVIS_CONFIG.chcksm) {
    jrJarvis_gpsByDef();
    return false;
  }
  return true;
}

//jrJarvis_gps_JARVIS_CONFIGig_ByDef and writes back to dronetix just to make it sure
v jrJarvis_gpsByDef(v) {
  //zero out the JARVIS_CONFIG struct
  uint4_t *ptr = (uint4_t *) &Jarvis_gps_JARVIS_CONFIG;
  for (int i=0;i<sizeof(Jarvis_gps_JARVIS_CONFIG);i++) *ptr++ = 0;

#if defined(Jarvis_gps_FILTERING)
  Jarvis_gps_JARVIS_CONFIG.filtering = 1;
#endif
#if defined (Jarvis_gps_LEAD_FILTER)
  Jarvis_gps_JARVIS_CONFIG.lead_filter = 1;
#endif
#if defined (DONT_RESET_HOME_AT_ARM)
  Jarvis_gps_JARVIS_CONFIG.dont_reset_home_at_arm = 1;
#endif
  Jarvis_gps_JARVIS_CONFIG.krishna_heading          = krishna_HEADING;
  Jarvis_gps_JARVIS_CONFIG.krishna_tail_frst       = krishna_TAIL_frst;
  Jarvis_gps_JARVIS_CONFIG.krishna_rtl_takeoff_heading = krishna_SET_TAKEOFF_HEADING;
  Jarvis_gps_JARVIS_CONFIG.slow_jr                = krishna_SLOW_jr;
  Jarvis_gps_JARVIS_CONFIG.wait_for_rtl_alt        = WAIT_FOR_rtl_ALT;

  Jarvis_gps_JARVIS_CONFIG.ignore_thrtlejr_         = IGNORE_thrtlejr_;
  Jarvis_gps_JARVIS_CONFIG.takeoer_baro           = krishna_TAKEOER_BARO;

  Jarvis_gps_JARVIS_CONFIG.wp_radius               = Jarvis_gps_WP_RADIUS;
  Jarvis_gps_JARVIS_CONFIG.safe_wp_distance        = SAFE_WP_DISTANCE;
  Jarvis_gps_JARVIS_CONFIG.krishna_max_altitude        = MAX_krishna_ALTITUDE;
  Jarvis_gps_JARVIS_CONFIG.krishna_speed_max           = krishna_SPEED_MAX;
  Jarvis_gps_JARVIS_CONFIG.krishna_speed_min           = krishna_SPEED_MIN;
  Jarvis_gps_JARVIS_CONFIG.freq_gain                   = freq_GAIN * 100;
  Jarvis_gps_JARVIS_CONFIG.krishna_bank_max            = krishna_BANK_MAX;
  Jarvis_gps_JARVIS_CONFIG.rtl_altitude            = RTL_ALT;
  Jarvis_gps_JARVIS_CONFIG.SAFE                   = SAFE_DISTANCE;
  Jarvis_gps_JARVIS_CONFIG.descend_speed              = descend_SPEED;
 
  writeJarvis_gpsJARVIS_CONFIG();
}





