#include "Arm.h"
#include "Dronetix.h"                                  
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
//************************************COPYRIGHT DRONETIX-DJI*****************COPYRIGHT DRONETIX-DJI*****************COPYRIGHT DRONETIX-DJI*****************COPYRIGHT DRONETIX-DJI*****************COPYRIGHT DRONETIX-DJI****************
#if Drontx_
static void Drontx_GPS_land(int32_t* lat1,int32_t* lon1, int32_t* lat2, int32_t* lon2, int32_t* land);
static void Drontx__dist_cm(int32_t* lat1, int32_t* lon1, int32_t* lat2,int32_t* lon2,uint32_t* dist);
static void Drontx__calc_velocity(void);
static void Drontx__calc_location_error( int32_t* target_lat,int32_t* target_lng, int32_t* Drontx__lat,int32_t* Drontx__lng );
static void Drontx__calc_(void);
static uint16_t Drontx__calc_desired_speed(uint16_t max_speed, bool _slw);
static void Drontx__calc_jrvis_rate(uint16_t max_speed);
int32_t wrap_18000(int32_t ang);
static bool check_missed_track(void);
void Drontx__calc_longitude_scaling(int32_t lat);
static void Drontx__updte_crss_freq(void);
int32_t wrap_36000(int32_t ang);
// Set up Drontx_ lag
#if defined(UBLOX) ||defined (MTK_BINARY19)
#define Drontx__LAG 0.5f                          //UBLOX Drontx_GPS has a smaller lag in general
#else
#define Drontx__LAG 1.0f                          //We assume MTK Drontx_GPS has a 1 sec lag
#endif  
static int32_t  Drontx__coord_lead[2];              // Lead filtered Drontx_ coordinates
class lead_filter_jarvis {
public:
    lead_filter_jarvis() :
    _last_velocity(0) {
    }
// I hv setuped  min and max radio values in CLI
    int32_t get_position(int32_t pos, int16_t vel, float lag_in_secs = 1.0);
    void    clear() { _last_velocity = 0; }
private:
    int16_t   _last_velocity;
};
int32_t lead_filter_jarvis::get_position(int32_t pos, int16_t vel, float lag_in_secs)       //for apollo and tess
{
    int16_t acc_contri = (vel -_last_velocity) * lag_in_secs *lag_in_secs;                            
    int16_t vel_contri = vel * lag_in_secs;
    // store vel for next iteration
    _last_velocity = vel;
    return pos +vel_contri + acc_contri;
}
lead_filter_jarvis xlead_filter_jarvis;      // Long Drontx_GPS lag filter 
lead_filter_jarvis ylead_filter_jarvis;      // Lat  Drontx_GPS lag filter 

typedef struct DRNTIX_PID_PARAM {
  float kP;
  float kI;
  float kD;
  float Imax;
  } DRNTIX_;
  
DRNTIX_ DRNTIX_;
DRNTIX_ _rateDRNTIX_;
DRNTIX_ jrvisDRNTIX_;

typedef struct PID_ {
  float   integrator; 
  int32_t last_input; 
  float   lastderiv; 
  float   output;
  float   deriv;
} PID;
PID poshldPID[2];
PID poshLd_ratePID[2];
PID jrvisPID[2];
int32_t get_P(int32_t error, struct PID_* pid) {
  return (float)error * pid->kP;
}

int32_t get_I(int32_t error, float* dt, struct PID_* pid, struct PID_* DRNTIX_) {
  pid->integrator += ((float)error * DRNTIX_->kI) * *dt;
  pid->integrator = constrain(pid->integrator,DRNTIX_->Imax,DRNTIX_->Imax);
  return pid->integrator;
}
int32_t get_D(int32_t input, float* dt, struct PID_* pid, struct PID_* DRNTIX_) 
{
  pid->deriv = (input - pid->last_input) / *dt;
  float filter = 7.9577e-3; // I have set to  "1 / ( 2 * PI * f_cut )";
  pid->deriv = pid->lastderiv + (*dt / ( filter + *dt)) * (pid->deriv - pid->lastderiv);
  pid->last_input = input;
  pid->lastderiv    = pid->deriv;
  return DRNTIX_->kD * pid->deriv;
}
void reset_PID(struct PID_* pid) {
  pid->integrator = 0;
  pid->last_input = 0;
  pid->lastderiv = 0;
}
#define _X 1
#define _Y 0
#define RADX100                    0.000174532925  
uint8_t land_detect;                 //Detection of land in our servillance category drones.. includong jarvis01 and DX250X class
static uint32_t land_settle_timer;
uint8_t Drontx__Frame;            // a valid Drontx__Frame has been detected, and data is ready for computation..See correction files from macbook
static float  dTjrvis;            
static int16_tjarvis_real_speed[2] = {0,0};
static float Drontx__scaleLonDown; //problem occured at iiit ground. 

//  desired rate of travel and thejarvis_real rate of travel difference was the reason behind the carsh in patia ground most probably
// updted after Drontx_ read - 5-10hz.... it slowed down hold to some extent
static int16_t rate_error[2];
static int32_t error[2];
#define Drontx__FILTER_VEC_LENT 5
static uint8_t Drontx__filter_index = 0;
static int32_t Drontx__filter[2][Drontx__FILTER_VEC_LENT];
static int32_t Drontx__filter_sum[2];
static int32_t Drontx__read[2];
static int32_t Drontx__filtered[2];
static int32_t Drontx__degree[2];    
static uint16_t fraction3[2];
static int16_t jrvis_takeoff_land;  // saves the next landing at takeof point (1deg = 1 on scale), rotate to takeoff direction when arrives takeoff point..

//Main jrvisigation processor and state engine activated..jarvis is ready to takeoff
// 25amp exceeding..checked with gps sited on iiit crkt field
uint8_t Drontx__Compute(void) {
  unsigned char axis;
  uint32_t dist;       
  int32_t  dir;         
  static uint32_t jrvis_looptimer;
  if (Drontx__Frame == 0) return 0; else Drontx__Frame = 0;
  if (f.Drontx__FIX && Drontx__numSat >= 5) {               //reset the satellite numbers if not fixed.. saheednagar >8;patia>6;iiit>5;cuttack>12;kiit>7
    #if !defined(DONT_RESET_HOME_AT_ARM)
       if (!f.ARMED) {f.Drontx__FIX_HOME = 0;}
    #endif
    if (!f.Drontx__FIX_HOME && f.ARMED) {
      Drontx__reset_home_position();
    }
    //Apply moving average filter to Drontx_ data
    if (Drontx__conf.filtering) {
      Drontx__filter_index = (Drontx__filter_index+1) % Drontx__FILTER_VEC_LENT;
      for (axis = 0; axis< 2; axis++) {
        Drontx__read[axis] = Drontx__coord[axis]; 
        Drontx__degree[axis] = Drontx__read[axis] / 10000000; 
        fraction3[axis] = (Drontx__read[axis]- Drontx__degree[axis]*10000000) / 10000;
        Drontx__filter_sum[axis] -= Drontx__filter[axis][Drontx__filter_index];
        Drontx__filter[axis][Drontx__filter_index] = Drontx__read[axis] - (Drontx__degree[axis]*10000000); 
        Drontx__filter_sum[axis] += Drontx__filter[axis][Drontx__filter_index];
        Drontx__filtered[axis] = Drontx__filter_sum[axis] / Drontx__FILTER_VEC_LENT + (Drontx__degree[axis]*10000000);
        if ( jrvis_state == jrvis_STATE_HOLD_INFINIT || jrvis_state == jrvis_STATE_HOLD_TIMED) {      
          if ( fraction3[axis]>1 && fraction3[axis]<999 ) Drontx__coord[axis] = Drontx__filtered[axis];
        }
      }
    }

    dTjrvis = (float)(millis() - jrvis_looptimer)/ 1000.0;
    jrvis_looptimer = millis();
    dTjrvis = min(dTjrvis, 1.0);  
    Drontx__land(&Drontx__coord[LAT],&Drontx__coord[LON],&Drontx__home[LAT],&Drontx__home[LON],&dir);
    Drontx__dist_cm(&Drontx__coord[LAT],&Drontx__coord[LON],&Drontx__home[LAT],&Drontx__home[LON],&dist);
    Drontx__distToHome = dist/100;
    Drontx__directionToHome = dir/100;
    if ((Drontx__conf.fence > 0) && (Drontx__conf.fence < Drontx__distToHome) && (f.Drontx__mode != Drontx__MODE_RTH) ) {
      init_RTH();
    }
    Drontx__calc_velocity();        
    if (f.Drontx__mode != Drontx__MODE_NONE) {  
      Drontx__land(&Drontx__coord[LAT],&Drontx__coord[LON],&Drontx__track[LAT],&Drontx__track[LON],&target_land);
      if (Drontx__conf.lead_filter) {
        Drontx__dist_cm(&Drontx__coord_lead[LAT],&Drontx__coord_lead[LON],&Drontx__track[LAT],&Drontx__track[LON],&track_dist);
        Drontx__calc_location_error(&Drontx__track[LAT],&Drontx__track[LON],&Drontx__coord_lead[LAT],&Drontx__coord_lead[LON]);
      } else {
        Drontx__dist_cm(&Drontx__coord[LAT],&Drontx__coord[LON],&Drontx__track[LAT],&Drontx__track[LON],&track_dist);
        Drontx__calc_location_error(&Drontx__track[LAT],&Drontx__track[LON],&Drontx__coord[LAT],&Drontx__coord[LON]);
      }
      // Adjust altitude 
      // if we are holding position and reached target altitude, then ignore altitude jrvis, and let the user trim alt
      if ( !((jrvis_state == jrvis_STATE_HOLD_INFINIT) && (alt_change_flag == REACHED_ALT))) {
        if (!f.LAND_IN_PROGRESS) {
          alt_to_hold = get_new_altitude();
          AltHold = alt_to_hold;
        }
      }

      int16_t speed = 0;                   //Desired jrvisigation speed

      switch(jrvis_state)                    //jrvisigation state machine
        {
        case jrvis_STATE_NONE:               //Just for clarity, do nothing when jrvis_state is none
          break;

        case jrvis_STATE_LAND_START:
          Drontx__calc_();              //Land in position hold
          land_settle_timer = millis();
          jrvis_state = jrvis_STATE_LAND_SETTLE;
          break;

        case jrvis_STATE_LAND_SETTLE:
          Drontx__calc_();
          if (millis()-land_settle_timer > 5000)
            jrvis_state = jrvis_STATE_LAND_START_DESCENT;
          break;

        case jrvis_STATE_LAND_START_DESCENT:
          Drontx__calc_();                //Land in position hold
          f.THROTTLE_IGNORED = 1;            //Ignore Throtte stick input
          f.Drontx__BARO_MODE    = 1;            //Take control of BARO mode
          land_detect = 0;                   //Reset land detector
          f.LAND_COMPLETED = 0;
          f.LAND_IN_PROGRESS = 1;            // Flag land process
          jrvis_state = jrvis_STATE_LAND_IN_PROGRESS;
          break;

        case jrvis_STATE_LAND_IN_PROGRESS:
          Drontx__calc_();
          check_land();  //Call land detector
          if (f.LAND_COMPLETED) {
            jrvis_timer_stop = millis() + 5000;
            jrvis_state = jrvis_STATE_LANDED;
          }
          break;

        case jrvis_STATE_LANDED:
          // Disarm if THROTTLE stick is at minimum or 5sec past after land detected
          if (rcData[THROTTLE]<MINCHECK || jrvis_timer_stop <= millis()) { //Throttle at minimum or 5sec passed.
            go_disarm();
            f.OK_TO_ARM = 0;                //Prevent rearming
            jrvis_state = jrvis_STATE_NONE;     //Disable position holding.... prevent flippover
            f.Drontx__BARO_MODE = 0;
            f.LAND_COMPLETED = 0;
            f.LAND_IN_PROGRESS = 0;
            land_detect = 0;
            f.THROTTLE_IGNORED = 0;
            Drontx__reset_jrvis();
          }
          break;

        case jrvis_STATE_HOLD_INFINIT:        //Constant position hold, no timer. Only an rcOption change can exit from this
          Drontx__calc_();
          break;

        case jrvis_STATE_HOLD_TIMED:
          if (jrvis_timer_stop == 0) {                         //We are start a timed 
            jrvis_timer_stop = millis() + 1000*jrvis_hold_time;  //Set when we will continue
          } else if (jrvis_timer_stop <= millis()) {           //did we reach our time limit ?
            if (DRONETIX.flag != DRNTIX_dronetix_track_FLAGGED) {
              jrvis_state = jrvis_STATE_PROCESS_NEXT;            //if yes then process next dronetix_track step
            }
            jrvis_error = jrvis_ERROR_TIMEWAIT;
          }
          Drontx__calc_();                                //comment this if u want subsiquent passing of comand
          break;

        case jrvis_STATE_RTH_START:
          if ((alt_change_flag == REACHED_ALT) || (!Drontx__conf.wait_for_rth_alt)) {             //Wait until we reach RTH altitude
            Drontx__set_next_track(&Drontx__home[LAT],&Drontx__home[LON], &Drontx__coord[LAT], &Drontx__coord[LON]); //If we reached then change mode and start RTH
            jrvis_state = jrvis_STATE_RTH_ENROUTE;
            jrvis_error = jrvis_ERROR_NONE;
          } else {
            Drontx__calc_();                                                               //hold position till we reach RTH alt
            jrvis_error = jrvis_ERROR_WAIT_FOR_RTH_ALT;
          }
          break;

        case jrvis_STATE_RTH_ENROUTE:                                                  //Doing RTH jrvisigation
          speed = Drontx__calc_desired_speed(Drontx__conf.jrvis_speed_max, Drontx__conf.slow_jrvis); 
          Drontx__calc_jrvis_rate(speed);
          Drontx__adjust_heading();
          if ((track_dist <= Drontx__conf.track_radius) || check_missed_track()) {            //if yes switch to customed mode
            if (DRONETIX.parameter1 == 0) jrvis_state = jrvis_STATE_HOLD_INFINIT;
            else jrvis_state = jrvis_STATE_LAND_START;                                  
            if (Drontx__conf.jrvis_rth_takeoff_heading) { magHold = jrvis_takeoff_land; }
          } 
          break;
// jarvis automated case shift: 
// experimented on iiit ground.. updates required for any other region shift
        case jrvis_STATE_track_ENROUTE:
          speed = Drontx__calc_desired_speed(Drontx__conf.jrvis_speed_max, Drontx__conf.slow_jrvis); 
          Drontx__calc_jrvis_rate(speed);
          Drontx__adjust_heading();
            // for automated flight.. set the flight path here:
          if ((track_dist <= Drontx__conf.track_radius) || check_missed_track()) {              
            if (DRONETIX.action == dronetix_track_LAND) {                                  //jarvis auti land starts
              jrvis_state = jrvis_STATE_LAND_START;                                         //Start landing
              set_new_altitude(alt.EstAlt);                                             //Stop any further altitude changes...react against the current
            } else if (DRONETIX.flag == DRNTIX_dronetix_track_FLAGGED) {                         
              jrvis_state = jrvis_STATE_HOLD_INFINIT;
              jrvis_error = jrvis_ERROR_FINISH;
            } else if (DRONETIX.action == dronetix_track_HOLD_UNLIM) {                     //If DRONETIX was _UNLIM and we reached the position then switch to  unlimited
              jrvis_state = jrvis_STATE_HOLD_INFINIT;
              jrvis_error = jrvis_ERROR_FINISH;
            } else if (DRONETIX.action == dronetix_track_HOLD_TIME) {                      
              jrvis_hold_time = DRONETIX.parameter1;
              jrvis_timer_stop = 0;                                                  //nt working for customed ui..      
              jrvis_state = jrvis_STATE_HOLD_TIMED;
            } else {
              jrvis_state = jrvis_STATE_PROCESS_NEXT;                                       
            }
          }
          break;

        case jrvis_STATE_DO_JUMP:
          if (jump_times < 0) {                                  //throw garbage values.. for 10 iterations
            next_step = DRONETIX.parameter1;
            jrvis_state = jrvis_STATE_PROCESS_NEXT;
          }
          if (jump_times == 0) {
            jump_times = -10;                                   
            if (DRONETIX.flag == DRNTIX_dronetix_track_FLAGGED) {         
              jrvis_state = jrvis_STATE_HOLD_INFINIT;
              jrvis_error = jrvis_ERROR_FINISH;
            } else
              jrvis_state = jrvis_STATE_PROCESS_NEXT;
          }

          if (jump_times > 0) {                                
            next_step = DRONETIX.parameter1;
            jrvis_state = jrvis_STATE_PROCESS_NEXT;
            jump_times--;
          }
          break;

        case jrvis_STATE_PROCESS_NEXT:                             //initiate next track
          jrvis_error = jrvis_ERROR_NONE;
          if (!recalltrack(next_step)) { 
            abort_dronetix_track(jrvis_ERROR_track_CRC);
          } else {
            switch(DRONETIX.action)
              {
              case dronetix_track_WAYPOINT:
              case dronetix_track_HOLD_TIME:
              case dronetix_track_HOLD_UNLIM:
              case dronetix_track_LAND:
                set_new_altitude(DRONETIX.altitude);
                Drontx__set_next_track(&DRONETIX.pos[LAT], &DRONETIX.pos[LON], &Drontx__prev[LAT], &Drontx__prev[LON]);  //fetch track information from the track ui file
                if ((track_dist/100) >= Drontx__conf.safe_track_dist)  abort_dronetix_track(jrvis_ERROR_TOOFAR);
                else jrvis_state = jrvis_STATE_track_ENROUTE;
                Drontx__prev[LAT] = DRONETIX.pos[LAT];  //Save track coordinates for precise route calc
                Drontx__prev[LON] = DRONETIX.pos[LON];
                break;
              case dronetix_track_RTH:
                f.Drontx__head_set = 0;
                if (Drontx__conf.rth_altitude == 0 && DRONETIX.altitude == 0) //very safe .. never override this .. has to be a master
                  set_new_altitude(alt.EstAlt);     // returns at thejarvis_real altitude 
                else {
                  uint32_t rth_alt;
                  if (DRONETIX.altitude == 0) rth_alt = Drontx__conf.rth_altitude * 100;   // priority given to set altititde nw
                  else rth_alt = DRONETIX.altitude;

                  if (alt.EstAlt < rth_alt) set_new_altitude(rth_alt);                     
                  else set_new_altitude(alt.EstAlt);
                }
                jrvis_state = jrvis_STATE_RTH_START;
                break;
              case dronetix_track_JUMP:
                if (jump_times == -10) jump_times = DRONETIX.parameter2;
                if (DRONETIX.parameter1 > 0 && DRONETIX.parameter1 < DRONETIX.number)
                  jrvis_state = jrvis_STATE_DO_JUMP;
                else //Error situation, invalid jump target
                  abort_dronetix_track(jrvis_ERROR_INVALID_JUMP);
                break;
              case dronetix_track_SET_HEADING:
                Drontx__poi[LAT] = 0; Drontx__poi[LON] = 0; 
                if (DRONETIX.parameter1 < 0) f.Drontx__head_set = 0;
                else {
                  f.Drontx__head_set = 1;
                  Drontx__directionToPoi = DRONETIX.parameter1;
                } 
                break;
              case dronetix_track_SET_POI:
                Drontx__poi[LAT] = DRONETIX.pos[LAT];
                Drontx__poi[LON] = DRONETIX.pos[LON];
                f.Drontx__head_set = 1;
                break;
              default:                                  // unknown action code abort track and hold position
                abort_dronetix_track(jrvis_ERROR_INVALID_DATA);
                break;
              }
            next_step++; //Prepare for the next step
          }
          break;
        } // switch end
    } //end of Drontx_ calcs  
  }
  return 1;
} // End of Drontx__compute
void abort_dronetix_track(unsigned char error_code) {
  Drontx__set_next_track(&Drontx__coord[LAT], &Drontx__coord[LON],&Drontx__coord[LAT], &Drontx__coord[LON]);
  jrvis_error = error_code;
  jrvis_state = jrvis_STATE_HOLD_INFINIT;
}
void Drontx__adjust_heading() {
  if (f.Drontx__head_set) { 
    if (Drontx__poi[LAT] == 0)
      magHold = wrap_18000((Drontx__directionToPoi*100))/100; //very unstable mag conversion..get sticks lowered ... experimented in patia
    else {
      Drontx__land(&Drontx__coord[LAT],&Drontx__coord[LON],&Drontx__poi[LAT],&Drontx__poi[LON],&Drontx__directionToPoi);
      Drontx__dist_cm(&Drontx__coord[LAT],&Drontx__coord[LON],&Drontx__poi[LAT],&Drontx__poi[LON],&track_dist);
      magHold = Drontx__directionToPoi /100;
    }
  } else {                                // heading controlled by the standard defines
    if (Drontx__conf.jrvis_controls_heading) {
      if (Drontx__conf.jrvis_tail_first) {
        magHold = wrap_18000(target_land-18000)/100;
      } else {
        magHold = wrap_18000(target_land)/100;
      }
    }
  }
}

#define LAND_DETECT_THRESHOLD 40      //Counts of land situation
#define BAROPIDMIN           -180     //BaroPID reach this if we landed.....

//Check if we landed or not
void check_land() {
  // detect whether we have landed by watching for low climb rate and throttle control
  if ( (abs(alt.vario) < 20) && (BaroPID < BAROPIDMIN)) {
    if (!f.LAND_COMPLETED) {
      if( land_detect < LAND_DETECT_THRESHOLD) {
        land_detect++;
      } else {
        f.LAND_COMPLETED = 1;
        land_detect = 0;
      }
    }
  } else {
    // we've detected movement up or down so reset land_detector
    land_detect = 0;
    if(f.LAND_COMPLETED) {
      f.LAND_COMPLETED = 0;
    }
  }
}

int32_t get_altitude_error() {
  return alt_to_hold - alt.EstAlt;
}

void clear_new_altitude() {
  alt_change_flag = REACHED_ALT;
}

void force_new_altitude(int32_t _new_alt) {
  alt_to_hold     = _new_alt;
  target_altitude = _new_alt;
  alt_change_flag = REACHED_ALT;
}

void set_new_altitude(int32_t _new_alt) {
  //Limit maximum altitude command
  if(_new_alt > Drontx__conf.jrvis_max_altitude*100) _new_alt = Drontx__conf.jrvis_max_altitude * 100;
  if(_new_alt == alt.EstAlt){
    force_new_altitude(_new_alt);
    return;
  }
  // We start at the current location altitude and gradually change alt
  alt_to_hold = alt.EstAlt;
  // for calculating the delta time
  alt_change_timer = millis();
  // save the target altitude
  target_altitude = _new_alt;
  // reset our altitude integrator
  alt_change = 0;
  // save the original altitude
  original_altitude = alt.EstAlt;
  // to decide if we have reached the target altitude
  if(target_altitude > original_altitude){
    // we are below, going up
    alt_change_flag = ASCENDING;
  } else if(target_altitude < original_altitude){
    // we are above, going down
    alt_change_flag = DESCENDING;
  } else {
    // No Change
    alt_change_flag = REACHED_ALT;
  }
}

int32_t get_new_altitude() {
  if(alt_change_flag == ASCENDING) {
    if(alt.EstAlt >=  target_altitude) alt_change_flag = REACHED_ALT; 
    if(alt_to_hold >=  target_altitude) return target_altitude; 
  } else if (alt_change_flag == DESCENDING) {
    if(alt.EstAlt <=  target_altitude) alt_change_flag = REACHED_ALT;
    if(alt_to_hold <=  target_altitude) return target_altitude;
  }
  if(alt_change_flag == REACHED_ALT) return target_altitude;

  int32_t diff  = abs(alt_to_hold - target_altitude);
  int8_t _scale = 4;

  if (alt_to_hold < target_altitude) {
    if(diff < 200) _scale = 4;
    else _scale = 3;
  } else {
    if(diff < 400) _scale = 5;  
    if(diff < 100) _scale = 6;  
  }
  int32_t change = (millis() - alt_change_timer) >> _scale;

  if(alt_change_flag == ASCENDING){
    alt_change += change;
  } else {
     alt_change -= change;
  }
  // for generating delta time
  alt_change_timer = millis();

  return original_altitude + alt_change;
}
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
void Drontx__calc_longitude_scaling(int32_t lat) {
  Drontx__scaleLonDown = cos(lat * 1.0e-7f * 0.01745329251f);
}
void Drontx__set_next_track(int32_t* lat_to, int32_t* lon_to, int32_t* lat_from, int32_t* lon_from) {
  Drontx__track[LAT] = *lat_to;
  Drontx__track[LON] = *lon_to;

  Drontx__FROM[LAT] = *lat_from;
  Drontx__FROM[LON] = *lon_from;

  Drontx__calc_longitude_scaling(*lat_to);

  Drontx__land(&Drontx__FROM[LAT], &Drontx__FROM[LON],&Drontx__track[LAT],&Drontx__track[LON],&target_land);
  Drontx__dist_cm(&Drontx__FROM[LAT],&Drontx__FROM[LON], &Drontx__track[LAT],&Drontx__track[LON], &track_dist);
  Drontx__calc_location_error(&Drontx__track[LAT],&Drontx__track[LON],&Drontx__FROM[LAT],&Drontx__FROM[LON]);
  waypoint_speed_gov = Drontx__conf.jrvis_speed_min;
  original_target_land = target_land;

}
static bool check_missed_track(void) {
  int32_t temp;
  temp = target_land - original_target_land;
  temp = wrap_18000(temp);
  return (abs(temp) > 10000);   
}

void Drontx__land(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, int32_t* land) {
  int32_t off_x = *lon2 - *lon1;
  int32_t off_y = (*lat2 - *lat1) / Drontx__scaleLonDown;

  *land = 9000 + atan2(-off_y, off_x) * 5729.57795f;     
  if (*land < 0) *land += 36000;
}

void Drontx__dist_cm(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist) {
  float dLat = (float)(*lat2 - *lat1);                                    
  float dLon = (float)(*lon2 - *lon1) * Drontx__scaleLonDown; //x
  *dist = sqrt(sq(dLat) + sq(dLon)) * 1.11318845f;
}

static void Drontx__calc_velocity(void){
  static int16_t speed_old[2] = {0,0};
  static int32_t last[2] = {0,0};
  static uint8_t init = 0;

  if (init) {
    float tmp = 1.0/dTjrvis;
   jarvis_real_speed[_X] = (float)(Drontx__coord[LON] - last[LON]) *  Drontx__scaleLonDown * tmp;
   jarvis_real_speed[_Y] = (float)(Drontx__coord[LAT]  - last[LAT])  * tmp;

    // Check for any unrealistic speed changes and signal registration about posibble signal degradation
    if (!Drontx__conf.lead_filter) {
     jarvis_real_speed[_X] = (actual_speed[_X] + speed_old[_X]) / 2;
     jarvis_real_speed[_Y] = (actual_speed[_Y] + speed_old[_Y]) / 2;

      speed_old[_X] =jarvis_real_speed[_X];
      speed_old[_Y] =jarvis_real_speed[_Y];
    }
  }
  init=1;

  last[LON] = Drontx__coord[LON];
  last[LAT] = Drontx__coord[LAT];

  if (Drontx__conf.lead_filter) {
    Drontx__coord_lead[LON] = xlead_filter_jarvis.get_position(Drontx__coord[LON],jarvis_real_speed[_X], Drontx__LAG);
    Drontx__coord_lead[LAT] = ylead_filter_jarvis.get_position(Drontx__coord[LAT],jarvis_real_speed[_Y], Drontx__LAG);
  }
}
// Drntic chart reference:
//   100  = 1m
//  1000  = 11m    
//  1800  = 19.80m 
static void Drontx__calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* Drontx__lat, int32_t* Drontx__lng ) {
  error[LON] = (float)(*target_lng - *Drontx__lng) * Drontx__scaleLonDown;  // X Error
  error[LAT] = *target_lat - *Drontx__lat; // Y Error
}
static void Drontx__calc_(void) {
  int32_t d;
  int32_t target_speed;
  uint8_t axis;
  
  for (axis=0;axis<2;axis++) {
    target_speed = get_P(error[axis], &DRNTIX_); // calculate desired speed from lat/lon error
    target_speed = constrain(target_speed,-100,100);      
    rate_error[axis] = target_speed -jarvis_real_speed[axis]; 

    jrvis[axis]      =
        get_P(rate_error[axis],                                               &_rateDRNTIX_)
       +get_I(rate_error[axis] + error[axis], &dTjrvis, &_ratePID[axis], &_rateDRNTIX_);

    d = get_D(error[axis],                    &dTjrvis, &_ratePID[axis], &_rateDRNTIX_);
    d = constrain(d, -2000, 2000);
    if(abs(actual_speed[axis]) < 50) d = 0;

    jrvis[axis] +=d;
    jrvis[axis]  = constrain_int16(jrvis[axis], -Drontx__conf.jrvis_bank_max, Drontx__conf.jrvis_bank_max);
    jrvisPID[axis].integrator = _ratePID[axis].integrator;
  }
}


static void Drontx__calc_jrvis_rate( uint16_t max_speed) {
  float trig[2];
  int32_t target_speed[2];
  int32_t tilt;
  uint8_t axis;

  Drontx__updte_crss_freq();
  int16_t cross_speed = crss_freq_error * (Drontx__conf.crss_freq_gain / 100.0);  //check is it ok ?
  cross_speed = constrain(cross_speed,-200,200);
  cross_speed = -cross_speed;

  float temp = (9000l - target_land) * RADX100;
  trig[_X] = cos(temp);
  trig[_Y] = sin(temp);

  target_speed[_X] = max_speed * trig[_X] - cross_speed * trig[_Y];
  target_speed[_Y] = cross_speed * trig[_X] + max_speed * trig[_Y];

  for (axis=0;axis<2;axis++) {
    rate_error[axis] = target_speed[axis] -jarvis_real_speed[axis];
    rate_error[axis] = constrain(rate_error[axis],-1000,1000);
    jrvis[axis]      =
        get_P(rate_error[axis],                        &jrvisDRNTIX_)
       +get_I(rate_error[axis], &dTjrvis, &jrvisPID[axis], &jrvisDRNTIX_)
       +get_D(rate_error[axis], &dTjrvis, &jrvisPID[axis], &jrvisDRNTIX_);
    jrvis[axis]  = constrain_int16(jrvis[axis], Drontx__conf.jrvis_bank_max, Drontx__conf.jrvis_bank_max);
    _ratePID[axis].integrator = jrvisPID[axis].integrator;
  }
}

