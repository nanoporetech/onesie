#ifndef THERMAL_INTERFACE_H
#define THERMAL_INTERFACE_H

// Definitions
#define LOG_LEN 240 //Maximum 6400 for test proj and 240 for real system

//Defines for control word
#define CTRL_EN_MASK 1
#define CTRL_TEC_OVERRIDE_MASK 2

//Defines for error word
//Thermistors are used on P1.  I2C digital sensors will be fitted on P2.
#define FC_THERM_OPEN 1
#define FC_THERM_SHORT 2
#define FC_THERM_RANGE 4
#define HSINK_THERM_OPEN 8
#define HSINK_THERM_SHORT 0x10
#define HSINK_THERM_RANGE 0x12

//Defines for res_conv error
#define THERM_OPEN -1
#define THERM_SHORT -2
#define OUT_OF_RANGE -3

//Number of PID profiles
#define NUM_PROFILES 4

//Data structures
struct data_log_point
{
    u16 tec_value;  //Raw value driven to TEC driver
    u16 fc_temp;    //Calculated process side temperature - -16C offset, 128C range
    u16 hsink_temp; //Calculated heatsink temperature
    s16 err_prop;   //Error signal used in control loop
};

/*NB 3 parameters make up integral gain:
 * gain = ki_gain * ni_len / 2 ^ ki_shift
 * e.g. for ki_gain = 2, ni_len = 128, ki_shift = 7:
 * gain = 2 * 128 / 2^7 = 2
 */
struct pid_profile_struct
{
    u16	kp_gain;            //Proportional gain
    u16	ki_gain;            //Integral gain
    u16	kd_gain;            //Differential gain
    u16 ki_shift;           // Right shift i.e. gain divisor of 2^ki_shift
    u16	ni_len;             //Length of integrator
    u16 post_scale;         //Right shift of combined gain value ( i.e. divide by 2^post_scale)
    u16	sample_t;           //Sampling period in ms - min 33 for LTC2460
    u16	fc_therm_weight;    //Weighting of thermistor input to temperature estimate
    u16	ch514_weight;       //Weighting of ASIC channel input to temperature estimate
};

struct message_struct
{
    u16 control_word;   //See defines for bit definitions
    u16 error_word;     //See defines for bit definitions
    u16 set_point;
    u16 tec_override;   //Raw value to write to TEC driver
    u16 tec_dead_zone;  //All TECs have a dead zone.  The control loop will jump over the dead zone to prevent oscillation.
    u16 tec_v;          //Measured voltage across TEC
    u16 tec_i;          //Measured current into TEC
    u16	tec_ref;        //Measured 2.5V TEC reference voltage
    u16 tec_sp;         //Measured TEC control voltage
    struct data_log_point data_log[LOG_LEN];    //circular buffer of entries
    u16 profile_thresh[(NUM_PROFILES - 1)];
    struct pid_profile_struct pid_profile[NUM_PROFILES];
    u16 data_log_pointer;   //Index into data log
};


#endif
