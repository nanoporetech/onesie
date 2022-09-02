/*
 * minion1c_therm.h
 *
 * Shared memory structures (and related definitions).
 *
 * Adapted from
 * https://git.oxfordnanolabs.local/minit/minion1c_fpga/-/blob/main/software/minion1c_therm_p2/minion1c_therm.h
 *
 */

#ifndef THERMAL_INTERFACE_H
#define THERMAL_INTERFACE_H

#define LOG_LEN 240 //Maximum 6400 for test proj and 240 for real system

//Number of PID profiles
#define NUM_PROFILES 2 /* second is RFU*/

//Data structures
struct data_log_point
{
    u16 tec_value;  //Raw value driven to TEC driver
    /* Temperature values are unsigned as the sensor does not measure below 0 = -45C*/
    u16 fc_temp;    //Calculated process side temperature
    u16 hsink_temp; //Calculated heatsink temperature
    /* NB err_prop is signed as the error can be positive or negative*/
    s16 err_prop;   //Error signal used in control loop
} __attribute__((packed));

/*NB 3 parameters make up integral gain:
 * gain = ki_gain * ni_len / 2 ^ ki_shift
 * e.g. for ki_gain = 2, ni_len = 128, ki_shift = 7:
 * gain = 2 * 128 / 2^7 = 2
 */
struct pid_profile_struct
{
    u16 kp_gain;        //Proportional gain
    u16 ki_gain;        //Integral gain
    u16 kd_gain;        //Differential gain
    u16 ki_shift;       // Right shift i.e. gain divisor of 2^ki_shift
    u16 diff_dist;      //  Differentiator distance in time samples
    u16 post_scale;     //Right shift of combined gain value ( i.e. divide by 2^post_scale)
    u16 sample_t;       //Sampling period in ms - min 33 for LTC2460
    u16 front_avg;      // Number (2^n) of samples that front end averaging uses - no loss of accuracy for 4 and below
    u16 dt_corr_shift;  //right shift to dT_TEC before being added to dt_corr_shift
    s16 dt_corr_const;  //constant applied to TEC voltage output for 0 dT_TEC
    u16 wind_band;      //The integrator is frozen when the error is outside of this band
    u16 wind_cap;       //The integrator is capped to this DAC value
    u16 sp_sig_change;  // Definition of a significant set point change - this affects the treatment of the integrator
    u16 intg_clear;     //Integrator clear: bit 0 = clear on significant sp change.
                        //bit 1 = clear on excursion from band
                        //bit 2 = user requested clear (this is not self-clearing)
} __attribute__((packed));

struct message_struct
{
    u16 control_word;   //See defines (minion_ioctl.h) for bit definitions
    u16 error_word;     //See defines (minion_ioctl.h) for bit definitions
    u16 set_point;
    u16 tec_override;   //Raw value to write to TEC driver
    u16 error_code;     //Error code from function causing stop
    /* Vtec(mV) = (tec_v / 2) - 5000 */
    u16 tec_v;          //Measured voltage across TEC
    /* Itec(mA) = (tec_i / 2.28) - 4386 */
    u16 tec_i;          //Measured current into TEC
    u16 tec_ref;        //Measured 2.5V TEC reference voltage
    u16 tec_sp;         //Measured TEC control voltage
    struct data_log_point data_log[LOG_LEN]; //circular buffer of entries
    u8 packing_1[6];    /* 6 unused bytes */
    struct pid_profile_struct pid_profile[NUM_PROFILES];
    u8 profile_packing[8];
    u16 ext_sens1_reading;
    u16 ext_sens2_reading;
    u16 ext_sens_control;
    u16 ext_sens_status;
    u16 data_log_pointer; //Index into data log
    /* Not including debug values */
} __attribute__((packed));


#endif
