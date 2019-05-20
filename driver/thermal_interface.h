#ifndef THERMAL_INTERFACE_H
#define THERMAL_INTERFACE_H

#define TEST_LEN 140
#define PID_PROFILES 4

//Data structures
struct data_log_point
{
        u16	tec_value;	//Raw value driven to TEC driver
        u16 fc_temp;		//Calculated process side temperature - -16°C offset, 128°C range
        u16 hsink_temp;	//Calculated heatsink temperature
} __attribute__(( packed ));

struct pid_profile_struct
{
        u32	kp_gain;	//Proportional gain
        u32	ki_gain;	//Integral gain
        u32	kd_gain;	//Differential gain
        u16	ni_len;		//Length of integrator
        u32	sample_t;	//Sampling period in us - min 33334 for LTC2460
        u16	fc_therm_weight;	//Weighting of thermistor input to temperature estimate
        u16	ch514_weight;		//Weighting of ASIC channel input to temperature esitimate
} __attribute__(( packed ));

struct message_struct
{
        u16 control_word;
        u16 error_word;
        u16 tec_override;
        u16 tec_dead_zone;		//All TECs have a dead zone.  The control loop will jump over the dead zone to prevent oscillation.
        u16 tec_v;	//Measured voltage across TEC
        u16 tec_i;	//Measured current into TEC
        struct data_log_point data_log[TEST_LEN];
        u16 profile_thresh_1;
        u16 profile_thresh_2;
        u16 profile_thresh_3;
        struct pid_profile_struct pid_profile[PID_PROFILES];
} __attribute__(( packed ));

#endif
