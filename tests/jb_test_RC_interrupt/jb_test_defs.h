/**
 * jb_main_defs.h
 *
 * Contains the settings for configuration of jb_main.c
 */

#ifndef RC_BALANCE_CONFIG
#define RC_BALANCE_CONFIG



 // Structural properties of JerboBot
#define GEARBOX_XY			26.851
#define GEARBOX_Z		5.182
#define ENCODER_RES		48
#define WHEEL_RADIUS_XY		0.0762 // omni-wheel radius (m), 3 inch
#define WHEEL_RADIUS_Z		0.02477 // silicone roller radius (m)
#define TRACK_WIDTH		0.52 // update to distance between omni wheels
#define V_NOMINAL		11.1
#define ANGLE_GLOBAL2OMNI M_PI/4
#define ACCEL_MAX			30 	// m/s2 acceleration, rad/s2 for this test
							// ^^^ was 10 for earlier, testing limits


// inner test loop controller, 100 hz?
#define D1_KP				10
#define D1_KI				0
#define D1_KD				0
#define D1_GAIN				0.5
#define D1_SATURATION_TIMEOUT	0.4
#define D4_KP				10
#define D4_KI				0
#define D4_KD				0
#define D4_GAIN				0.5
#define D2_KP				10
#define D2_KI				0
#define D2_KD				0
#define D2_GAIN				0.5
#define D3_KP				10
#define D3_KI				0
#define D3_KD				0
#define D3_GAIN				0.5


// electrical hookups
// (recall motors1&4 = x_r, 2&3 = y_r)
#define MOTOR_CHANNEL_1		1
#define MOTOR_CHANNEL_2		2
#define MOTOR_CHANNEL_3		3
#define MOTOR_CHANNEL_4		4
#define MOTOR_CHANNEL_5		5
#define MOTOR_POLARITY_1	1
#define MOTOR_POLARITY_2	1
#define MOTOR_POLARITY_3	-1
#define MOTOR_POLARITY_4	-1
#define MOTOR_POLARITY_5	-1
#define ENCODER_CHANNEL_1	1
#define ENCODER_CHANNEL_2	3
#define ENCODER_CHANNEL_3	4
#define ENCODER_CHANNEL_4	2
//#define ENCODER_CHANNEL_5
#define ENCODER_POLARITY_1	-1
#define ENCODER_POLARITY_2	-1
#define ENCODER_POLARITY_3	1
#define ENCODER_POLARITY_4	1
//#define ENCODER_POLARITY_5	-1

//	drive speeds when using remote control (dsm2)
#define DRIVE_RATE_NOVICE	16
#define TURN_RATE_NOVICE	6
#define DRIVE_RATE_ADVANCED	26
#define TURN_RATE_ADVANCED	10

// DSM channel config
#define DSM_DRIVE_POL		1
#define DSM_TURN_POL		1
#define DSM_DRIVE_CH		3
#define DSM_TURN_CH		2
#define DSM_DEAD_ZONE		0.04

// Thread Loop Rates
#define BATTERY_CHECK_HZ	5
#define SETPOINT_MANAGER_HZ	200
#define PRINTF_HZ		50
#define SAMPLE_RATE_HZ		200
#define DT					0.005
#define RC_READER_HZ	20

// other
#define TIP_ANGLE		0.85
#define START_ANGLE		0.3
#define START_DELAY		0.4
#define PICKUP_DETECTION_TIME	0.6
#define ENABLE_POSITION_HOLD	1
#define SOFT_START_SEC		0.2

#endif	// endif RC_BALANCE_CONFIG
