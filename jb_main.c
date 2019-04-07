/**
* JerboBot Odometry
* Taylor Sun (taysun@umich.edu)
* Last updated: 3/27/19
*
* JerboBot control code, adapted from "rc_balance"
* solution in Robot Control Library.
*/

#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strof()
#include <math.h> // for M_PI
#include <robotcontrol.h>


#include "odom.h"

/**
 * ARMED or DISARMED to indicate if the controller is running
 */
typedef enum arm_state_t {
	ARMED,
	DISARMED
}arm_state_t;

/**
* Feedback controller setpoint written to by set_point manager and read by the 
* controller.
*/
typedef struct setpoint_t {
	arm_state_t arm_state;	///< see arm_state_t declaration
	double x;	///< side-to-side position (m), global coords
	double x_dot; 
	double y;	///< front-and-back position (m), global coords
	double y_dot;
	double z;	///< up-and-down, telescoping arm position (m), global coords
	double z_dot; 
	double theta; ///< body yaw angle (rad)
}setpoint_t;

/**
* This is the system state written by the position controller.
*/
typedef struct core_state_t {
	double wheelAngle1;	///< wheel rotation relative to body @ start
	double wheelAngle2; 
	double wheelAngle3; 
	double wheelAngle4; 
	double wheelAngle5; ///< "wheel" rotation for telescoping arm

	double x;
	double y;
	double z; 
	double theta; ///< error in angle of omni-wheel axes
	double vBatt; ///< battery voltage 

	double d1_u; ///< output of position controller D1, to motor 1
	double d2_u; ///< output of position controller D2, to motor 2
	double d3_u; ///< output of position controller D3, to motor 3
	double d4_u; ///< output of position controller D4, to motor 4
	double d5_u; ///< output of rotation controller, D5, to x-y motors
	double d6_u; ///< output of position controller, D6, to motor 5

	double mot_drive; ///< u compensated for battery voltage
} core_state_t;

// operation modes, user selected with command line arguments
typedef enum m_input_mode_t {
	NONE,
	MANU, ///< control with wireless controller
	AUTO  ///< control with saved trajec file
} m_input_mode_t;

static void __print_usage(void);
static void __position_controller(void);	///< mpu interrupt routine
static void* __setpoint_manager(void* ptr);	///< background thread
static void* __battery_checker(void* ptr);	///< background thread
static void* __printf_loop(void* ptr);		///< background thread
static int __zero_out_controller(void);
static int __disarm_controller(void);
static int __arm_controller(void);
static int __wait_for_starting_condition(void);
static void __on_pause_press(void);
static void __on_mode_release(void);

// global variables
// must be declared static so they get zero initialized
static core_state_t cstate;
static setpoint_t setpoint;
static rc_filter_t D1 = RC_FILTER_INITIALIZER;
static rc_filter_t D2 = RC_FILTER_INITIALIZER;
static rc_filter_t D3 = RC_FILTER_INITIALIZER;
static rc_mpu_data_t mpu_data;
static m_input_mode_t m_input_mode = MANU; 

/*
 * Printed if some invalid argument was given
 */
static void __print_usage(void)
{
	printf("\n");
	printf("-i {manu|auto|none}     specify input\n");
	printf("-q                      Don't print diagnostic info\n");
	printf("-h                      print this help message\n");
	printf("\n");
}

/**
* Initialize the filters, mpu, threads, & wait until shut down
* 
* @return		0 on success, -1 on failure
*/
int main(int argc, char *argv[]) 
{
	int c; 
	pthread_t setpoint_thread = 0; 
	pthread_t battery_thread = 0; 
	pthread_t printf_thread = 0;
	bool adc_ok = true; 
	bool quiet = false; 

	// parse arguments
	opterr = 0; 
	while ((c = getopt(argc, argv, "i:qh")) != -1) {
		switch (c) {
		case 'i': // input option
			if (!strcmp("manu", optarg)) {
				m_input_mode = MANU;
			}
			else if (!strcmp("auto", optarg)) {
				m_input_mode = AUTO;
			}
			else if (!strcmp("none", optarg)) {
				m_input_mode = NONE;
			}
			else {
				__print_usage();
				return -1;
			}
			break;
		case 'q':
			quiet = true;
			break;
		case 'h':
			__print_usage();
			return -1;
			break;
		default:
			__print_usage();
			return -1;
			break;
		}
	}

	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privileges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if (rc_kill_existing_process(2.0) < -2) return -1;

	// start signal handler so we can exit cleanly
	if (rc_enable_signal_handler() == -1) {
		fprintf(stderr, "ERROR: failed to start signal handler\n");
		return -1;
	}

	// initialize buttons
	if (rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
		RC_BTN_DEBOUNCE_DEFAULT_US)) {
		fprintf(stderr, "ERROR: failed to initialize pause button\n");
		return -1;
	}
	if (rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
		RC_BTN_DEBOUNCE_DEFAULT_US)) {
		fprintf(stderr, "ERROR: failed to initialize mode button\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE, __on_pause_press, NULL);
	rc_button_set_callbacks(RC_BTN_PIN_MODE, NULL, __on_mode_release);

	//initialize encoders
	if (rc_encoder_init() == -1) {
		fprintf(stderr, "ERROR: failed to initialize encoders\n");
		return -1;
	}

	//initialize motors
	if (rc_motor_init() == -1) {
		fprintf(stderr, "ERROR: failed to initialize motors\n");
		return -1;
	}
	rc_motor_standby(1); // start with motors in standby

	// start wireless controller listener
	// SERVO LIBRARY?

	// initialize adc
	if (rc_adc_init() == -1) {
		fprintf(stderr, "failed to initialize adc\n");
		adc_ok = false; 
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	printf("\nPress and release MODE button to toggle MANU drive mode\n");
	printf("Press and release PAUSE button to pause/start the motors\n");
	printf("hold pause button down for 2 seconds to exit\n");
	printf("press e-stop at any time to exit\n");

	if (rc_led_set(RC_LED_GREEN, 0) == -1) {
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_GREEN\n");
		return -1;
	}
	if (rc_led_set(RC_LED_RED, 1) == -1) {
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_RED\n");
		return -1;
	}

	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Y_UP;

	// if gyro isn't calibrated, run the calibration routine
	if (!rc_mpu_is_gyro_calibrated()) {
		printf("Gyro not calibrated, automatically starting calibration routine\n");
		printf("Let your MiP sit still on a firm surface\n");
		rc_mpu_calibrate_gyro_routine(mpu_config);
	}

	// make sure setpoint starts at normal value
	setpoint.arm_state = DISARMED;

	// set up D1 position controller 
	// INSERT ALL CONTROLLER INIT HERE








	// start a thread to slowly sample battery
	if (adc_ok) {
		if (rc_pthread_create(&battery_thread, __battery_checker, (void*)NULL, SCHED_OTHER, 0)) {
			fprintf(stderr, "failed to start battery thread\n");
			return -1;
		}
	}
	else { // if we can't get the battery voltage
		cstate.vBatt = V_NOMINAL; // set to a nominal value
	}

	// wait for the battery thread to make the first read
	while (cstate.vBatt < 1.0 && rc_get_state() != EXITING) rc_usleep(10000);

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if (isatty(fileno(stdout)) && (quiet == false)) {
		if (rc_pthread_create(&printf_thread, __printf_loop, (void*)NULL, SCHED_OTHER, 0)) {
			fprintf(stderr, "failed to start battery thread\n");
			return -1;
		}
	}

	// start mpu
	if (rc_mpu_initialize_dmp(&mpu_data, mpu_config)) {
		fprintf(stderr, "ERROR: can't talk to IMU, all hope is lost\n");
		rc_led_blink(RC_LED_RED, 5, 5);
		return -1;
	}

	// start control stack to control setpoints
	if (rc_pthread_create(&setpoint_thread, __setpoint_manager, (void*)NULL, SCHED_OTHER, 0)) {
		fprintf(stderr, "failed to start battery thread\n");
		return -1;
	}

	// this should be the last step in initialization
	// to make sure other setup functions don't interfere
	//rc_mpu_set_dmp_callback(&__balance_controller);



	// NOMINAL OP AND CLEANUP BELOW


	// start in the RUNNING state, pressing the pause button will swap to
	// the PAUSED state then back again.
	printf("\nHold your MIP upright to begin balancing\n");
	rc_set_state(RUNNING);

	// chill until something exits the program
	rc_set_state(RUNNING);
	while (rc_get_state() != EXITING) {
		rc_usleep(200000);
	}

	// join threads
	rc_pthread_timed_join(setpoint_thread, NULL, 1.5);
	if (battery_thread) rc_pthread_timed_join(battery_thread, NULL, 1.5);
	if (printf_thread) rc_pthread_timed_join(printf_thread, NULL, 1.5);

	// cleanup
	rc_filter_free(&D1);
	rc_filter_free(&D2);
	rc_filter_free(&D3); // OTHER CONTROLLERS? ?????????

	rc_mpu_power_off();
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_encoder_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}


/**
* discrete-time position controller for in-plane motion, called at 
* SAMPLE_RATE_HZ
*/
static void __position_controller(void)
{
	static int inner_saturation_counter = 0; 
	double duty1, duty2, duty3, duty4, duty5; 

	/******************************************************************
	* STATE_ESTIMATION
	* read sensors and compute the state when either ARMED or DISARMED
	******************************************************************/
	double wheel1_old = cstate.wheelAngle1;
	double wheel4_old = cstate.wheelAngle4;
	double wheel2_old = cstate.wheelAngle2;
	double wheel3_old = cstate.wheelAngle3;
	double wheel5_old = cstate.wheelAngle5;

	cstate.wheelAngle1 = (rc_encoder_read(ENCODER_CHANNEL_1) * 2.0 * M_PI) \
		/ (ENCODER_POLARITY_1 * GEARBOX_XY * ENCODER_RES);
	cstate.wheelAngle2 = (rc_encoder_read(ENCODER_CHANNEL_2) * 2.0 * M_PI) \
		/ (ENCODER_POLARITY_2 * GEARBOX_XY * ENCODER_RES);
	cstate.wheelAngle3 = (rc_encoder_read(ENCODER_CHANNEL_3) * 2.0 * M_PI) \
		/ (ENCODER_POLARITY_3 * GEARBOX_XY * ENCODER_RES);
	cstate.wheelAngle4 = (rc_encoder_read(ENCODER_CHANNEL_4) * 2.0 * M_PI) \
		/ (ENCODER_POLARITY_4 * GEARBOX_XY * ENCODER_RES);
	cstate.wheelAngle5 = (rc_encoder_read(ENCODER_CHANNEL_5) * 2.0 * M_PI) \
		/ (ENCODER_POLARITY_5 * GEARBOX_Z * ENCODER_RES); 

	// find change in encoder position
	double dAngle1 = cstate.wheelAngle1 - wheel1_old;
	double dAngle4 = cstate.wheelAngle4 - wheel4_old;
	double dAngle2 = cstate.wheelAngle2 - wheel2_old;
	double dAngle3 = cstate.wheelAngle3 - wheel3_old;
	double dYaw = cstate.wheelAngle5 - wheel5_old;

	// change in position along resultant omni axes
	double dX_r = 0.5 * WHEEL_RADIUS_XY * (dAngle1 + dAngle4);
	double dY_r = 0.5 * WHEEL_RADIUS_XY * (dAngle2 + dAngle3);
	// rotation in omni axes due to differential drive
	cstate.theta += (1 / TRACK_WIDTH) *
		(dAngle4 - dAngle1 + dAngle2 - dAngle3);

	// convert to change in global coords
	cstate.x += dX_r * cos(ANGLE_GLOBAL2OMNI + cstate.theta) 
		+ dY_r * sin(ANGLE_GLOBAL2OMNI + cstate.theta);
	cstate.y += dX_r * sin(ANGLE_GLOBAL2OMNI + cstate.theta)
		+ dY_r * cos(ANGLE_GLOBAL2OMNI + cstate.theta);
		
	// correct for full rotation
	if (cstate.theta >= 2 * M_PI) {
		cstate.theta = cstate.theta - 2 * M_PI;
	}
	else if (cstate.theta <= -2 * M_PI) {
		cstate.theta = cstate.theta + 2 * M_PI;
	}

	cstate.z += cstate.wheelAngle5 * WHEEL_RADIUS_Z;

	/*************************************************************
	* check for various exit conditions AFTER state estimate
	***************************************************************/





	/************************************************************
	* OUTER LOOP X-Y controller D1
	* Move the position setpoint based on phi_dot.
	* Input to the controller is phi error (setpoint-state).
	*************************************************************/



	/************************************************************
	* INNER LOOP X controller D2
	* Input to D1 is theta error (setpoint-state). Then scale the
	* output u to compensate for changing battery voltage.
	*************************************************************/


}


static int __convert_position(void)
{
	// return direction of 1
	int a = (cstate.wheelAngle1 > 0) - (cstate.wheelAngle1 < 0);
	int b = (cstate.wheelAngle2 > 0) - (cstate.wheelAngle2 < 0);




}



// REQ: valid trajectory input
// MOD:	command motors in simple strategy
// EFFECT:	operate in simple strategy, ramp up & down
void mode_simple(rc_vector_trajec *coord_omni) {

}

// REQ: valid trajectory input
// MOD:	command motors in PID strategy
// EFFECT:	operate in PID strategy
void mode_pid() { }

// prior to continuing with movement, check if have rotated 
void rot_correct() { }
