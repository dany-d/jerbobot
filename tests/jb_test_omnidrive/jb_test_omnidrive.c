/**
* JerboBot OmniDrive Test
* Taylor Sun (taysun@umich.edu)
* Last updated: 4/13/19
*
* Test PID tracking of trapezoidal velocity
* profile for full x-y trajectory
*/

#include <stdio.h>
#include <robotcontrol.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#include "jb_test_defs.h"



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
	double wheelAngle1; // TEMP, for double motor test ONLY
	double wheelAngle4; 
	double wheelAngle2;
	double wheelAngle3;
	double x;	///< side-to-side position (m), global coords
	double y;	///< front-and-back position (m), global coords
	double z;	///< up-and-down, telescoping arm position (m), global coords
	double theta; ///< body yaw angle (rad)
}setpoint_t;

/** 
* This is the system state written to by the position controller. 
*/
typedef struct core_state_t {
	double wheelAngle1;	///< wheel rotation relative to body 
	double wheelAngle2;
	double wheelAngle3;
	double wheelAngle4;
	double wheelAngle5; ///< "wheel" rotation for telescoping arm
	double d1_u;		/// < output of test motor controller D1
	double d4_u; 
	double d2_u;
	double d3_u;
	double vBatt;		///< battery voltage
	double x;			///< global coordinates, x
	double y;
	double x_r;			///< 45d rotated, omni coordinates, x
	double y_r;
	double z;
	double theta;		///< error in angle of omni axis relative to global
	int step;			///< step (row) in trajec mat to pursue
	double t_1;			///< initial time (end of previous) in trajectory
	double t_2;			///< next time to reach trajectory pt
	uint64_t t_curr;	///< current time in ms
	double v_xr_des;	///< desired x_r velocity, to be updated by trajec
	double v_yr_des;	///< desired x_r velocity, to be updated by trajec
} core_state_t;

static void __print_usage(void);
static void __position_controller(void);	///< mpu interrupt routine
static void __setpoint_manager(void);
static void __traject_new(void);
static void* __print_loop(void* ptr);		///< background thread
static void* __battery_checker(void* ptr);	///< background thread
static int __zero_out_controller(void);
static int __disarm_controller(void);
static int __arm_controller(void);

// global variables
static core_state_t cstate;
static setpoint_t setpoint; 
static rc_filter_t D1 = RC_FILTER_INITIALIZER;
static rc_filter_t D4 = RC_FILTER_INITIALIZER;
static rc_filter_t D2 = RC_FILTER_INITIALIZER;
static rc_filter_t D3 = RC_FILTER_INITIALIZER;
static rc_mpu_data_t mpu_data;
static FILE* fout = NULL;
static uint64_t test_start; // record start time of trial
static rc_matrix_t trajec_mat = RC_MATRIX_INITIALIZER;

/*
 * Printed if some invalid argument was given
 */
static void __print_usage(void)
{
	printf("\n");
	printf("-f {filename}     print results to filename\n");
	printf("-s                print results to terminal\n");
	printf("-h                print this help message\n");
	printf("\n");
}

int main(int argc, char *argv[]) {
	int c;
	pthread_t printf_thread = 0;
	pthread_t battery_thread = 0;
	bool adc_ok = true; 

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, ":f:sh")) != -1) {
		switch (c) {
		case 'f':  // print to file
			fout = fopen(optarg, "w");
			// CHECK IF WRITING TO A WRITE PROTECTED FILE
			break;
		case 's':
			break;
		case 'h':
			__print_usage();
			return -1;
			break;
		case ':':
			__print_usage();
			return -1;
			break;
		default:
			__print_usage();
			return -1;
			break;
		}
	}

	if (rc_kill_existing_process(2.0) < -2) return -1;

	// start signal handler, so can exit cleanly
	if (rc_enable_signal_handler() == -1) {
		fprintf(stderr, "ERROR: failed to start signal handler\n");
		return -1;
	}

	// initialize buttons
	/*
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
	*/

	// initialize encoders
	if (rc_encoder_init() == -1) {
		fprintf(stderr, "ERROR: failed to initialize encoders\n");
		return -1;
	}

	// initialize motors
	if (rc_motor_init() == -1) {
		fprintf(stderr, "ERROR: failed to initialize motors\n");
		return -1;
	}
	rc_motor_standby(1); // start with motors in standby

	//rc_usleep(1000000); // wait 1 second before starting

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
	mpu_config.orient = ORIENTATION_Z_UP;

	// if gyro isn't calibrated, run the calibration routine
	if (!rc_mpu_is_gyro_calibrated()) {
		printf("Gyro not calibrated, automatically starting calibration routine\n");
		printf("Let your MiP sit still on a firm surface\n");
		rc_mpu_calibrate_gyro_routine(mpu_config);
	}

	// make sure setpoint starts at normal values
	setpoint.arm_state = DISARMED;

	// initialize all control loops
	// D1 control loop for motor1
	if (rc_filter_pid(&D1, D1_KP, D1_KI, D1_KD, 4 * DT, DT)) {
		fprintf(stderr, "ERROR in jb_main, failed to make filter D1\n");
		return -1;
	}
	if (rc_filter_pid(&D4, D4_KP, D4_KI, D4_KD, 4 * DT, DT)) {
		fprintf(stderr, "ERROR in jb_main, failed to make filter D4\n");
		return -1;
	}
	if (rc_filter_pid(&D2, D2_KP, D2_KI, D2_KD, 4 * DT, DT)) {
		fprintf(stderr, "ERROR in jb_main, failed to make filter D2\n");
		return -1;
	}
	if (rc_filter_pid(&D3, D3_KP, D3_KI, D3_KD, 4 * DT, DT)) {
		fprintf(stderr, "ERROR in jb_main, failed to make filter D3\n");
		return -1;
	}

	printf("Motor1 controller D1:\n");
	rc_filter_print(D1);
	printf("Motor2 controller D4:\n");
	rc_filter_print(D4);
	printf("Motor2 controller D2:\n");
	rc_filter_print(D2);
	printf("Motor2 controller D3:\n");
	rc_filter_print(D3);

	// start a thread to slowly sample battery
	if (adc_ok) {
		if (rc_pthread_create(&battery_thread, __battery_checker, (void*)NULL, SCHED_OTHER, 0)) {
			fprintf(stderr, "failed to start battery thread\n");
			return -1;
		}
	}
	else { // If we can't get the battery voltage
		cstate.vBatt = V_NOMINAL; // Set to a nominal value
	}

	// wait for the battery thread to make the first read
	while (cstate.vBatt < 1.0 && rc_get_state() != EXITING) rc_usleep(10000);

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if (isatty(fileno(stdout))) {
		if (rc_pthread_create(&printf_thread, __print_loop, (void*)NULL, SCHED_OTHER, 0)) {
			fprintf(stderr, "failed to start battery thread\n");
			return -1;
		}
	}

	// start mpu
	if (rc_mpu_initialize_dmp(&mpu_data, mpu_config)) {
		fprintf(stderr, "ERROR: can't talk to IMU\n");
		rc_led_blink(RC_LED_RED, 5, 5);
		return -1;
	}

	// allocate memory then save trajectories to matrix	
	trajec_mat = rc_matrix_empty();
	if (rc_matrix_zeros(&trajec_mat, 4, 3)) {
		fprintf(stderr, "ERROR: can't free memory for matrix");
		return -1;
	}
	trajec_mat.d[0][0] = 0; // time 0
	trajec_mat.d[0][1] = 0; // pos_x_r 0
	trajec_mat.d[0][2] = 0; // pos_y_r 1
	trajec_mat.d[1][0] = 3; // time 1
	trajec_mat.d[1][1] = 40; 
	trajec_mat.d[1][2] = 0;
	trajec_mat.d[2][0] = 6; // time 2
	trajec_mat.d[2][1] = 40; 
	trajec_mat.d[2][2] = 40;
	trajec_mat.d[3][0] = 9; // time 3
	trajec_mat.d[3][1] = 80;
	trajec_mat.d[3][2] = 80;
	
	// declare time (ms)
	cstate.t_1 = trajec_mat.d[0][0]; // assign first times
	cstate.t_2 = trajec_mat.d[1][0];
	
	// this should be the last step in initialization
	// to make sure other setup functions don't interfere
	rc_mpu_set_dmp_callback(&__position_controller);
	rc_set_state(RUNNING);
	test_start = rc_nanos_since_boot(); // in ns, used to check if first run
	__arm_controller();
	rc_led_set(RC_LED_RED, 0);
	rc_led_set(RC_LED_GREEN, 1);
	while (rc_get_state() != EXITING) {
		rc_usleep(200000);
	}

	// join parallel threads
	if (printf_thread) rc_pthread_timed_join(printf_thread, NULL, 1.5);
	if (battery_thread) rc_pthread_timed_join(battery_thread, NULL, 1.5);

	// final cleanup
	rc_filter_free(&D1);
	rc_filter_free(&D4);
	rc_filter_free(&D2);
	rc_filter_free(&D3);
	rc_motor_cleanup();
	rc_mpu_power_off();
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_encoder_cleanup();
	rc_remove_pid_file(); // remove pid file LAST
	return 0;
}

/**
 * This thread is in charge of adjusting the controller setpoint. 
 * In this test, is only responsible for a single motor.
 *
 * @param      ptr   The pointer
 *
 * @return     { description_of_the_return_value }
 */
void __setpoint_manager(void)
{
	// wait for mpu to settle
	//__disarm_controller();
	//rc_usleep(2500000);
	rc_set_state(RUNNING);
	rc_led_set(RC_LED_RED, 0);
	rc_led_set(RC_LED_GREEN, 1);

	
		// clear out input of old data before waiting for new data

		// sleep at beginning of loop so we can use the 'continue' statement
		//rc_usleep(1000000 / SETPOINT_MANAGER_HZ);

		// nothing to do if paused, go back to beginning of loop
		//if (rc_get_state() != RUNNING) continue;

		// if we got here the state is RUNNING, but controller is not
		// necessarily armed. If DISARMED, wait for the user to pick MIP up
		// which will we detected by wait_for_starting_condition()
		/*
		if (setpoint.arm_state == DISARMED) {
			if (__wait_for_starting_condition() == 0) {
				__zero_out_controller();
				__arm_controller();
			}
			else continue;
		}
		*/

	// if state becomes EXITING the above loop exits and we disarm here
	//__disarm_controller();
	//return NULL;
}

/**
* helper function to update setpoint based on a trapezoidal
* velocity model to reach specified destination
*/
static void __traject_new(void) {	
	// update current time, ms
	cstate.t_curr = rc_nanos_since_boot() / 1000000;

	// check if trajectory makes sense, more than just a start
	if (trajec_mat.rows < 2) {
		printf("ERROR: trajectory not filled");
		__disarm_controller();
	}

	// desired state = step + 1
	// current/prev state = step
	if (cstate.step + 2 < trajec_mat.rows) {
		// not yet aiming for final destination
		// update desired destination and time if surpassed
		if ((double)(cstate.t_curr - test_start) / 1000 >=
			trajec_mat.d[cstate.step + 1][0]) {
			++cstate.step;
			cstate.t_1 = trajec_mat.d[cstate.step][0]; // previous time, s
			cstate.t_2 = trajec_mat.d[cstate.step + 1][0]; // next time, s
		}
	}
	else {
		// aiming for final destination
		if ((double)(cstate.t_curr - test_start) / 1000 >=
			trajec_mat.d[cstate.step + 1][0]) {
			__disarm_controller();
			printf("Final destination reached. Thank you for choosing JerboBot Express.");
			cstate.v_xr_des = 0;
			cstate.v_yr_des = 0;
			rc_set_state(EXITING);
			return;
		}
	}
	
	// check times make sense
	if (cstate.t_1 > cstate.t_2) {
		fprintf(stderr, "ERROR: can't travel backwards in time :(");
		return;
	}
	
	// time since beginning of current maneuver
	double t_test = (double)(cstate.t_curr - test_start) / 1000
		- cstate.t_1;

	double xr_diff = trajec_mat.d[cstate.step + 1][1] -
		trajec_mat.d[cstate.step][1];
	int xr_sign = (xr_diff > 0) - (xr_diff < 0);
	double yr_diff = trajec_mat.d[cstate.step + 1][2] -
		trajec_mat.d[cstate.step][2];
	int yr_sign = (yr_diff > 0) - (yr_diff < 0);
	
	// update differences to be magnitude only
	xr_diff *= xr_sign;
	yr_diff *= yr_sign;

	// solutions for acceleration (and deacc) time for trapezoidal profile
	double t_ax = ((cstate.t_2 - cstate.t_1 -
		pow(pow((cstate.t_2 - cstate.t_1),2) - 4 * xr_diff / ACCEL_MAX,.5)) / 2);
	double t_ay = ((cstate.t_2 - cstate.t_1 -
		pow(pow((cstate.t_2 - cstate.t_1), 2) - 4 * yr_diff / ACCEL_MAX, .5)) / 2);

	// assign desired x_r velocity based on trapezoidal profile
	if (t_test <= t_ax) {
		// accelerating, trapezoid left
		cstate.v_xr_des = xr_sign * ACCEL_MAX * t_test;
	}
	else if (t_test >= cstate.t_2 - cstate.t_1 - t_ax) {
		// decelerating, trapezoid right
		cstate.v_xr_des = xr_sign * ACCEL_MAX * 
			(cstate.t_2 - cstate.t_1 - t_test);
	}
	else {
		// constant velocity, trapezoid plateau
		cstate.v_xr_des = xr_sign * ACCEL_MAX * t_ax;
	}

	// similarly, for y_r
	if (t_test <= t_ay) {
		cstate.v_yr_des = yr_sign * ACCEL_MAX * t_test;
	}
	else if (t_test >= cstate.t_2 - cstate.t_1 - t_ay) {
		cstate.v_yr_des = yr_sign * ACCEL_MAX * 
			(cstate.t_2 - cstate.t_1 - t_test);
	}
	else {
		cstate.v_yr_des = yr_sign * ACCEL_MAX * t_ay;
	}

	// update desired state
	setpoint.wheelAngle1 += (cstate.v_xr_des * DT); // /wheel_radius_xy
	setpoint.wheelAngle4 += (cstate.v_xr_des * DT);
	setpoint.wheelAngle2 += (cstate.v_yr_des * DT);
	setpoint.wheelAngle3 += (cstate.v_yr_des * DT);
}

/**
* discrete-time position controller for in-plane motion, called at
* SAMPLE_RATE_HZ
*/
static void __position_controller(void)
{
	static int inner_saturation_counter = 0;
	double duty1, duty4, duty2, duty3;
	//double duty5;
	
	if (test_start > 999999999) {
		// first time initializing test_start 
		// properly sync with start of control
		// careful if run on boot, may not work
		test_start = rc_nanos_since_boot() / 1000000; // ms
	}

	/**
	* updating desired state
	*/
	__traject_new();

	/******************************************************************
	* STATE_ESTIMATION
	* read sensors and compute the state when either ARMED or DISARMED
	* recall that wheels 1&4 move +y_r, wheels 2&3 move +x_r
	******************************************************************/
	double wheel1_old = cstate.wheelAngle1;
	double wheel4_old = cstate.wheelAngle4;
	double wheel2_old = cstate.wheelAngle2;
	double wheel3_old = cstate.wheelAngle3;
	//double wheel5_old = cstate.wheelAngle5;

	cstate.wheelAngle1 = (rc_encoder_read(ENCODER_CHANNEL_1) * 2.0 * M_PI) \
		/ (ENCODER_POLARITY_1 * GEARBOX_XY * ENCODER_RES);
	cstate.wheelAngle2 = (rc_encoder_read(ENCODER_CHANNEL_2) * 2.0 * M_PI) \
		/ (ENCODER_POLARITY_2 * GEARBOX_XY * ENCODER_RES);
	cstate.wheelAngle3 = (rc_encoder_read(ENCODER_CHANNEL_3) * 2.0 * M_PI) \
		/ (ENCODER_POLARITY_3 * GEARBOX_XY * ENCODER_RES);
	cstate.wheelAngle4 = (rc_encoder_read(ENCODER_CHANNEL_4) * 2.0 * M_PI) \
		/ (ENCODER_POLARITY_4 * GEARBOX_XY * ENCODER_RES);
	/*cstate.wheelAngle5 = (rc_encoder_read(ENCODER_CHANNEL_5) * 2.0 * M_PI) \
		/ (ENCODER_POLARITY_5 * GEARBOX_Z * ENCODER_RES);
	*/

	if (rc_mpu_read_accel(&mpu_data) < 0) {
		printf("read accel data failed\n");
	}
	if (rc_mpu_read_gyro(&mpu_data) < 0) {
		printf("read gyro data failed\n");
	}
	// TO DO: read magnetometer?

	// find change in encoder position
	double dAngle1 = cstate.wheelAngle1 - wheel1_old;
	double dAngle4 = cstate.wheelAngle4 - wheel4_old;
	double dAngle2 = cstate.wheelAngle2 - wheel2_old;
	double dAngle3 = cstate.wheelAngle3 - wheel3_old;
	//double dYaw = cstate.wheelAngle5 - wheel5_old;

	// change in position along resultant omni axes
	double dX_r = 0.5 * WHEEL_RADIUS_XY * (dAngle1 + dAngle4);
	double dY_r = 0.5 * WHEEL_RADIUS_XY * (dAngle2 + dAngle3);
	// rotation in omni axes due to differential drive
	cstate.theta += (2 * WHEEL_RADIUS_XY / (4 * TRACK_WIDTH)) *
		(dAngle4 - dAngle1 + dAngle2 - dAngle3);

	// translation in omni, rotated coordinates
	cstate.x_r += dX_r;
	cstate.y_r += dY_r;

	// convert to change in global coords
	cstate.x += dX_r * cos(ANGLE_GLOBAL2OMNI + cstate.theta)
		- dY_r * sin(ANGLE_GLOBAL2OMNI + cstate.theta);
	cstate.y += dX_r * sin(ANGLE_GLOBAL2OMNI + cstate.theta)
		+ dY_r * cos(ANGLE_GLOBAL2OMNI + cstate.theta);

	// correct for full rotation
	if (cstate.theta > 2 * M_PI) {
		cstate.theta = cstate.theta - 2 * M_PI;
	}
	else if (cstate.theta < -2 * M_PI) {
		cstate.theta = cstate.theta + 2 * M_PI;
	}
	//cstate.z += cstate.wheelAngle5 * WHEEL_RADIUS_Z;

	/*************************************************************
	* check for various exit conditions AFTER state estimate
	***************************************************************/
	/*
	if (rc_get_state() == EXITING) {
		rc_motor_set(0, 0.0);
		return;
	}
	// if controller is still ARMED while state is PAUSED, disarm it
	if (rc_get_state() != RUNNING && setpoint.arm_state == ARMED) {
		__disarm_controller();
		return;
	}
	// exit if the controller is disarmed
	if (setpoint.arm_state == DISARMED) {
		return;
	}
	*/

	/************************************************************
	* INNER LOOP ANGLE Theta controller D1
	* Input to D1 is theta error (setpoint-state). Then scale the
	* output u to compensate for changing battery voltage.
	*************************************************************/
	D1.gain = D1_GAIN * V_NOMINAL / cstate.vBatt; // comp for batt voltage
	D2.gain = D2_GAIN * V_NOMINAL / cstate.vBatt;
	D3.gain = D3_GAIN * V_NOMINAL / cstate.vBatt;
	D4.gain = D4_GAIN * V_NOMINAL / cstate.vBatt;
	cstate.d1_u = rc_filter_march(&D1, setpoint.wheelAngle1 
		- cstate.wheelAngle1);
	cstate.d4_u = rc_filter_march(&D4, setpoint.wheelAngle4
		- cstate.wheelAngle4);
	cstate.d2_u = rc_filter_march(&D2, setpoint.wheelAngle2
		- cstate.wheelAngle2);
	cstate.d3_u = rc_filter_march(&D3, setpoint.wheelAngle3
		- cstate.wheelAngle3);
	

	/*************************************************************
	* Check if the inner loop saturated. If it saturates for over
	* a second disarm the controller to prevent stalling motors.
	*************************************************************/
	if (fabs(cstate.d1_u) > 0.95) inner_saturation_counter++;
	else inner_saturation_counter = 0;
	// if saturate for a second, disarm for safety
	/*
	if (inner_saturation_counter > (SAMPLE_RATE_HZ*D1_SATURATION_TIMEOUT)) {
		printf("inner loop controller saturated\n");
		__disarm_controller();
		inner_saturation_counter = 0;
		return;
	}
	*/
	/**********************************************************
	* Send signal to motors
	* add D1 balance control u and D3 steering control also
	* multiply by polarity to make sure direction is correct.
	***********************************************************/
	duty1 = cstate.d1_u;
	duty4 = cstate.d4_u;
	duty2 = cstate.d2_u;
	duty3 = cstate.d3_u;
	rc_motor_set(MOTOR_CHANNEL_1, MOTOR_POLARITY_1 * duty1);
	rc_motor_set(MOTOR_CHANNEL_4, MOTOR_POLARITY_4 * duty4);
	rc_motor_set(MOTOR_CHANNEL_2, MOTOR_POLARITY_2 * duty2);
	rc_motor_set(MOTOR_CHANNEL_3, MOTOR_POLARITY_3 * duty3);

	return;
}

/**
 * Clear the controller's memory and zero out setpoints.
 *
 * @return     { description_of_the_return_value }
 */
static int __zero_out_controller(void)
{
	rc_filter_reset(&D1);
	rc_filter_reset(&D4);
	rc_filter_reset(&D2);
	rc_filter_reset(&D3);
	//setpoint.wheelAngle1 = 0.0;
	//setpoint.phi = 0.0;
	//setpoint.gamma = 0.0;
	rc_motor_set(0, 0.0);
	//rc_motor_set(4,0.0); // 0 has a bug, doesn't include motor4
	//rc_motor_set(5,0.0);
	return 0;
}

/**
 * disable motors & set the setpoint.core_mode to DISARMED
 *
 * @return     { description_of_the_return_value }
 */
static int __disarm_controller(void)
{
	rc_motor_standby(1);
	rc_motor_free_spin(0);
	setpoint.arm_state = DISARMED;
	return 0;
}

/**
 * zero out the controller & encoders. Enable motors & arm the controller.
 *
 * @return     0 on success, -1 on failure
 */
static int __arm_controller(void)
{
	__zero_out_controller();
	rc_encoder_write(ENCODER_CHANNEL_1, 0);
	rc_encoder_write(ENCODER_CHANNEL_4, 0);
	rc_encoder_write(ENCODER_CHANNEL_2, 0);
	rc_encoder_write(ENCODER_CHANNEL_3, 0);
	// prefill_filter_inputs(&D1,cstate.theta);
	rc_motor_standby(0);
	setpoint.arm_state = ARMED;
	return 0;
}

/**
 * prints diagnostics to console this only gets started if executing from
 * terminal
 *
 * @return     nothing, NULL pointer
 */
 static void* __print_loop(__attribute__((unused)) void* ptr)
 {
	 rc_state_t last_rc_state, new_rc_state; // keep track of last state
	 last_rc_state = rc_get_state();
	 if (!fout) {
		 printf("WARNING: Not saving output to file\n");
		 fout = stdout; // print to terminal if no filename
	 }
	 while (rc_get_state() != EXITING) {
		 new_rc_state = rc_get_state();
		 // check if this is the first time since being paused
		 if (new_rc_state == RUNNING && last_rc_state != RUNNING) {
			 rc_usleep(50000); // let controller catch up
			 fprintf(fout, "    t    "); // col1
			 fprintf(fout, "  wh_1   ");
			 fprintf(fout, "  wh_1s  ");
			 fprintf(fout, "  wh_2   ");
			 fprintf(fout, "  wh_2s  "); // col 5
			 fprintf(fout, "  wh_3   ");
			 fprintf(fout, "  wh_3s  ");
			 fprintf(fout, "  wh_4   ");
			 fprintf(fout, "  wh_4s  ");
			 fprintf(fout, " v_xr_des"); // col 10
			 fprintf(fout, " v_yr_des");
			 fprintf(fout, "    x    ");
			 fprintf(fout, "    y    ");
			 fprintf(fout, "   x_r   ");
			 fprintf(fout, "   y_r   "); // col 15
			 fprintf(fout, "  theta  ");
			 fprintf(fout, "   d1_u  ");
			 fprintf(fout, "   d2_u  ");
			 fprintf(fout, "   d3_u  ");
			 fprintf(fout, "   d4_u  "); // col 20
			 fprintf(fout, "   a_x   ");
			 fprintf(fout, "   a_y   ");
			 fprintf(fout, "theta_dot");
			 fprintf(fout, "\n");
		 }
		 else if (new_rc_state == PAUSED && last_rc_state != PAUSED) {
			 printf("\nPAUSED: press pause again to start.\n");
		 }
		 last_rc_state = new_rc_state;

		 // decide what to print or exit
		 if (new_rc_state == RUNNING) {
			 double x_r = cstate.x * cos(ANGLE_GLOBAL2OMNI + cstate.theta)
				 + cstate.y * sin(ANGLE_GLOBAL2OMNI + cstate.theta);
			 double y_r = -cstate.x * cos(ANGLE_GLOBAL2OMNI + cstate.theta)
				 + cstate.y * sin(ANGLE_GLOBAL2OMNI + cstate.theta);

			 fprintf(fout, "\r");
			 fprintf(fout, "%7.3f  ", (double)(cstate.t_curr - test_start)/1000);
			 fprintf(fout, "%7.3f  ", cstate.wheelAngle1);
			 fprintf(fout, "%7.3f  ", setpoint.wheelAngle1);
			 fprintf(fout, "%7.3f  ", cstate.wheelAngle2);
			 fprintf(fout, "%7.3f  ", setpoint.wheelAngle2);
			 fprintf(fout, "%7.3f  ", cstate.wheelAngle3);
			 fprintf(fout, "%7.3f  ", setpoint.wheelAngle3);
			 fprintf(fout, "%7.3f  ", cstate.wheelAngle4);
			 fprintf(fout, "%7.3f  ", setpoint.wheelAngle4);
			 fprintf(fout, "%7.3f  ", cstate.v_xr_des);
			 fprintf(fout, "%7.3f  ", cstate.v_yr_des);
			 fprintf(fout, "%7.3f  ", cstate.x);
			 fprintf(fout, "%7.3f  ", cstate.y);
			 fprintf(fout, "%7.3f  ", x_r);
			 fprintf(fout, "%7.3f  ", y_r);
			 fprintf(fout, "%7.5f  ", cstate.theta);
			 fprintf(fout, "%7.3f  ", cstate.d1_u);
			 fprintf(fout, "%7.3f  ", cstate.d2_u);
			 fprintf(fout, "%7.3f  ", cstate.d3_u);
			 fprintf(fout, "%7.3f  ", cstate.d4_u);
			 fprintf(fout, "%7.5f  ", mpu_data.accel[0]);
			 fprintf(fout, "%7.5f  ", mpu_data.accel[1]);
			 fprintf(fout, "%7.5f  ", mpu_data.gyro[2] * DEG_TO_RAD);
			 //fprintf(fout, "\n");
		 }
		 rc_usleep(1000000 / PRINTF_HZ);
	 }
	 return NULL;
 }

 /**
 * Slow loop checking battery voltage. Also changes the D1-4 saturation limit
 * since that is dependent on the battery voltage.
 *
 * @return     nothing, NULL poitner
 */
 static void* __battery_checker(__attribute__((unused)) void* ptr)
 {
	 double new_v;
	 while (rc_get_state() != EXITING) {
		 new_v = rc_adc_batt();
		 // if the value doesn't make sense, use nominal voltage
		 if (new_v > 13 || new_v < 10.0) new_v = V_NOMINAL;
		 cstate.vBatt = new_v;
		 rc_usleep(1000000 / BATTERY_CHECK_HZ);
	 }
	 return NULL;
 }