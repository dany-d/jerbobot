/**
* JerboBot OdomDrive Test
* Taylor Sun (taysun@umich.edu)
* Last updated: 4/11/19
*
*
* Test performance of motors to track
* specified trajectory
*/

#include <stdio.h>
#include <robotcontrol.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#define ENCODER_CHANNEL_1	1
#define ENCODER_CHANNEL_2	3
#define ENCODER_CHANNEL_3	4
#define ENCODER_CHANNEL_4	2
#define ENCODER_POLARITY_1	-1
#define ENCODER_POLARITY_2	-1
#define ENCODER_POLARITY_3	1
#define ENCODER_POLARITY_4	1
#define ENCODER_POLARITY_5	-1
#define WHEEL_RADIUS_XY		0.0762 // omni-wheel radius (m)
#define TRACK_WIDTH			0.52
#define ANGLE_GLOBAL2OMNI	M_PI/4
#define GEARBOX_XY			26.851
#define SAMPLE_RATE_HZ		100
#define ENCODER_RES			48
#define SAMPLE_RATE_HZ		100
#define DT					0.01

/** 
* This is the system state written to by the position controller. 
*/
typedef struct core_state_t {
	double wheelAngle1;	///< wheel rotation relative to body @ start
	double wheelAngle2;
	double wheelAngle3;
	double wheelAngle4;
	double wheelAngle5; ///< "wheel" rotation for telescoping arm

	double x;
	double y;
	double x_r; // 4/10 added rotated axes to test only  !!!!!
	double y_r;
	double z;
	double theta; ///< error in angle of omni-wheel axes
} core_state_t;

static void __position_controller(void);	///< mpu interrupt routine
static void* __print_loop(void* ptr); 

// global variables
static core_state_t cstate;
static rc_mpu_data_t mpu_data;
static FILE* fout = NULL;

/*
 * Printed if some invalid argument was given
 */
static void __print_usage(void)
{
	printf("\n");
	printf("-f {filename}     print results to filename (beware overwrite)\n");
	printf("-s                print results to terminal\n");
	printf("-h                print this help message\n");
	printf("\n");
}

int main(int argc, char *argv[]) {
	int c; 
	pthread_t printf_thread = 0;

	// parse arguments
	opterr = 0; 
	while ((c = getopt(argc, argv, ":f:sh")) != -1) {
		switch (c) {
		case 'f':  // print to file
			fout = fopen(optarg, "w");
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

	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privileges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
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

	/*
	// initialize motors
	if (rc_motor_init() == -1) {
		fprintf(stderr, "ERROR: failed to initialize motors\n");
		return -1;
	}
	rc_motor_standby(1); // start with motors in standby

	rc_usleep(1000000); // wait 1 second before starting
	*/

	/*
	// initialize adc
	if (rc_adc_init() == -1) {
		fprintf(stderr, "failed to initialize adc\n");
		adc_ok = false;
	}
	*/

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

	// this should be the last step in initialization
	// to make sure other setup functions don't interfere
	rc_mpu_set_dmp_callback(&__position_controller);

	/*
	// running motors
	rc_motor_set(0, 0.5);
	rc_motor_set(4, 0.5); // all channels doesn't include motor4, bug
	*/
	rc_set_state(RUNNING);
	while (rc_get_state() != EXITING) {
		rc_usleep(200000);
	}

	// final cleanup
	//rc_motor_cleanup();
	rc_mpu_power_off();
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_encoder_cleanup();
	return 0;
}

/**
* discrete-time position controller for in-plane motion, called at
* SAMPLE_RATE_HZ
*/
static void __position_controller(void)
{
	//static int inner_saturation_counter = 0;
	//double duty1, duty2, duty3, duty4, duty5;

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
	return;
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
			 fprintf(fout, "  wh_1   ");
			 fprintf(fout, "  wh_2   ");
			 fprintf(fout, "  wh_3   ");
			 fprintf(fout, "  wh_4   ");
			 fprintf(fout, "    x    ");
			 fprintf(fout, "    y    ");
			 fprintf(fout, "   x_r   ");
			 fprintf(fout, "   y_r   ");
			 fprintf(fout, "  theta  ");
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
			 fprintf(fout, "%7.3f  ", cstate.wheelAngle1);
			 fprintf(fout, "%7.3f  ", cstate.wheelAngle2);
			 fprintf(fout, "%7.3f  ", cstate.wheelAngle3);
			 fprintf(fout, "%7.3f  ", cstate.wheelAngle4);
			 fprintf(fout, "%7.3f  ", cstate.x);
			 fprintf(fout, "%7.3f  ", cstate.y);
			 fprintf(fout, "%7.3f  ", x_r);
			 fprintf(fout, "%7.3f  ", y_r);
			 fprintf(fout, "%7.5f  ", cstate.theta);
			 fprintf(fout, "\n");
		 }
		 rc_usleep(1000000 / 50);
	 }
	 return NULL;
 }