/**
* JerboBot Odom Test
* Taylor Sun (taysun@umich.edu)
* Last updated: 3/27/19
*
*
* Test reading of encoders, conversion
* from omni-coord to global-coord
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


static int running = 0; 

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

// interupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running = 0;
	return;
}

static void __position_controller(void);	///< mpu interrupt routine
//static void* __printf_loop(void* ptr);

// global variables
static core_state_t cstate; 

int main() {
	//pthread_t printf_thread = 0;

	// set signal handler so the loop can exit cleanly
	//signal(SIGINT, __signal_handler);
	
	if (rc_kill_existing_process(2.0) < -2) return -1;

	// start signal handler, so can exit cleanly
	if (rc_enable_signal_handler() == -1) {
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

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

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	/*
	if (isatty(fileno(stdout))) {
		if (rc_pthread_create(&printf_thread, __printf_loop, (void*)NULL, SCHED_OTHER, 0)) {
			fprintf(stderr, "failed to start battery thread\n");
			return -1;
		}
	}
	*/

	signal(SIGINT, __signal_handler);
	running = 1;

	/*
	// running motors
	rc_motor_set(0, 0.5);
	rc_motor_set(4, 0.5); // all channels doesn't include motor4, bug
	*/

	// temp fix
	printf("  wh_1   ");
	printf("  wh_2   ");
	printf("  wh_3   ");
	printf("  wh_4   ");
	printf("    x    ");
	printf("    y    ");
	printf("   x_r   ");
	printf("   y_r   ");
	printf("  theta  ");
	printf("\n");

	while (running) {
		__position_controller();

		printf("\r");
		printf("%7.3f  ", cstate.wheelAngle1);
		printf("%7.3f  ", cstate.wheelAngle2);
		printf("%7.3f  ", cstate.wheelAngle3);
		printf("%7.3f  ", cstate.wheelAngle4);
		printf("%7.3f  ", cstate.x);
		printf("%7.3f  ", cstate.y);
		printf("%7.3f  ", cstate.x_r);
		printf("%7.3f  ", cstate.y_r);
		printf("%7.5f  ", cstate.theta);
		printf("\n");
		rc_usleep(50000);
	}
	
	// final cleanup
	//rc_motor_cleanup();
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
	cstate.theta += (2* WHEEL_RADIUS_XY/(4*TRACK_WIDTH)) * 
		(dAngle4 - dAngle1 + dAngle2 - dAngle3);


	// CHANGE MADE (4/9): removed 1/trackwidth from above in theta

	// ADDED TO TEST ONLY
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
	else if (cstate.theta < - 2 * M_PI) {
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
/*
static void* __printf_loop(__attribute__((unused)) void* ptr)
{
	rc_state_t last_rc_state, new_rc_state; // keep track of last state
	last_rc_state = rc_get_state();
	while (rc_get_state() != EXITING) {
		new_rc_state = rc_get_state();
		// check if this is the first time since being paused
		if (new_rc_state == RUNNING && last_rc_state != RUNNING) {
			printf("  wh_1   |");
			printf("  wh_2   |");
			printf("  wh_3   |");
			printf("  wh_4   |");
			printf("    x    |");
			printf("    y    |");
			printf("   x_r   |");
			printf("   y_r   |");
			printf("  theta  |");
			printf("\n");
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

			printf("\r");
			printf("%7.3f  |", cstate.wheelAngle1);
			printf("%7.3f  |", cstate.wheelAngle2);
			printf("%7.3f  |", cstate.wheelAngle3);
			printf("%7.3f  |", cstate.wheelAngle4);
			printf("%7.3f  |", cstate.x);
			printf("%7.3f  |", cstate.y);
			printf("%7.3f  |", x_r);
			printf("%7.3f  |", y_r);
			printf("%7.3f  |", cstate.theta);
		}
		rc_usleep(1000000 / 50);
	}
	return NULL;
}
*/