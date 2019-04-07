/**
 * @file rc_test_motors.c
 * @example    rc_test_motors
 *
 * Demonstrates use of H-bridges to drive motors with the Robotics Cape and
 * BeagleBone Blue. Instructions are printed to the screen when called.
 */


#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <rc/motor.h>
#include <rc/time.h>

static int running = 0;
void motor_run(int motorR, int motorL, double duty, int delay);
void run_routine(double duty);

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__((unused)) int dummy)
{
	running = 0;
	return;
}

int main()
{
	double duty = 0.0;
	int freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ;
	duty = 1;

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;

	// initialize hardware first
	if (rc_motor_init_freq(freq_hz)) return -1;

	printf("sending duty cycle %0.4f\n", duty);

	run_routine(duty);
	// wait untill the user exits
	while (running) {
		// if not in SWEEP mode, the motors have already been set so do nothing

		rc_usleep(500000);
	}


	// final cleanup
	printf("\ncalling rc_motor_cleanup()\n");
	rc_motor_cleanup();
	return 0;
}

void run_routine(double duty) {
	int delay = 250000;
	
	// move +X_robot
	motor_run(1, 4, duty, delay);

	// move -X_robot
	motor_run(1, 4, -duty, delay);
	rc_usleep(delay);

	// move +Y_robot
	motor_run(3, 2, duty, delay);
	rc_usleep(delay);

	// move -Y_robot
	motor_run(3, 2, -duty, delay);
	rc_usleep(delay);

	// move diagonal
	rc_motor_set(1, -duty);
	rc_motor_set(4, duty);
	rc_motor_set(2, duty);
	rc_motor_set(3, -duty);
	rc_usleep(delay);
	rc_motor_set(1, 0);
	rc_motor_set(4, 0);
	rc_motor_set(2, 0);
	rc_motor_set(3, 0);
	rc_usleep(delay);

	rc_motor_set(1, duty);
	rc_motor_set(4, -duty);
	rc_motor_set(2, -duty);
	rc_motor_set(3, duty);
	rc_usleep(delay);
	rc_motor_set(1, 0);
	rc_motor_set(4, 0);
	rc_motor_set(2, 0);
	rc_motor_set(3, 0);
	rc_usleep(delay);
}

void motor_run(int motorR, int motorL, double duty, int delay) {
	rc_motor_set(motorR, duty);
	rc_motor_set(motorL, -duty);
	rc_usleep(delay);
	// let come to stop
	rc_motor_set(motorR, 0);
	rc_motor_set(motorL, 0);
	rc_usleep(delay);
}