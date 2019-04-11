/**
* test_flysky.c
*
* Testing implementation of reading of flysky 
* controller's servo signal
*/


#include <stdio.h>

#define E_STOP = 3; 

static int running = 0; 

static void* __estop_handler(void* ptr);

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__((unused)) int dummy)
{
	running = 0;
	return;
}

int main() {
	rc_set_state(RUNNING);
	rc_led_set(RC_LED_GREEN, 1);
	
	while (rc_get_state() != EXITING) {

		// always sleep at some point
		rc_usleep(100000);
	}


	// turn off LEDs and close file descriptors
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	return 0;
}

/**
* Parallel thread monitoring state of lure attach and manual
* e-stop. 
*/
static void* __estop_handler(void* ptr) {
	rc_set_state(EXITING);

}

