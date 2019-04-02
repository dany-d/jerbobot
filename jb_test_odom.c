/**
* JerboBot Motor Test
* Taylor Sun (taysun@umich.edu)
* Last updated: 3/27/19
*
*
* convert stored trajectory to
* moving strategy for robot
*/

#include <stdio.h>
#include "odom.h"
#include <time.h>


int main() {
	

	// start signal handler, so can exit cleanly
	if (rc_enable_signal_handler() == -1) {
		fprintf(stderr,"ERROR: failed to complete rc_enable_signal handler\n");
	}

	

}