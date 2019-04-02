/**
* JerboBot Odometry
* Taylor Sun (taysun@umich.edu)
* Last updated: 3/27/19
*
* convert stored trajectory to
* moving strategy for robot
*/

#include "odom.h"
#include <rc/motor.h>

// REQ: 
// MOD:	
// EFFECT:	set operation mode, PID vs simple
void run_mode(int mode) { }

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
