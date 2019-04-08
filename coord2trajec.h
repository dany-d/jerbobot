/**
* JerboBot Coord2Trajec
* Taylor Sun (taysun@umich.edu)
* Last updated: 3/27/19
*
*
* Convert input coordinates to 
* stored trajectory for jerbobot 
*/

#include <stdio.h>
#include <stdlib.h>
#include <rc/math.h>

void file_open(char *coord_in, rc_vector_t *trajec) { }

// REQ: movement command within field limits
// MOD:	write to omni-wheel, rotated coords
// EFFECT:	convert cartesian coords to rotated 45
//			, relevant to omni-wheels
void coord_convert(const int *coord_dif, int *coord_omni) { }

