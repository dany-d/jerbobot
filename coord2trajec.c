/**
* JerboBot Coord2Trajec
* Taylor Sun (taysun@umich.edu)
* Last updated: 3/27/19
*
*
* convert input coordinates to
* trajectory for jerbobot
*/

#include "coord2trajec.h"

int file_open(char *coord_in, rc_vector_t *trajec) {
	int status; 

	FILE *fptr = fopen(coord_in, "r");
	if (fptr == NULL) {
		fprintf(stderr, "ERROR: Could not open file");
		return(-1);
	}
	
	fscanf(fptr, "1%d", trajec); // CHECK ON THIS SYNTAX

	return 0;
}

void coord_convert(const int *trajec, int *coord_omni) { 
	// while still have inputs from input trajectory, 
	// write to coord_omni
}