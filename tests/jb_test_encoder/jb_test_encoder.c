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

#define CLR B00000000
#define RD B01000000
#define WR B10000000
#define LOAD B11000000

#define MDR0 B00001000
#define MDR1 B00010000
#define DTR B00011000
#define CNTR B00100000
#define OTR B00101000
#define STR B00110000

#define MDR0_CONF B00000011
#define MDR1_CONF B00000000

#define SPI_SPEED 24000000
#define BUS 1
#define SLAVE 0



int main() {
	char buf[32];
	int ret;

	
	if (rc_spi_auto_slave(BUS, SLAVE, SPI_MODE_0, SPI_SPEED)) {
		return -1;
	}

	// ss low

	rc_spi_write(BUS, SLAVE, WR | MDR0, 1);
	rc_spi_write(BUS, SLAVE, MDR0_CONF, 1);
	rc_spi_write(BUS, SLAVE, WR | MDR1, 1);
	rc_spi_write(BUS, SLAVE, MDR1_CONF, 1);
	rc_spi_write(BUS, SLAVE, CLR | CNTR, 1);

	uint64 count;
	while (true) {
		rc_spi_write(BUS, SLAVE, LOAD | OTR, 1);
		rc_spi_transfer(BUS, SLAVE, RD | OTR, 1, count);
		count <<= 8;
		printf(count);
	}

	rc_spi_close(BUS);
	return 0;
}