
#include "p24HJ64GP502.h"
#include "stdint.h"
#include "string.h"

/*******************************
 * Configuration selections
 ******************************/
// FOSCSEL and FOSC default to FRC with divider and pin 10 = Fcpu
_FWDT(FWDTEN_OFF); 			// Turn off watchdog timer
_FICD(JTAGEN_OFF & ICS_PGD1);  // JTAG is disabled; ICD on PGC1, PGD1
_FOSC(IOL1WAY_OFF);

#define DATA_WORDS 8

char min_data[DATA_WORDS];


void init();
void zero_dc();
void update_display();

int DELAY;
int SCRATCH;
int i;


#define Delay(x) DELAY = x+1; while(--DELAY){ Nop(); Nop(); }
#define XBLNK LATBbits.LATB12
#define GSLAT_M LATBbits.LATB6

//////// Main program //////////////////////////////////////////

int main()
{

    init();
    //zero_dc();
    XBLNK = 1;
    while(1)
    {
        for(i = 0; i < 8; i++){
            min_data[i] = i;
        }
    update_display();
    
    Delay(1000);
    }
}

/*******************************
 * Initialize oscillator and main clock
 * Set IO pins and output to zero
 * Initialize drivers' data to zero
 * Get initial RPG state
 * Initialize LED driver clock
 ******************************/
void init() {
	//__builtin_write_OSCCONH(0x07);

	//_IOLOCK =0;
	OSCCON = 0x0701;		// Use fast RCoscillator with Divide by 16
	CLKDIV = 0x0200;		// Divide by 4
	OSCTUN = 17;                    // Tune for Fcpu = 1 MHz

	// unlock peripheral pin mapping
	__builtin_write_OSCCONL(0x46);
	__builtin_write_OSCCONL(0x57);
	__builtin_write_OSCCONL(0x17);

	TRISA = 0x0004;
	// Set up IO. 1 is input
	TRISB = 0x003F;
	AD1PCFGL = 0xFFFF;

	Nop();

	// Turn off drivers
	//LATB = 0x3040;
	XBLNK = 0;
	GSLAT_M = 1;

	Delay(200);

	// Connect a SPI peripheral to the DC pins
	// Clear and disable interrupts
	_SPI1IF = 0;
	_SPI1IE = 0;

	// SCK1 is 01000
	// SDO1 is 00111
	// Connect it to the output pins RP11 and RP10
	RPOR5 = 0x0807;

	// Turn it on in master mode
	SPI1CON1 = 0x013F;
	SPI1STATbits.SPIEN = 1;

	// Just in case
	Delay(8000)

	// lock peripheral pin mapping
	__builtin_write_OSCCONL(0x46);
	__builtin_write_OSCCONL(0x57);
	__builtin_write_OSCCONL(0x57);

}

void update_display() {
	int i;
	// send minutes
	GSLAT_M = 0;
	for (i = DATA_WORDS - 1; i >= 0; i--) {
		SPI1BUF = min_data[i];
		while (!SPI1STATbits.SPITBF);
	}
	GSLAT_M = 1;
}
