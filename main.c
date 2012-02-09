#include "p24HJ64GP502.h"


/*******************************
 * Configuration selections
 ******************************/
// FOSCSEL and FOSC default to FRC with divider and pin 10 = Fcpu
_FWDT(FWDTEN_OFF); 			// Turn off watchdog timer
_FICD(JTAGEN_OFF & ICS_PGD1);  // JTAG is disabled; ICD on PGC1, PGD1
_FOSC(IOL1WAY_OFF);

#define DATA_WORDS 8


/*************************************************
 *
 * Set up the variables and constants for PSOC SPI
 *
 *************************************************/
#define PSOC_TX_LENGTH 12               // Data Length
#define PSOC_SS LATBbits.LATB7          // Slave Select for the PSOC SPI
char MTR_PSOC_TX_BUF[PSOC_TX_LENGTH];   // PSOC TX data buffer
void MTR_PSOC_SPI_INIT(void);
void MTR_PSOC_SPI(void);
void Motorpsoc_Foreward(void);
void Motorpsoc_ClrBuf(void);



#define MA_DUTY   (MTR_PSOC_TX_BUF[0])
#define MB_DUTY   (MTR_PSOC_TX_BUF[1])
#define MA_CTL    (MTR_PSOC_TX_BUF[2])
#define MB_CTL    (MTR_PSOC_TX_BUF[3])

#define DIR_MASK            0x80//0b10000000
#define BRAKE_MASK          0x40//0b01000000
#define COAST_MASK          0x20//0b00100000
#define RESET_MASK          0x08//0b00001000
#define HALFBRIDGE_MASK     0x01//0b00000001

/**************************************************
 *
 *
 * Set up a timer to control the SPI
 *
 *************************************************/
void timerinit(void);






char min_data[DATA_WORDS];


void init();
void zero_dc();
void update_display();

int DELAY;
int SCRATCH;
int i;


#define Delay(x) DELAY = x+1; while(--DELAY){ Nop(); Nop(); }
#define XBLNK LATBbits.LATB15
#define GSLAT_M LATBbits.LATB6

//////// Main program //////////////////////////////////////////

int main()
{

    init();
    //zero_dc();
    XBLNK = 1;
    timerinit();
    Motorpsoc_ClrBuf();
    
    while(1)
    {
        Motorpsoc_Foreward();
        if(IFS0bits.T1IF){
            XBLNK != XBLNK;
            IFS0bits.T1IF = 0;
            MTR_PSOC_SPI();
        }
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
        /*
	__builtin_write_OSCCONL(0x46);
	__builtin_write_OSCCONL(0x57);
	__builtin_write_OSCCONL(0x17);
        */
	TRISA = 0x0004;
	// Set up IO. 1 is input
	TRISB = 0x003F;
	AD1PCFGL = 0xFFFF;

	Nop();

        

	// Turn off drivers
	//LATB = 0x3040;
	XBLNK = 0;
	GSLAT_M = 1;
        PSOC_SS = 1;


	Delay(200);

	// Connect a SPI peripheral to the DC pins
	// Clear and disable interrupts
	_SPI1IF = 0;
	_SPI1IE = 0;

	// SCK1 is 01000
	// SDO1 is 00111
	// Connect it to the output pins RP11 and RP10
	//RPOR5 = 0x0807;
        MTR_PSOC_SPI_INIT();

	// Turn it on in master mode
	SPI1CON1 = 0x013F;
	SPI1STATbits.SPIEN = 1;

	// Just in case
	Delay(8000)

	// lock peripheral pin mapping
	//__builtin_write_OSCCONL(0x46);
	//__builtin_write_OSCCONL(0x57);
	//__builtin_write_OSCCONL(0x57);

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
void MTR_PSOC_SPI(){
    int i;
    SPI1BUF = 0x55;
    while(!SPI1STATbits.SPITBF);
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    
    
    for(i = 0; i < PSOC_TX_LENGTH; i++)
    {
        PSOC_SS = 0;
        SPI1BUF = MTR_PSOC_TX_BUF[i];
        while(!SPI1STATbits.SPITBF);
        Delay(100);
    }
    Nop();
    PSOC_SS = 1;
}

void MTR_PSOC_SPI_INIT(){

    __builtin_write_OSCCONL(0x46);
    __builtin_write_OSCCONL(0x57);
    __builtin_write_OSCCONL(0x17);        // Unlock Peripherals,
                                          // IOLOCK did not work

    RPOR4bits.RP8R = 7;          // Data out on Pin 8
    RPOR3bits.RP6R = 8;          // Serial Clock on Pin 6
    RPINR20bits.SDI1R = 9;      // Pin 9 Set as SPI
                                // Slave Select set up on Pin 7
    //RPOR5 = 0x0807;


    __builtin_write_OSCCONL(0x46);
    __builtin_write_OSCCONL(0x57);
    __builtin_write_OSCCONL(0x57);          // Lock Peripherals



}




void Motorpsoc_Foreward() {


    if (MA_DUTY ==0 && MB_DUTY == 0) {  // if stopped
        MA_CTL = BRAKE_MASK | COAST_MASK;   // reverse motor direction and go forwards
        MB_CTL = BRAKE_MASK | COAST_MASK;
    }

    if (MA_CTL & DIR_MASK)  {    // if going backwards, decrease duty cycle
        MA_CTL = DIR_MASK | BRAKE_MASK | COAST_MASK;    // add DIR_MASK to reverse motors
        MB_CTL = DIR_MASK | BRAKE_MASK | COAST_MASK;

        if (MA_DUTY >= 80)    // a duty cycle between 0 and 80 doesn't move the robot
          MA_DUTY -= 10;
        else
          MA_DUTY = 0;

        if (MB_DUTY >= 80)
          MB_DUTY -= 10;
        else
          MB_DUTY = 0;

    } else {                     // if going forward, increase duty cycle

       MA_CTL = BRAKE_MASK | COAST_MASK;
       MB_CTL = BRAKE_MASK | COAST_MASK;

       if (MA_DUTY < 80)   // a duty cycle between 0 and 80 doesn't move the robot
        MA_DUTY = 80;
       else if (MA_DUTY < 192) // max duty cycle at 192
         MA_DUTY += 10;
       else
         MA_DUTY = 192;

       if (MB_DUTY < 80)
        MB_DUTY = 80;
       else if (MB_DUTY < 192)
         MB_DUTY += 10;
       else
         MB_DUTY = 192;

    }
}

void Motorpsoc_ClrBuf() {
  MA_DUTY = 0;
  MB_DUTY = 0;
  MA_CTL = 0;
  MB_CTL = 0;
  MTR_PSOC_TX_BUF[4] = 0xaa;
  MTR_PSOC_TX_BUF[5] = 0xaa;
  MTR_PSOC_TX_BUF[6] = 0xaa;
  MTR_PSOC_TX_BUF[7] = 0xaa;
  MTR_PSOC_TX_BUF[8] = 0xaa;
  MTR_PSOC_TX_BUF[9] = 0xaa;
  MTR_PSOC_TX_BUF[10] = 0xaa;
  MTR_PSOC_TX_BUF[11] = 0xaa;
}

void timerinit(){

    T1CONbits.TON = 0; // Disable Timer
    T1CONbits.TCS = 0; // Select internal instruction cycle clock
    T1CONbits.TGATE = 0; // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b11; // Select 1:1 Prescaler
    TMR1 = 0x00;  // Clear timer register
    PR1 = 0x7FFF;  // Load the period value
    IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
    IEC0bits.T1IE = 0; // Enable Timer1 interrupt
    T1CONbits.TON = 1; // Start Timer

}
