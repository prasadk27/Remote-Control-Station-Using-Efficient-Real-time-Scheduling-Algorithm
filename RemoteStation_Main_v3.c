#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
//#include <string.h>
//#include <semaphore.h>

#define PERIOD050USEC 52                    // assuming ~1.048576 MHz DCO clock

#define L0DUTYCYCLE   0                     // L0 PWM duty cycle (fan off)
#define L1DUTYCYCLE   30                    // L1 PWM duty cycle
#define L2DUTYCYCLE   45                    // L2 PWM duty cycle
#define L3DUTYCYCLE   60                    // L3 PWM duty cycle
#define L4DUTYCYCLE   80                    // L4 PWM duty cycle
#define L5DUTYCYCLE   100                   // L5 PWM duty cycle (fan full speed)

typedef enum Status {
  ALARM, OK
} Status;

typedef enum FanLevel {
  FANSLEVEL0 = 0, FANSLEVEL1 = 1, FANSLEVEL2 = 2, FANSLEVEL3 = 3, FANSLEVEL4 = 4, FANSLEVEL5 = 5
} FanLevel;

// Port1 Pin Configuration
#define ACTIVITY    BIT4
#define LED1	BIT5
#define LED2	BIT3

// Port2 Pin Configuration
#define TACH        BIT2
#define PWM         BIT0

#define MAXLEVELSETTINGS  6
#define MAXTICKS	14

#define PULSESPERREVOLUTION  2              // # of frequency generator pulses per 1 revolution of the fan

FanLevel level = FANSLEVEL0;                // Cooling level
Status status = OK;                         // Alarm status of system whenever smoke is detected
volatile unsigned int flagTask0 = 0;		// Flags indicating status of tasks
volatile unsigned int flagTask1 = 0;
volatile unsigned int flagTask2 = 0;
volatile unsigned int flagTask3 = 0;
volatile unsigned int LEDstate = 0;						// Represents the 4 states of the 2 LEDs, default = 0
volatile unsigned char Rx_Data = 0;						// Store received data
char *Tx_Data;											// Store data to be transmitted
//char *string2;											// Store complete string along with RPM value

volatile uint8_t  task_id; 					// has the current running task

unsigned int tickCount = 0;					// Global Tick Count
unsigned int dlTask0 = 2;					// Preset deadlines in terms of ticks
unsigned int dlTask1 = 4;
unsigned int dlTask2 = 10;
unsigned int dlTask3 = 6;

unsigned int ticksToDlTask0 = MAXTICKS;			// Count of ticks left to reach deadline
unsigned int ticksToDlTask1 = MAXTICKS;
unsigned int ticksToDlTask2 = MAXTICKS;
unsigned int ticksToDlTask3 = MAXTICKS;

unsigned int countTask0 = 0;				// For count of ticks once task is released
unsigned int countTask1 = 0;
unsigned int countTask2 = 0;
unsigned int countTask3 = 0;

unsigned int count = 0;
unsigned int count0 = 0;
unsigned int count1 = 0;					// For debug
unsigned int count2 = 0;
unsigned int count3 = 0;
unsigned int countADC = 0;

											// Duty cycles for each level
unsigned int dutyCycles[ MAXLEVELSETTINGS] =
  { L0DUTYCYCLE, L1DUTYCYCLE, L2DUTYCYCLE,
    L3DUTYCYCLE, L4DUTYCYCLE, L5DUTYCYCLE};

void Sys_init();
void ADC_init(void);
void USCI_A0_init(void);
void PWM_init(void);
void task0(void);
void task1(void);
void task2(void);
void task3(void);
void scheduler(void);

int main(void) {
	Sys_init();								// Initialize the system
	ADC_init();								// Initialize ADC
	USCI_A0_init();							// Initialize UART mode
	PWM_init();								// Initialize PWM
	flagTask1 = 1;							// Set initial level and speed
	Tx_Data = "Level 0 ";

	ADC12CTL0 |= ADC12SC;                   		// Start sampling/conversion
	task_id = 0;
	task0();

	_EINT();								// Enable Global Interrupts

	while(1)
	{
		__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
		__no_operation();							// No Operation
	}
}



void scheduler(void)
{
	if (task_id == 0)
	{
		task0();
	}
	else if (task_id == 1)
	{
		task1();
	}
	else if (task_id == 2)
	{
		task2();
	}
	else if (task_id == 3)
	{
		task3();
	}

	count3++;									// for debug
}

void Sys_init()
{
	WDTCTL = WDTPW | WDTHOLD;				  // Stop watchdog timer
	P3SEL = BIT3 + BIT4;                      // P3.3,4 = USCI_A0 TXD/RXD
	P1SEL &= ~(ACTIVITY);                     // Set I/O function for alarm
	P1DIR |= (ACTIVITY);                      // Set output direction for alarm
	P1OUT &= ~(ACTIVITY);						// Set as low
	P1SEL &= ~(LED1);                     	// Set I/O function for LED1
	P1DIR |= (LED1);                      	// Set output direction for LED1
	P1OUT &= ~(LED1);						// Set as low
	P1SEL &= ~(LED2);                     	// Set I/O function for LED2
	P1DIR |= (LED2);                      	// Set output direction for LED2
	P1OUT &= ~(LED2);						// Set as low
	P2SEL &= ~(TACH);                         // Set I/O function
	P2DIR &= ~(TACH);                         // Set input direction
	//P2IE  |= (TACH);                          // Enable Port 2 interrupt
	//P2IES |= (TACH);                          // Generate ints falling edge
	//TA0CCTL0 = CCIE;						// CCR0 interrupt enabled
	//TA0CCR0 = 12500;						// Count value to generate timer intervals every 100ms
	//TA0CTL = TASSEL_2 + MC_1 + ID_3;         // SMCLK, upmode, clear TAR
	WDTCTL = WDTPW + WDTSSEL1 + WDTTMSEL + WDTCNTCL + WDTIS_7;		// Set WDT in interval timer mode, ~1.95 ms intervals
	SFRIE1 |= WDTIE;                          // Enable Watchdog interrupt
}

void USCI_A0_init(void)
{
	UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
	UCA0CTL1 |= UCSSEL_2;                     // SMCLK
	UCA0BR0 = 109;                            // 1MHz 9600 - Check for compatibility with HC06
	UCA0BR1 = 0;                              // 1MHz 9600
	UCA0MCTL |= UCBRS_1 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
	UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

void PWM_init(void)
{
  P2DIR |= PWM;                             // Configure output direction
  P2SEL |= PWM;                             // Select peripheral option
  TA1CCR0 = PERIOD050USEC;                  // Set up the PWM Period
  TA1CTL = TASSEL_2 + MC_1 + TACLR;         // Use SMCLK, count up mode, clear TAR
  TA1CCTL1 = OUTMOD_1;                      // Keep fan(s) off initially
}

void ADC_init(void)
{
	ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
	ADC12CTL1 = ADC12SHP;                     // Use sampling timer
	ADC12IE = 0x01;                           // Enable interrupt
	ADC12CTL0 |= ADC12ENC;					  // Enable Conversion
	P6SEL |= 0x01;                            // P6.0 ADC option select for sensor data
}

void task0(void)
{
		if (flagTask0 == 1)								// If Smoke detected, sound alarm
			{
				flagTask0 = 0;							// Clear Flag and count
				countTask0 = 0;
				ticksToDlTask0 = MAXTICKS;
				status = ALARM;
				P1OUT |= ACTIVITY;                   	   // Set ALARM LED
				level = FANSLEVEL0;						// Change level
				flagTask1 = 1;           				// Event registered for task 1 to take place
				Tx_Data = "X";							// Store emergency character to be transmitted
			}
		count0++;										// for debug
}

void task1(void)
{
	flagTask1 = 0;							// Clear flag, count for Task1
	countTask1 = 0;
	ticksToDlTask1 = MAXTICKS;

	float dutyCycle;
	                                            // Convert duty cycle to a %
	dutyCycle = ((float)(dutyCycles[level]))/100;

	TA1CCR1 = (dutyCycle * PERIOD050USEC);  // Update the compare register
	TA1CCTL1 = OUTMOD_7;                    // Generate PWM via out mode 7

	flagTask2 = 1;							// Event registered for task 2 to take place

	count1++;								// for debug
}

void task2(void)
{
	flagTask2 = 0;							// Clear flag, count for Task2
	countTask2 = 0;
	ticksToDlTask2 = MAXTICKS;

	char *string1 = Tx_Data;					// Store data to be transmitted
	//strTX = malloc(strlen(string1)+strlen(string2)+1);
	//strcpy(strTX, string1);						// Copy string1 in strTX
	//strcat(strTX, string2);						// Add string2
	UCA0IE |= UCTXIE;               		// Enable USCI_A0 TX interrupt
	while (*string1 != '\0')
	{
		while (!(UCA0IFG&UCTXIFG));     	// USCI_A0 TX buffer ready?
		UCA0TXBUF = *string1;				// TX string, character by character
		string1++;
	}
	UCA0IE &= ~UCTXIE;						// Disable USCI_A0 TX interrupt

	count2++;								// for debug
}

void task3(void)
{
	flagTask3 = 0;								// Clear flag, count for Task3
	countTask3 = 0;
	ticksToDlTask3 = MAXTICKS;

	switch (LEDstate)
	{
		case 1:
			P1OUT &= ~(LED1);						// Set as low
			LEDstate = 0;
			flagTask2 = 1;							// Event registered for task 2 to take place
			break;

		case 2:
			P1OUT |= LED1;							// Set as high
			LEDstate = 0;
			flagTask2 = 1;							// Event registered for task 2 to take place
			break;

		case 3:
			P1OUT &= ~(LED2);						// Set as low
			LEDstate = 0;
			flagTask2 = 1;							// Event registered for task 2 to take place
			break;

		case 4:
			P1OUT |= LED2;							// Set as high
			LEDstate = 0;
			flagTask2 = 1;							// Event registered for task 2 to take place
			break;

		default: break;
	}
}

// WDT Interrupt Service Routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(WDT_VECTOR))) WDT_ISR (void)
#else
#error Compiler not supported!
#endif
{
	if (flagTask0 == 1)											// Increase Tick count for a particular task once it is released
	{
		countTask0++;
		ticksToDlTask0 = dlTask0 - countTask0;					// calculate ticks left to reach deadline
	}
	else if (flagTask1 == 1)
	{
		countTask1++;
		ticksToDlTask1 = dlTask1 - countTask1;					// calculate ticks left to reach deadline
	}
	else if (flagTask2 == 1)
	{
		countTask2++;
		ticksToDlTask2 = dlTask2 - countTask2;					// calculate ticks left to reach deadline
	}
	else if (flagTask3 == 1)
	{
		countTask3++;
		ticksToDlTask3 = dlTask3 - countTask3;					// calculate ticks left to reach deadline
	}

	// EDF Scheduling algorithm
	if ((ticksToDlTask0 < ticksToDlTask1) && (ticksToDlTask0 < ticksToDlTask2) && (ticksToDlTask0 < ticksToDlTask3))
	{
		task_id = 0;
	}
	else if ((ticksToDlTask1 < ticksToDlTask0) && (ticksToDlTask1 < ticksToDlTask2) && (ticksToDlTask1 < ticksToDlTask3))
	{
		task_id = 1;
	}
	else if ((ticksToDlTask2 < ticksToDlTask0) && (ticksToDlTask2 < ticksToDlTask1) && (ticksToDlTask2 < ticksToDlTask3))
	{
		task_id = 2;
	}
	else if ((ticksToDlTask3 < ticksToDlTask0) && (ticksToDlTask3 < ticksToDlTask1) && (ticksToDlTask3 < ticksToDlTask2))
	{
		task_id = 3;
	}
	else
	{
		task_id = 0;
	}

	tickCount++;
	count++;									// for debug
	scheduler();								// Invoke scheduler
	LPM0_EXIT;                                    // Exit out of LPM0
}

// USCI_A0 ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
		Rx_Data = UCA0RXBUF;								// Store received data in variable
		switch (Rx_Data)
		  	  {
		  	  	  case 'A':									// Level 1
		  	  		level = FANSLEVEL1;						// Change level
		  	  		flagTask1 = 1;              			// Update PWM and speed, Event registered for task 1 to take place
		  	  		Tx_Data = "Level 1 ";
		  	  		break;

		  	  	  case 'B':									// Level 2
		  	  		level = FANSLEVEL2;						// Change level
		  	  		flagTask1 = 1;              			// Update PWM and speed, Event registered for task 1 to take place
		  	  		Tx_Data = "Level 2 ";
		  	 	  	break;

		  	  	  case 'C':									// Level 3
		  	  		level = FANSLEVEL3;						// Change level
		  	  		flagTask1 = 1;              			// Update PWM and speed, Event registered for task 1 to take place
		  	  		Tx_Data = "Level 3 ";
			  	  	break;

		  	  	  case 'D':									// Level 4
		  	  		level = FANSLEVEL4;						// Change level
		  	  		flagTask1 = 1;              			// Update PWM and speed, Event registered for task 1 to take place
		  	  		Tx_Data = "Level 4 ";
				  	break;

		  	  	  case 'E':									// Level 5
		  	  		level = FANSLEVEL5;						// Change level
		  	  		flagTask1 = 1;              			// Update PWM and speed, Event registered for task 1 to take place
		  	  		Tx_Data = "Level 5 ";
		  	  		break;

		  	  	  case 'F':									// Level 0 - OFF
		  	  		level = FANSLEVEL0;						// Change level
		  	  		flagTask1 = 1;              			// Update PWM and speed, Event registered for task 1 to take place
		  	  		Tx_Data = "Level 0 ";
		  	  		break;

		  	  	  case 'G':
		  	  		LEDstate = 1;							// LED1 Off
		  	  		flagTask3 = 1;							// Event registered for Task 3 to take place
		  	  		Tx_Data = "LED1 is OFF";
		  	  		break;

		  	  	  case 'H':
		  	  		LEDstate = 2;							// LED1 On
		  	  		flagTask3 = 1;							// Event registered for Task 3 to take place
		  	  		Tx_Data = "LED1 is ON";
		  	  		break;

		  	  	  case 'I':
		  	  		LEDstate = 3;							// LED2 Off
		  	  		flagTask3 = 1;							// Event registered for Task 3 to take place
		  	  		Tx_Data = "LED2 is OFF";
		  	  		break;

		  	  	  case 'J':
		  	  		LEDstate = 4;							// LED2 On
		  	  		flagTask3 = 1;							// Event registered for Task 3 to take place
		  	  		Tx_Data = "LED2 is ON";
		  	  		break;

		  	  	  default:
		  	  		break;
		  	  }
	//}
	//__bic_SR_register_on_exit(LPM0_bits);		// Wake-up CPU

}

//ADC12_ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6:                                  // Vector  6:  ADC12IFG0
	countADC++;
    if (ADC12MEM0 >= 0x7ff)              	// ADC12MEM = A0 > 0.5AVcc, Threshold value for firing alarm
    {
      flagTask0 = 1;
    }
    else
    {
    	ADC12CTL0 |= ADC12SC;                   		// Start sampling/conversion
    }
    //__bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
      break;
  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                           // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break;
  }
}
