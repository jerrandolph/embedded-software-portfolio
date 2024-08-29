#include <msp430.h>

/*
 * Port Mapping
 * Port 1
 *      Pin 0: Green LED
 *      Pin 1:
 *      Pin 2:
 *      Pin 3: Btn Reset Horizontal Position
 *      Pin 4: Accelerometer Y Analog Input (A4)
 *      Pin 5: Accelerometer X Analog Input (A5)
 *      Pin 6:
 *      Pin 7: Accelerometer Z Analog Input (A7) (is this needed?)
 * Port 2
 *      Pin 0
 *      Pin 1: DC Motor PWM negative signal
 *      Pin 2: DC Motor PWM positive signal
 *      Pin 3
 *      Pin 4
 *      Pin 5
 *      Pin 6
 *      Pin 7
 */

#define BIT_GREEN_LED BIT0
#define BIT_G_RESET BIT3

// On P2, Bits 2 and 1 support PWM on TA1.1
#define BIT_DC_MOTOR_PWM_POS BIT1
#define BIT_DC_MOTOR_PWM_NEG BIT2

#define PWM_TICK_PERIOD (0x33)

#define PWM_PORT_DIR P2DIR
#define PWM_PORT_OUT P2OUT
#define PWM_PORT_SEL P2SEL

// Incoming analog acceleration data for the Y direction is from P1.3
#define BIT_ACC_IN_Y BIT4

void setupIO() {

    // Setup the green LED as an output
    P1DIR |= BIT_GREEN_LED;
    P1OUT &= ~BIT_GREEN_LED;

    // Setup the button as an input
    P1DIR &= ~BIT_G_RESET;
    P1IE |= BIT_G_RESET;
}


void setupAccelerometerAnalogInput() {
    ADC10CTL0 &= ~ENC;      // Disable AD conversion
    ADC10CTL0 |= ADC10ON;   // Turn on ADC10

    // Set up a reference voltage. This accelerometer ranges from 0v to 3.3v, with 0 acceleration at 1.65 volts.
    // Normal earth gravity registers somewhere around 2 volts, and -gravity is 1.29 volts.
    // We're using earth's gravity field to tell if we're tilted, so we shouldn't expect accelerations over 2.5 volts.
    ADC10CTL0 |= REFON;     // Use reference voltage

    // 1.5v will be the ADC max, so the incoming analog signal will have to be dropped.
    ADC10CTL0 &= ~REF2_5V;   // Ref voltage to 1.5
    //ADC10CTL0 |= REF2_5V;   // Ref voltage to 2.5

    ADC10CTL0 |= SREF_1;    // Use the ref voltage as an upper bound
    ADC10CTL0 |= MSC;       // Automatically perform multiple sample captures
    ADC10CTL0 |= ADC10IE;   // Enable interrupts
    ADC10CTL0 &= ~ADC10IFG; // Clear the interrupt flag

    ADC10CTL1 |= INCH_4;      // Read analog values on channel 4, which corresponds to P1.4
    ADC10AE0 |= BIT_ACC_IN_Y; // ADC10 Input Enable

    ADC10CTL1 |= ADC10DIV_7;  // Divide clock by 8;
    ADC10CTL1 |= ADC10SSEL_3; // Use SMCLK
    ADC10CTL1 |= CONSEQ_2;    // Repeat single-channel mode

    ADC10CTL0 |= ENC;   // Enable AD conversion
}

// transmitFloat transmits the given float over UART.
void transmitFloat(float f) {
    // Transmit the 32 bit float as two 16 bit ints.
    long n = *(long*)&f;
    int lsb = n & 0xFFFF;
    transmitInt(lsb);
    int msb = (n & 0xFFFF0000) >> 16;
    transmitInt(msb);
}

float mostRecentPowerValue = 0.0;

// setMotorVoltagePercent sets the motor voltage and direction. 0 is no voltage and 1
// is the max voltage. Negative values reverse the direction of the motor (using an h-bridge).
// PWM is used to drive the motor.
void setMotorVoltagePercent(float p) {
    // Save the most recent value to a global variable so it can be transmitted over UART.
    mostRecentPowerValue = p;
    if (p == 0) {
        PWM_PORT_DIR &= ~BIT_DC_MOTOR_PWM_POS;
        PWM_PORT_DIR &= ~BIT_DC_MOTOR_PWM_NEG;
        return;
    }
    // Note: The MSP-430 does not have a FPU, so these floating point operations are not very efficient.
    // Enable/Disable the two PWM signals to set the direction of the DC motor.
    if (p < 0) {
        p *= -1;
        PWM_PORT_DIR |= BIT_DC_MOTOR_PWM_NEG;
        PWM_PORT_DIR &= ~BIT_DC_MOTOR_PWM_POS;
    } else {
        PWM_PORT_DIR &= ~BIT_DC_MOTOR_PWM_NEG;
        PWM_PORT_DIR |= BIT_DC_MOTOR_PWM_POS;
    }

    // make sure 0 < p < 1;
    if (p > 1) {p = 1;}
    if (p < 0) {p = 0;}

    int duty_cycle_ticks = (PWM_TICK_PERIOD-1) * p;
    TA1CCR1 = duty_cycle_ticks;
}

void setupMotorControl(){
    // Use PWM to drive the motor.
    TA1CTL |= TACLR;
    TA1CTL |= TASSEL_2;          // Use SMCLK
    TA1CTL |= MC_1;              // Count up to TA2CCR0
    TA1CCR0 = PWM_TICK_PERIOD;   // TA2CCR0 is the PWM period

    PWM_PORT_SEL |= BIT_DC_MOTOR_PWM_POS;
    PWM_PORT_SEL |= BIT_DC_MOTOR_PWM_NEG;

    // The DIR field will enable/disable these pins for PWM. To start, neither pin should be enabled.
    PWM_PORT_DIR &= ~BIT_DC_MOTOR_PWM_POS;
    PWM_PORT_DIR &= ~BIT_DC_MOTOR_PWM_NEG;

    TA1CCTL1 = OUTMOD_7;
    setMotorVoltagePercent(0);
}

void setupCommunication() {
    /*
     * The recommended USCI initialization or reconfiguration process is:
     * 1. Set UCSWRST ( BIS.B #UCSWRST,&UCAxCTL1 )
     * 2. Initialize all USCI registers with UCSWRST = 1 (including UCAxCTL1)
     * 3. Configure ports.
     * 4. Clear UCSWRST via software ( BIC.B #UCSWRST,&UCAxCTL1 )
     * 5. Enable interrupts (optional) via UCAxRXIE and/or UCAxTXIE
     */

    // Enable SW Reset
    UCA0CTL1 |= UCSWRST;

    /*
     * A line plucked from Table 15-4 "Commonly Used Baud Rates, Settings, and Errors, UCOS16 = 0":
     *
     * BRCLK Frequency[Hz]    Baud Rate[Baud] UCBRx UCBRSx UCBRFx Maximum TX Error [%] Maximum RX Error [%]
     * 1,048,576              115200          9     1      0      -1.1 to 10.7         -11.5 to 11.3
     */
    UCA0CTL1 |= UCSSEL_2; // Use SMCLK
    UCA0BR0 = 9;
    UCA0BR1 = 0;

    /*
     * UCAxMCTL layout:
     * | 7 6 5 4 | 3 2 1 | 0      |
     * | UCBRF   | UCBRS | UCSO16 |
     *
     * UCBRF should be 0, UCBRS should be 1 and UCSO16 should be 0:
     * 0 0 0 0 | 0 0 1 | 0
     */
    //
    UCA0MCTL = 2;


    // Configure Ports
    P1SEL |= BIT2;
    P1SEL2 |= BIT2;


    // Clear SW Reset
    UCA0CTL1 &= ~UCSWRST;

    //IE2 |= UCA0TXIE;

}

// accMid is the target acceleration value.
int accMid = 0xFFFF/2;
// useNextAccAsZeroG is a flag that is set when the button is pressed. It tells us that the nexct recorded
// acceleration value should be our target acceleration.
int useNextAccAsZeroG = 0;
int mostRecentAccValue = 0;


// displayAcc turns on or off the green LED depending on if our current acceleration is above or below the
// target value.
void displayAcc(int value) {
    if (useNextAccAsZeroG){
        useNextAccAsZeroG = 0;
        accMid = value;
    }

    if (value < accMid) {
        P1OUT |= BIT0;
    } else{
        P1OUT &= ~BIT0;
    }
}


// Parameters for the PID controller
float kp = -.4;
float kd = 0;
float ki = .3;

int prev_error = 0;

// restoreOrientation is the main PID control calculation.
void restoreOrientation(int currAcc) {
    int error = accMid - currAcc;

    float d_error = error - prev_error;

    float i_error = 0;

    float output = kp * error + kd * d_error + ki * i_error;
    setMotorVoltagePercent(output);
    prev_error = error;
}

int isInitialGravitySet = 0;

// transmit int sends the given 16 bit integer over UART
void transmitInt(int n) {
    short lsb = n & 0x00FF;
    short msb = (n & 0xFF00) >> 8;

    UCA0TXBUF = lsb;
    while(!(IFG2 & UCA0TXIFG)){}
    UCA0TXBUF = msb;
    while(!(IFG2 & UCA0TXIFG)){}
}


int sentinelValue = 0xBEEF;
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

	// Setup LEDs and buttons
	setupIO();

	// Setup analog reading
	setupAccelerometerAnalogInput();

	// Setup motor control
	setupMotorControl();

	// Setup outgoing communication via UART
	setupCommunication();

	__enable_interrupt();

	// Initiate the first ADC capture. Subsequent captures will happen automatically.
	ADC10CTL0 |= ADC10SC;
	
	float currAcc = 0;
	while(1){

	    int currAcc = mostRecentAccValue;
	    displayAcc(currAcc);
	    restoreOrientation(currAcc);

	    transmitInt(sentinelValue);
	    transmitInt(currAcc);
	    transmitFloat(mostRecentPowerValue);
	}

	return 0;
}

// Interrupt vector for ADC conversion
#pragma vector = ADC10_VECTOR
__interrupt void ADC10_Data_Ready(void) {
    ADC10CTL0 &= ~ADC10IFG;
    mostRecentAccValue = ADC10MEM;
}

// Interrupt vector for a button press
#pragma vector = PORT1_VECTOR
__interrupt void Btn_Reset_Gravity(void) {
    P1IFG &= ~BIT_G_RESET;
    isInitialGravitySet = 1;
    useNextAccAsZeroG = 1;
}

