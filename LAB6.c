/*******************************
 * Name: Eddie Molina
 * Student ID#: 1001088363
 * Lab Day: Wednesday, November 15, 2017
 * CSE 3442/5442 - Embedded Systems 1
 * Lab 6: Timers and Interrupts
 ********************************/

#include <pic18f452.h> //For the XC8 Compiler
#include <xc.h>


#define _XTAL_FREQ 10000000

// PIC18F452 Configuration Bit Settings
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config OSCS = OFF       // Oscillator System Clock Switch Enable bit (Oscillator system clock switch option is disabled (main oscillator is source))
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bit (Brown-out Reset disabled)
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config CCP2MUX = ON     // CCP2 Mux bit (CCP2 input/output is multiplexed with RC1)
#pragma config STVR = ON        // Stack Full/Underflow Reset Enable bit (Stack Full/Underflow will cause RESET)
#pragma config LVP = OFF        // Low Voltage ICSP Enable bit (Low Voltage ICSP disabled)

char COUNT; // Counter available as local to functions

char RPGCNT; // Used to display RPG changes


/*******************************
 * Strings for LCD Initialization and use
 * For stability reasons, each array must have 10 total elements (no less, no more)
 ********************************/
const char LCDstr[] = {0x33, 0x32, 0x28, 0x01, 0x0c, 0x06, 0x00, 0x00}; // LCD Initialization string (do not change)

//Never change element [0] or [9] of these char arrays
//You may only change the middle 8 elements for displaying on the LCD
char Str_1[] = {0x80, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 0};           // First line of LCD
char Str_2[] = {0xC0, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 0};           // Second line of LCD

char pause1[] = {0x58, 0x9B, 0x9B, 0x9B, 0x9B, 0x9B, 0x9B, 0x9B, 0x9B, 0};  // create pause symbol
char pause2[] = {0xC0, 0x03, ' ',' ',' ',' ',' ',' ',' ',' ',0};            // place pause symbol in the beginning
                                                                            // of the second line of LCD

char play1[] = {0x60, 0x90, 0x98, 0x9C, 0x9E, 0x9C, 0x98, 0x90, 0x80, 0};   // create play button symbol
char play2[] = {0xC0, 0x04, ' ',' ',' ',' ',' ',' ',' ',' ',0};             // place the play symbol in the begining
                                                                            // of the second line of LCD

const char Clear1[] = {0x80, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 0};    // Clear first line of LCD
const char Clear2[] = {0xC0, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 0};    // Clear second line of LCD


/*******************************
 * Function prototypes
 ********************************/
void Initial(void);
void InitLCD(void);
void DisplayC(const char *);
void Delay();
void interrupt My_ISR_High(void);
void interrupt low_priority My_ISR_Low(void);
void time(int seconds, int minutes, int hours);
void timeInDay(void);

int seconds;        // global variable for time
int minutes;        // global variable for time
int hours;          // global variable for time
int counter = 0;    // counter variable used to toggle LEDs
int switch1 = 0;    // flag for switch 1


void main() 
{
    Initial(); //Initialize all settings required for general QwikFlash and LCD operation
    DisplayC(Clear1); //Clear the LCD one time at the beginning of your program  
    DisplayC(Clear2);

    //Your personal PORT/TRIS/ADCON/etc settings or configurations can go here 
    //Or make your own function and call it

    TRISBbits.RB0 = 1;          // set PortB0 as input port
    TRISBbits.RB1 = 1;          // set PortB1 as input port
    TRISBbits.RB2 = 1;          // set PortB2 as input port
    TRISBbits.RB3 = 1;          // set PortB3 as input port

    T0CON = 0b00000101;         // timer 0 configuration register bit settings
    INTCON = 0b00110000;        // interrupt configuration register bit settings
    INTCON2 = 0b11110000;       // interrupt configuration register 2 bit settings
    INTCON3 = 0b00001000;       // interrupt configuration register 3 bit settings
    RCON = 0b10000000;

    TMR0H = 0x67;               // Load preload value in timer 0 in high byte
    TMR0L = 0x69;               // Load preload value in timer 0 in low byte

    INTCONbits.PEIE = 1;        // Set PEIE enable bit
    INTCONbits.GIE = 1;         // GIE enable bit
    T0CONbits.TMR0ON = 1;       // turn timer 0 on


    while (1) 
    {
        // switch1 is a flag that is set from the low 
        // priority interrupt
        if (switch1)
        {
            DisplayC(pause1);   // display the pause symbol
            Str_2[1] = 0x03;
        }
        else
        {
            DisplayC(play1);    // display the play symbol
            Str_2[1] = 0x04;
        }

        // if switch 1, switch 2, and pushbutton 1 are set
        // then increment timer
        if (switch1 == 1 && PORTBbits.RB2 == 1 && PORTBbits.RB3 == 1)
        {
            Str_2[7] = '+';     // display ++ for increment
            Str_2[8] = '+';
            timeInDay();     
        }
        else
        {
            Str_2[7] = ' ';     // clear ++ for increment
            Str_2[8] = ' ';
        }
        DisplayC(Str_2);
    }
}

/************************************************************************
 * Initial()
 *
 * This function performs all initializations of variables and registers.
 *
 * DO NOT CHANGE ANYTHING IN THIS FUNCTION
 ************************************************************************/
void Initial() 
{
    ADCON1 = 0b10001110; // Enable PORTA & PORTE digital I/O pins
    TRISA = 0b11100001; // Set I/O for PORTA
    TRISB = 0b11011100; // Set I/O for PORTB
    TRISC = 0b11010000; // Set I/0 for PORTC
    TRISD = 0b00001111; // Set I/O for PORTD
    TRISE = 0b00000000; // Set I/O for PORTE
    PORTA = 0b00010000; // Turn off all four LEDs driven from PORTA
    RPGCNT = 0; // Clear counter to be displayed        
    InitLCD(); // Initialize LCD
}

/************************************************************************
 * InitLCD()
 *
 * Initialize the Optrex 8x2 character LCD.
 * First wait for 0.1 second, to get past display's power-on reset time.
 *
 * DO NOT CHANGE ANYTHING IN THIS FUNCTION
 ************************************************************************/
void InitLCD() 
{
    char currentChar;
    char *tempPtr;
    COUNT = 10;

    while (COUNT) 
    {
        Delay();
        COUNT--;
    }

    PORTEbits.RE0 = 0; // RS=0 for command
    tempPtr = LCDstr;

    while (*tempPtr) // if the byte is not zero
    {
        currentChar = *tempPtr;
        PORTEbits.RE1 = 1; // Drive E pin high
        PORTD = currentChar; // Send upper nibble
        PORTEbits.RE1 = 0; // Drive E pin low so LCD will accept nibble          
        Delay();
        currentChar <<= 4; // Shift lower nibble to upper nibble
        PORTEbits.RE1 = 1; // Drive E pin high again
        PORTD = currentChar; // Write lower nibble
        PORTEbits.RE1 = 0; // Drive E pin low so LCD will process byte        
        Delay();
        tempPtr++; // Increment pointerto next character
    }
}

/*******************************
 * DisplayC(const char *) 
 *
 * This function is called with the passing in of an array of a constant
 * display string.  It sends the bytes of the string to the LCD.  The first
 * byte sets the cursor position.  The remaining bytes are displayed, beginning
 * at that position.
 * This function expects a normal one-byte cursor-positioning code, 0xhh, or
 * an occasionally used two-byte cursor-positioning code of the form 0x00hh.
 *
 * DO NOT CHANGE ANYTHING IN THIS FUNCTION
 ********************************/
void DisplayC(const char * tempPtr) 
{
    char currentChar;
    PORTEbits.RE0 = 0; // Drive RS pin low for cursor-positioning code

    while (*tempPtr) // if the byte is not zero
    {
        currentChar = *tempPtr;
        PORTEbits.RE1 = 1; // Drive E pin high
        PORTD = currentChar; // Send upper nibble
        PORTEbits.RE1 = 0; // Drive E pin low so LCD will accept nibble
        currentChar <<= 4; // Shift lower nibble to upper nibble
        PORTEbits.RE1 = 1; // Drive E pin high again
        PORTD = currentChar; // Write lower nibble
        PORTEbits.RE1 = 0; // Drive E pin low so LCD will process byte
        Delay();
        PORTEbits.RE0 = 1; // Drive RS pin high for displayable characters
        tempPtr++; // Increment pointerto next character
    }
}

/******************************************************************************
 *  int Delay()
 *  This function is called to create a 1ms delay multiplied 
 *  by the integer given to it to make a specific delay time
 ******************************************************************************/
void Delay() 
{
    int delay = 2;
    int index = 0;

    for (index = 0; index < delay; index++) 
    {
        __delay_ms(1); // each delay cost .001 of a second (1ms)
    }
}


/******************************************************************************
 *   void interrupt My_ISR_High(void) 
 *   This function is used for high priority interrupts. Push button 1 is set
 *   as a high priority interrupt. If push button 1 is pressed, the time is reset
 *   to 00:00:00 (hours:minutes:seconds) and then displayed onto the LCD. Push
 *   button 1 take precedence over any other inturrupts. This function also
 *   toggles the LEDs on the QwikFlash board every time push button 1 is pressed.
 ******************************************************************************/

void interrupt My_ISR_High(void) 
{
    //interrupt handling for HIGH
    if (INT0IF == 1 && INT0IE == 1) 
    {
        seconds = 0;
        minutes = 0;
        hours = 0;
        Str_1[1] = '0';
        Str_1[2] = '0';
        Str_1[3] = ':';
        Str_1[4] = '0';
        Str_1[5] = '0';
        Str_1[6] = ':';
        Str_1[7] = '0';
        Str_1[8] = '0';
        DisplayC(Str_1);
        INT0IF = 0;    // reset interrupt flag
        
        // toggle LEDs on QwikFlash Board
        if (counter % 2 == 0)
        {
            PORTAbits.RA3 = 1;
            PORTAbits.RA2 = 0;
            PORTAbits.RA1 = 1;
        }
        else
        {
            PORTAbits.RA3 = 0;
            PORTAbits.RA2 = 1;
            PORTAbits.RA1 = 0;
        }
        counter++;

    }

    
}

/******************************************************************************
 * void interrupt low_priority My_ISR_Low()
 * This function is used for low priority interrupts. Switch 1 on the IDL is
 * a low priority interrupt. If switch 1 is set an interrupt signal is sent
 * causing the timer to be paused. Once switch 1 goes back to 0 the time 
 * is resumed and continues off where it left off.
 ******************************************************************************/
void interrupt low_priority My_ISR_Low(void) 
{
    // if timer overflow flag is set
    // increment 1 second
    if (TMR0IF == 1 && TMR0IE == 1) 
    {
        TMR0IF = 0;     // reset timer overflow flag
        TMR0H = 0x67;   // preload value for 1 second
        TMR0L = 0x69;   // preload value for 1 second
        timeInDay();    // call function and increment time by 1 second
    }

    // if switch 1 is set then puse time
    if (INT1IF == 1 && INT1IE == 1) 
    {
        if (INTCON2bits.INTEDG1 == 1 && TMR0ON == 1)
        {
            TMR0ON = 0;                 // pause time
            INTCON2bits.INTEDG1 = 0;    // set timer positive edge to 0
            switch1 = 1;                // set flag to be used in main function
            
            
        }
        else if (INTCON2bits.INTEDG1 == 0 && TMR0ON == 0)
        {
            TMR0ON = 1;                 // resume time
            INTCON2bits.INTEDG1 = 1;    // set timer positive edge to 1
            switch1 = 0;                // reset flag to be used in main function
        }
        INT1IF = 0; // reset interrupt flag
    }
    
}

/******************************************************************************
 *   void timeInDay()
 *   This function is used to increment the time from 0 seconds all the way to
 *   24 hours. Then it resets after a full 24 hours. This function also displays
 *   the time onto the LCD.
 ******************************************************************************/
void timeInDay(void) 
{
    seconds++;
    if (seconds > 59) 
    {
        seconds = 0;
        minutes++;
    }

    if (minutes > 59) 
    {
        minutes = 0;
        hours++;
    }

    if (hours > 23) 
    {
        hours = 0;
    }

    int num1 = seconds / 10;
    int num2 = seconds % 10;
    int num3 = minutes / 10;
    int num4 = minutes % 10;
    int num5 = hours / 10;
    int num6 = hours % 10;
    Str_1[1] = num5 + 48;
    Str_1[2] = num6 + 48;
    Str_1[3] = ':';
    Str_1[4] = num3 + 48;
    Str_1[5] = num4 + 48;
    Str_1[6] = ':';
    Str_1[7] = num1 + 48;
    Str_1[8] = num2 + 48;
    DisplayC(Str_1);
}