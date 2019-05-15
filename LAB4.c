/*******************************
 * Name: Eddie Molina
 * Student ID#: 1001088363
 * Lab Day: 10/18/2017
 * CSE 3442/5442 - Embedded Systems 1
 * Lab 4: Analog to Digital Conversion (ADC)
 ********************************/

#include <pic18f452.h> //For the XC8 Compiler
//#include <p18f452.h> //For the C18 Compiler
#include <delays.h>

// PIC18F452 Configuration Bit Settings
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config OSCS = OFF       // Oscillator System Clock Switch Enable bit (Oscillator system clock switch option is disabled (main oscillator is source))
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bit (Brown-out Reset disabled)
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config CCP2MUX = ON     // CCP2 Mux bit (CCP2 input/output is multiplexed with RC1)
#pragma config STVR = ON        // Stack Full/Underflow Reset Enable bit (Stack Full/Underflow will cause RESET)
#pragma config LVP = OFF        // Low Voltage ICSP Enable bit (Low Voltage ICSP disabled)


char COUNT;             // Counter available as local to functions
char RPGCNT;            // Used to display RPG changes


/*******************************
 * Strings for LCD Initialization and use
 * For stability reasons, each array must have 10 total elements (no less, no more)
 ********************************/
const char LCDstr[]  = {0x33,0x32,0x28,0x01,0x0c,0x06,0x00,0x00}; // LCD Initialization string (do not change)

//Never change element [0] or [9] of these char arrays
//You may only change the middle 8 elements for displaying on the LCD
char Str_1[] = {0x80,' ',' ',' ',' ',' ',' ',' ',' ',0};    // First line of LCD
char Str_2[] = {0xC0,' ',' ',' ',' ',' ',' ',' ',' ',0};   // Second line of LCD

const char Clear1[] = {0x80,' ',' ',' ',' ',' ',' ',' ',' ',0};  // Clear first line of LCD
const char Clear2[] = {0xC0,' ',' ',' ',' ',' ',' ',' ',' ',0};  // Clear second line of LCD


/*******************************
 * Function prototypes
 ********************************/
void Initial(void);
void InitLCD(void);
void DisplayC(const char *);
void Delay();
long tenK();
long pot1();

void main()
{
    Initial(); 	  	  //Initialize all settings required for general QwikFlash and LCD operation
	DisplayC(Clear1); //Clear the LCD one time at the beginning of your program  
	DisplayC(Clear2);
    long retTen;        // initialize variable for 10K ohm potentiometer
    long retPot;        // initialize variable for Potentiometer 1 on circuit board
    
    
  
    //Your personal PORT/TRIS/ADCON/etc settings or configurations can go here 
    //Or make your own function and call it            
    while(1)
    {
        Delay10KTCYx(25);
        retTen = tenK();        // Store value for 10K ohm potentiometer
        Delay10KTCYx(25);
        retPot = pot1();        // Store value for Potentiometer 1 on circuit board
        
        if (retTen > retPot)        //Turns LED D6 on
        {
            PORTAbits.RA3 = 1;
            PORTAbits.RA1 = 0;
            PORTAbits.RA2 = 0;
            
        }
        else if (retTen < retPot)   //Turns LED D4 on
        {
            PORTAbits.RA1 = 1;
            PORTAbits.RA3 = 0;
            PORTAbits.RA2 = 0;
        }
        else                        //Turns LED D5 on
        {
            PORTAbits.RA2 = 1;
            PORTAbits.RA3 = 0;
            PORTAbits.RA1 = 0;
        }
        

    }
}

/*******************************
 * Initial()
 *
 * This function performs all initializations of variables and registers.
 *
 * DO NOT CHANGE ANYTHING IN THIS FUNCTION
 ********************************/
void Initial()
{
        ADCON1 = 0b10001110;            // Enable PORTA & PORTE digital I/O pins
        TRISA  = 0b11100001;            // Set I/O for PORTA
        TRISB  = 0b11011100;            // Set I/O for PORTB
        TRISC  = 0b11010000;            // Set I/0 for PORTC
        TRISD  = 0b00001111;            // Set I/O for PORTD
        TRISE  = 0b00000000;            // Set I/O for PORTE
        PORTA  = 0b00010000;            // Turn off all four LEDs driven from PORTA
        RPGCNT   = 0;                   // Clear counter to be displayed        
        InitLCD();                      // Initialize LCD
}

/*******************************
 * InitLCD()
 *
 * Initialize the Optrex 8x2 character LCD.
 * First wait for 0.1 second, to get past display's power-on reset time.
 *
 * DO NOT CHANGE ANYTHING IN THIS FUNCTION
 *******************************/
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

        PORTEbits.RE0 = 0;              // RS=0 for command
        tempPtr = LCDstr;

        while (*tempPtr)                // if the byte is not zero
        {
          currentChar = *tempPtr;
          PORTEbits.RE1 = 1;            // Drive E pin high
          PORTD = currentChar;          // Send upper nibble
          PORTEbits.RE1 = 0;            // Drive E pin low so LCD will accept nibble          
          Delay();	
	      currentChar <<= 4;            // Shift lower nibble to upper nibble
          PORTEbits.RE1 = 1;            // Drive E pin high again
          PORTD = currentChar;          // Write lower nibble
          PORTEbits.RE1 = 0;            // Drive E pin low so LCD will process byte        
          Delay();	
	      tempPtr++;                    // Increment pointerto next character
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
        PORTEbits.RE0 = 0;              // Drive RS pin low for cursor-positioning code

        while (*tempPtr)                // if the byte is not zero
        {
          currentChar = *tempPtr;
          PORTEbits.RE1 = 1;            // Drive E pin high
          PORTD = currentChar;          // Send upper nibble
          PORTEbits.RE1 = 0;            // Drive E pin low so LCD will accept nibble
          currentChar <<= 4;            // Shift lower nibble to upper nibble
          PORTEbits.RE1 = 1;            // Drive E pin high again
          PORTD = currentChar;          // Write lower nibble
          PORTEbits.RE1 = 0;            // Drive E pin low so LCD will process byte
	      Delay();		
          PORTEbits.RE0 = 1;            // Drive RS pin high for displayable characters
          tempPtr++;                    // Increment pointerto next character
        }
}

/******************************************************************************
	int Delay()
	This function is called to create a 1ms delay multiplied 
	by the integer given to it to make a specific delay time
******************************************************************************/
void Delay()
{   
	int delay = 2;
    int index = 0;

	for( index = 0; index < delay; index++ )
	{
		Delay100TCYx(25);		// each delay cost .001 of a second (1ms)
	}
	
	
}
/******************************************************************************
 *	int tenK()
 *	This function is for the 10 Kohm potentiometer and it converts the value 
 *  read from the potentiometer, which is analog, and turns it into two digital 
 *  values. From there it is displayed onto LCD of the QwikFlash.
******************************************************************************/
long tenK()
{
    long adLow = ADRESL;
    long adHigh = ADRESH;
    TRISEbits.TRISE2 = 1; // Set E2 as an input port.
    ADCON0 = 0b00111001;
    ADCON1 = 0b11000000;
    
    Delay10TCYx(4);
    ADCON0bits.GO = 1;
    Delay10TCYx(4);
    
    adLow = ADRESL;
    adHigh = ADRESH;
   
    long addbit = (adHigh << 8) + adLow;// Shift adHighy to the left 8 bits
                                        // and add adLow bits.
    addbit = (addbit * 50) / 1023;      // Make addbit a 2 digit value.
    unsigned char fnum = addbit / 10;   // Store the first digit in fnum 
    unsigned char snum = addbit % 10;   // Store the second digit in snum
    
    Str_1[1] = '1';
    Str_1[2] = '0';
    Str_1[3] = 'K';
    Str_1[4] = ':';
    Str_1[5] = fnum + 48;   // Store the first digit in fnum
    Str_1[6] = '.';
    Str_1[7] = snum + 48;    // Store the second digit in snum
    Str_1[8] = ' ';
        
    ADCON1 = 0b10001110;
    DisplayC(Str_1);
    Delay(250);
    return addbit;
}
/******************************************************************************
 *	int pot1()
 *	This function is for the potentiometer that is fixed on the circuit board, 
 *  and it converts the value read from the potentiometer, which is analog, and
 *  turns it into two digital values. From there it is displayed onto LCD of the
 *  QwikFlash.
******************************************************************************/
long pot1()
{
    long adLow1;
    long adHigh1;
    ADCON0 = 0b00100001;
    ADCON1 = 0b11000000;
    
    Delay10TCYx(4);
    ADCON0bits.GO = 1;
    Delay10TCYx(4);
    adHigh1 = ADRESH;
    adLow1 = ADRESL;
    long addbit1 = (adHigh1 << 8) + adLow1; // Shift adHighy1 to the left 8 bits
                                            // and add adLow1 bits.
    addbit1 = (addbit1 * 50) / 1023;        // Make addbit1 a 2 digit value.
    unsigned char fnum1 = addbit1 / 10;     // Store the first digit in fnum1
    unsigned char snum1 = addbit1 % 10;     // Store the second digit in snum1
    
    Str_2[1] = 'P';
    Str_2[2] = 'O';
    Str_2[3] = 'T';
    Str_2[4] = ':';
    Str_2[5] = fnum1 + 48;      // Add 48 to get the ascii value
    Str_2[6] = '.';
    Str_2[7] = snum1 + 48;      // Add 48 to get the ascii value
    Str_2[8] = ' ';
    
    ADCON1 = 0b10001110;
    DisplayC(Str_2);
    Delay(250);
    
    return addbit1;
}