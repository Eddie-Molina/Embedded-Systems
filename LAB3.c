/*******************************
 * Name: Eddie Molina
 * Student ID#: 1001088363
 * Lab Day: 10/11/17
 * CSE 3442/5442 - Embedded Systems 1
 * Lab 3: Simple Input/Output Circuit
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
char Str_2[] = {0xC0,' ',' ',' ',' ',' ',' ',' ',' ',0};    // Second line of LCD

const char Clear1[] = {0x80,' ',' ',' ',' ',' ',' ',' ',' ',0};  // Clear first line of LCD
const char Clear2[] = {0xC0,' ',' ',' ',' ',' ',' ',' ',' ',0};  // Clear second line of LCD


/*******************************
 * Function prototypes
 ********************************/
void Initial(void);
void InitLCD(void);
void DisplayC(const char *);
void Delay();
void add(unsigned char x, unsigned char y);
void sub(unsigned char x, unsigned char y);
void and(unsigned char x, unsigned char y);
void not(unsigned char x, unsigned char y);
void mult(unsigned char x, unsigned char y);

void main()
{
    Initial(); 	  	  //Initialize all settings required for general QwikFlash and LCD operation
	DisplayC(Clear1); //Clear the LCD one time at the beginning of your program  
	DisplayC(Clear2);
    
    TRISB = 0b00111111; // make portB an input port
    TRISC = 0b00001111; // make portC an input port
    
	//Your personal PORT/TRIS/ADCON/etc settings or configurations can go here 
	//Or make your own function and call it
    while(1)
    {
        
        unsigned char pB = PORTB;
        unsigned char pC = PORTC;
        unsigned char mask = 0b00001111;
        pB = pB & mask;         // mask portB off
        pC = pC & mask;         // mask portC off
          
        if((PORTBbits.RB4 == 0) && (PORTBbits.RB5 == 0))    // for add operation
        {                                                   // or multiply operation
            add(pB, pC);
            //mult(pB, pC);
        }
    
        if((PORTBbits.RB4 == 0) && (PORTBbits.RB5 == 1))    // for subtract operation
        {
            sub(pB, pC);    
        }
        if((PORTBbits.RB4 == 1) && (PORTBbits.RB5 == 0))    // for AND operation
        {
            and(pB, pC);
        }
        
        if((PORTBbits.RB4 == 1) && (PORTBbits.RB5 == 1))    // for NOT operation
        {
            not(pB, pC);        
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
 * void add()
 * This function adds the bits from portB and portC resulting in integer
 * values which are converted to ascii, then are displayed on the LCD
******************************************************************************/
void add(unsigned char x, unsigned char y)
{
    unsigned char fnum;
    unsigned char snum;
    unsigned char add;
    add = x + y;            // adds portB and portC
    fnum = add / 10;        // save the first digit in variable fnum
    snum = add % 10;        // save the second digit in variable snum
    Str_1[1] = ' ';
    Str_1[2] = ' ';
    Str_1[3] = 'B';
    Str_1[4] = '+';
    Str_1[5] = 'C';
    Str_1[6] = '=';
    Str_2[5] = 48 + fnum;   // offset fnum by 48 in order 
                            // to display equivalent digit from ascii table
    Str_2[6] = 48 + snum;   // offset snum by 48 in order 
                            // to display equivalent digit from ascii table
    Str_2[7] = ' ';
    Str_2[8] = ' ';
    DisplayC(Str_1);        // display characters in first row of LCD
    DisplayC(Str_2);        // display characters in second row of LCD

}
/******************************************************************************
 * void sub()
 * This function subtracts the bits in portC from portB resulting in integer
 * values, which are converted to ascii, then are displayed on the LCD.
******************************************************************************/
void sub(unsigned char x, unsigned char y)
{
    unsigned char fnum;
    unsigned char snum;
    int sub;
    sub = x - y;                // subtract portC from portB
    if (sub >= 0)               // if statement for positive numbers
    {
        fnum = sub / 10;        // save the first digit in variable fnum
        snum = sub % 10;        // save the second digit in variable snum
        Str_1[1] = ' ';
        Str_1[2] = ' ';
        Str_1[3] = 'B';
        Str_1[4] = '-';
        Str_1[5] = 'C';
        Str_1[6] = '=';
        Str_1[7] = ' ';
        Str_1[8] = ' ';
        Str_2[4] = '+';
        Str_2[5] = 48 + fnum;   // offset fnum by 48 in order 
                                // to display equivalent digit from ascii table
        Str_2[6] = 48 + snum;   // offset snum by 48 in order 
                                // to display equivalent digit from ascii table
        Str_2[7] = ' ';
        Str_2[8] = ' ';
        DisplayC(Str_1);        // display characters in first row of LCD
        DisplayC(Str_2);        // display characters in second row of LCD

    }
    else                        // else for negative numbers
    {
        sub = sub * -1;         // convert sub to a negative number
        fnum = sub / 10;        // save the first digit in variable fnum    
        snum = sub % 10;        // save the second digit in variable snum   
        Str_1[1] = ' ';
        Str_1[2] = ' ';
        Str_1[3] = 'B';
        Str_1[4] = '-';
        Str_1[5] = 'C';
        Str_1[6] = '=';
        Str_1[7] = ' ';
        Str_1[8] = ' ';
        Str_2[4] = '-';
        Str_2[5] = 48 + fnum;   // offset fnum by 48 in order 
                                // to display equivalent digit from ascii table
        Str_2[6] = 48 + snum;   // offset snum by 48 in order 
                                // to display equivalent digit from ascii table
        Str_2[7] = ' ';
        Str_2[8] = ' ';
        DisplayC(Str_1);        // display characters in first row of LCD
        DisplayC(Str_2);        // display characters in second row of LCD
    }
}
/******************************************************************************
 * void and()
 * This function performs the AND operation with the bits from portB and portC,
 * and displays the ascii characters 1 or 0 onto the LCD.
******************************************************************************/
void and(unsigned char x, unsigned char y)
{
    Str_1[1] = ' ';
    Str_1[2] = ' ';
    Str_1[3] = 'B';
    Str_1[4] = '&';
    Str_1[5] = 'C';
    Str_1[6] = '=';
    Str_1[7] = ' ';
    Str_1[8] = ' ';
    Str_2[8] = (PORTBbits.RB0 & PORTCbits.RC0) + 48;  // AND operation for bits RB0 and RC0  
    Str_2[7] = (PORTBbits.RB1 & PORTCbits.RC1) + 48;  // AND operation for bits RB1 and RC1 
    Str_2[6] = (PORTBbits.RB2 & PORTCbits.RC2) + 48;  // AND operation for bits RB2 and RC2 
    Str_2[5] = (PORTBbits.RB3 & PORTCbits.RC3) + 48;  // AND operation for bits RB3 and RC3 
    Str_2[4] = ' ';
    Str_2[3] = ' ';
    Str_2[2] = ' ';
    DisplayC(Str_1);    // display characters in first row of LCD
    DisplayC(Str_2);    // display characters in second row of LCD
    
}
/******************************************************************************
 * void not()
 * This function executes the NOT operation for the bits in portB, then displays
 * the ascii characters 1 or 0 onto the LCD.
******************************************************************************/
void not(unsigned char x, unsigned char y)
{
    Str_1[1] = ' ';
    Str_1[2] = 'N';
    Str_1[3] = 'O';
    Str_1[4] = 'T';
    Str_1[5] = '(';
    Str_1[6] = 'B';
    Str_1[7] = ')';
    Str_1[8] = ' ';
    Str_2[8] = (!PORTBbits.RB0) + 48;   //not operation for bit 0 in portB
    Str_2[7] = (!PORTBbits.RB1) + 48;   //not operation for bit 1 in portB
    Str_2[6] = (!PORTBbits.RB2) + 48;   //not operation for bit 2 in portB
    Str_2[5] = (!PORTBbits.RB3) + 48;   //not operation for bit 3 in portB
    Str_2[4] = ' ';
    Str_2[3] = ' ';
    Str_2[2] = ' ';
    DisplayC(Str_1);                  // display characters in first row of LCD
    DisplayC(Str_2);                  // display characters in second row of LCD
}
/******************************************************************************
 * void mult()
 * This function multiplies the bits from portB and portC, resulting in integer
 * values, which are converted to ascii, then are displayed on the LCD.	
******************************************************************************/
void mult(unsigned char x, unsigned char y)
{
    unsigned char fnum;
    unsigned char snum;
    unsigned char tnum;
    unsigned char prod;
    prod = x * y;                // multiply bits in portB and portC
    fnum = prod / 100;           // save the first digit in variable fnum
    snum = prod / 10;            // make prod a 2 digit number      
    snum = snum % 10;            // save the second digit in variable snum
    tnum = prod % 10;            // save the third digit in variable tnum
    Str_1[1] = ' ';
    Str_1[2] = ' ';
    Str_1[3] = 'B';
    Str_1[4] = '*';
    Str_1[5] = 'C';
    Str_1[6] = '=';
    Str_2[2] = ' ';
    Str_2[3] = ' ';
    Str_2[4] = ' ';
    Str_2[5] = 48 + fnum;       // offset fnum by 48 in order 
                                // to display equivalent digit from ascii table
    Str_2[6] = 48 + snum;       // offset snum by 48 in order 
                                // to display equivalent digit from ascii table
    Str_2[7] = 48 + tnum;;      // offset tnum by 48 in order 
                                // to display equivalent digit from ascii table
    Str_2[8] = ' ';
    DisplayC(Str_1);        // display characters in first row of LCD
    DisplayC(Str_2);        // display characters in second row of LCD
    
}
