/*******************************
 * Name: Eddie Molina
 * Student ID#: 1001088363
 * Lab Day: Wednesday
 * CSE 3442/5442 - Embedded Systems 1
 * Lab 7 (ABET): Building a PIC18F4520 Standalone Alarm System with EUSART Communication 
 ********************************/

 /**
 NOTE: 	
	*Your comments need to be extremely detailed and as professional as possible
	*Your code structure must be well-organized and efficient
	*Use meaningful naming conventions for variables, functions, etc.
	*Your code must be cleaned upon submission (no commented out sections of old instructions not used in the final system)
	*Your comments and structure need to be detailed enough so that someone with a basic 
            embedded programming background could take this file and know exactly what is going on
 **/
 
#include <p18F4520.h>
//#include <pic18.h>
#include <xc.h> 
#include <stdio.h>
#include <stdlib.h>


//other includes
// PIC18F4520 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = HS    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//Configuration bits settings code goes here

// Defining _XTAL_FREQ and selecting the XC8 compiler allows you to use the delay functions __delay_ms(x) and __delay_us(x)
//#define _XTAL_FREQ 8000000 //If using the Internal Oscillator of 8 MHz
#define _XTAL_FREQ 20000000 //If using the External Crystal of 20 MHz

/**********************************************
 *          FUNCTION PROTOTYPES               *
 * ********************************************/
void interrupt low_priority My_ISR_Low(void);
void interrupt My_ISR_High(void);
double TempSensor(void);
void IRSensor(void);
char keyPadButtons(void);
void keyPad(void);
void menu(void);
void status(void);
void passCodeOptions(void);
void PIRSensorOptions(void);
void tempSensorOptions(void);
void enterKey(void);
void options(void);
void changePassCode(void);
void getPassCode(void);
void passCode(void);
void comparisonPassCode(void);
void putch(char c);
void checkCode(void);
void disableKeyboard(void);
void disableKeypad(void);

/**********************************************
 *              GLOBAL VARIABLES              *
 * ********************************************/
char passWord[4];
char passWord1[4];
char tryPassWord[4];
char newPassWord[4];
double temperature = 0;
char setTemp1 = 0;
char setTemp2 = 0;
char setTemp3 = 0;
int setTemp = 0;
double voltage = 0;
double cTemp = 0;
double fTemp = 0;
int i = 0;
int k = 0;
int j = 0;
int count = 0;
char input;
char enter;
char PIRflag = 0;
char tempFlag = 0;
char keyPadFlag = 0;


void main()
{
    // TRIS output settings  
    TRISAbits.RA1 = 0;  // yellow led
    TRISAbits.RA2 = 0;  // green led
    TRISAbits.RA3 = 0;  // red led
    TRISBbits.RB7 = 0;  // blue led
    TRISCbits.RC6 = 0;  // transmitter TX
    TRISBbits.RB3 = 0;  // keypad
    TRISBbits.RB4 = 0;  // keypad
    TRISBbits.RB5 = 0;  // keypad
    TRISBbits.RB6 = 0;  // keypad
    
    // TRIS input settings
    TRISAbits.RA0 = 1;  // temperature sensor
    TRISAbits.RA6 = 1;  // external oscillator
    TRISAbits.RA7 = 1;  // external oscillator  
    TRISDbits.RD4 = 1;  // keypad
    TRISDbits.RD5 = 1;  // keypad
    TRISDbits.RD6 = 1;  // keypad
    TRISDbits.RD7 = 1;  // keypad
    TRISBbits.RB0 = 1;  // motion detector
    TRISCbits.RC7 = 1;  // receiver RX
    TRISEbits.RE3 = 1;  // MCLR
    
    // Settings for interrupts
    PEIE   = 1;  // set peripheral interrupt enable bit
    GIE    = 1;  // set global interrupt enable bit
    RCONbits.IPEN     = 1;  // enable priority bits on interrupts
    
    // Settings for timer interrupt
    TMR0IE = 0;  // set timer 0 interrupt enable bit
    TMR0IF = 0;  // clear timer 0 interrupt flag bit (overflow)
    
    // Setting for External interrupt
    INT0IE = 1;  // set INT0 external interrupt enable bit
    INT0IF = 0;  // clear INT0 external interrupt flag bit
    
    // Setting for ADC interrupt
    PIE1bits.ADIE = 1;  // set A/D converter interrupt enable bit
    IPR1bits.ADIP = 0;  // A/D converter interrupt priority bit (low priority)
    PIR1bits.ADIF = 0;  // A/D converter interrupt flag bit(conversion not done)
    
    SPBRG = 31;
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 0;
    RCSTAbits.RX9 = 0;
    RCSTAbits.SPEN = 1;
    RCSTAbits.CREN = 1;
    TXSTAbits.TXEN = 1;
    
    ADCON0 = 0b00000001;    // ADCON0 configuration bits
    ADCON1 = 0b00001110;    // ADCON1 configuration bits
    ADCON2 = 0b10010100;
    TMR0H = 0x67;           // preload value for high byte
    TMR0L = 0x6A;           // prelaod value for low byte
    T0CON = 0b10000111;     // timer0 configuration bits
    PORTAbits.RA1 = 0;      // yellow led
    PORTAbits.RA2 = 0;      // green led
    PORTAbits.RA3 = 0;      // red led
    
    printf("\033[2J");
    printf("\033[0;0H");
    
    if (eeprom_read(0x00) == 0xFF)
    {
        printf("\rEnter the 4-digit passcode: ");  
        passCode();
    }
    
    if (eeprom_read(0x07) != 0xFF && eeprom_read(0x08) != 0xFF)
    {
        setTemp1 = eeprom_read(0x07);
        setTemp2 = eeprom_read(0x08);
        setTemp = (setTemp1 - 48)*10 + (setTemp2 - 48);
    }
    count = 0;
    while (count != 4 )
    {
        printf("\n\rType in password: ");
        checkCode();      
    }

	while(1)
	{       
        menu();
        options(); 
        
	}// end of while
	
} //end of void main()

/*******************************************************************************
 *  void TempSensor() function is for converting the output signal of the      *
 *  temperature sensor into degrees fahrenheit.                                *
 * *****************************************************************************/
double TempSensor(void)
{
    long addBits = 0;
    
    ADCON0bits.GO = 1;
    while(ADCON0bits.DONE == 1);
    long adLow = ADRESL;
    long adHigh = ADRESH;
    __delay_ms(250);
    
    adHigh = adHigh << 8;
    addBits = adHigh + adLow;
    voltage = (addBits*5) / 1023;
    voltage = voltage - 0.5;
    cTemp = (double)(voltage / 0.01);
    fTemp = (cTemp * (9/4)) + 32;
    fTemp *= -1;
    
    if (fTemp >= setTemp)
    {
        // set alarm
    }
    return fTemp;
}
/*******************************************************************************
 *  void interrupt My_ISR_High(void) function is for high priority interrupts. *
 *  Which in this case it is used for a IR motion detector.                    *
 * *****************************************************************************/
void interrupt My_ISR_High(void)
{
    // interrupt handling for HIGH PRIORITY
    if (INT0IF == 1 && INT0IE == 1)
    {
        PORTAbits.RA3 = 1;  // red led
        INT0IF = 0;
        if (PORTBbits.RB0 == 0)
        {
            INT0IF = 0; // reset interrupt flag
        }                   
    }
}
/*******************************************************************************
 *  void interrupt low_priority My_ISR_Low(void) function is for both timer    *
 *  interrupt and temperature sensor interrupt.                                *
 ******************************************************************************/
void interrupt low_priority My_ISR_Low(void) 
{
    // timer interrupt
    if (TMR0IF == 1 && TMR0IE == 1)
    {
        PORTAbits.RA1 ^= 1;      // yellow led
        TMR0H = 0x67;           // preload value for high byte
        TMR0L = 0x6A;           // prelaod value for low byte
        TempSensor();
        TMR0IF = 0; // reset timer overflow flag
    }
    if (ADIF == 1 && ADIE == 1)
    {
        // ADC conversion done
        // get result from ADRESH/L
        ADIF = 0; // clear flag
    }

}

/*******************************************************************************
 *  void keyPad() function is for using the keypad to make selections          *
 *  in the menu.                                                               *
 * *****************************************************************************/
void keyPad(void)
{
    while(PIR1bits.RCIF == 0);
    input = keyPadButtons();
    while(TXSTAbits.TRMT == 0);
    TXREG = RCREG;
    enterKey();
     
    if (input == '1')
    {
        passCodeOptions();
    }
    else if (input == '2')
    {
        PIRSensorOptions();
    }
    else if (input == '3')
    {
        tempSensorOptions();
    }
    else if (input == '4')
    {
        //use keyboard as only input
        keyPadFlag = '0';
        eeprom_write(0x06, keyPadFlag);
    }
    else if (input == '5')
    {
    }
    else if (input == '0')
    {
        // return to main menu     
    }
}

/*******************************************************************************
 *  void keyPadButtons() function is for returning a character once a button   *
 *  is pressed on the keypad.                                                  *
 * *****************************************************************************/
char keyPadButtons(void)
{
    for (i = 0; i < 4; i++)
    {
        PORTBbits.RB2 = 1;
        PORTBbits.RB3 = 0;
        PORTBbits.RB4 = 0;
        PORTBbits.RB5 = 0;
        if (PORTBbits.RB1 == 1)
        {
            return '1';
        }
        else if (PORTDbits.RD6 == 1)
        {
            return '2';
        }
        else if (PORTDbits.RD5 == 1)
        {
            return '3';
        }
        else if (PORTDbits.RD4 == 1)
        {           
            return 'a';
        }        
    }
    for (i = 0; i < 4; i++)
    {
        PORTBbits.RB2 = 0;
        PORTBbits.RB3 = 1;
        PORTBbits.RB4 = 0;
        PORTBbits.RB5 = 0;
        if (PORTBbits.RB1 == 1)
        {
            return '4';
        }
        else if (PORTDbits.RD6 == 1)
        {
            return '5';
        }
        else if (PORTDbits.RD5 == 1)
        {
            return '6';
        }
        else if (PORTDbits.RD4 == 1)
        {
            return 'b';
        }        
    }
    for (i = 0; i < 4; i++)
    {
        PORTBbits.RB2 = 0;
        PORTBbits.RB3 = 0;
        PORTBbits.RB4 = 1;
        PORTBbits.RB5 = 0;
        if (PORTBbits.RB1 == 1)
        {
            return '7';
        }
        else if (PORTDbits.RD6 == 1)
        {
            return '8';
        }
        else if (PORTDbits.RD5 == 1)
        {
            return '9';
        }
        else if (PORTDbits.RD4 == 1)
        {
            return 'c';
        }        
    }
    for (i = 0; i < 4; i++)
    {
        PORTBbits.RB2 = 0;
        PORTBbits.RB3 = 0;
        PORTBbits.RB4 = 0;
        PORTBbits.RB5 = 1;
        if (PORTBbits.RB1 == 1)
        {
            return '*';
        }
        else if (PORTDbits.RD6 == 1)
        {
            return '0';
        }
        else if (PORTDbits.RD5 == 1)
        {
            return '#';
        }
        else if (PORTDbits.RD4 == 1)
        {
            return 'd';
        }        
    }
    
    if (j == 4)
    {
        j = 0;
    }
    j++;    
}

/*******************************************************************************
 *  void status() function is for printing all of the statusses of the system  *
 *  at its current state.                                                      *
 * *****************************************************************************/
void status(void)
{
    printf("\033[2J");
    printf("\033[0;0H");
    printf("\n\r*************************************************************");
    printf("\n\r*                    Alarm System is Connected              *");
    printf("\n\r*               CSE 3442/5442 - Embedded System 1           *");
    printf("\n\r*         Lab 7 (ABET) - Standalone PIC with Communication  *");
    printf("\n\r*                       (Eddie Molina)                      *");
    printf("\n\r*************************************************************");
    printf("\n\n\rComponent Statuses");
    printf("\n\r-------------------------------------------------------------");
    printf("\n\rPIR Sensor Alarm State: ");
    PORTAbits.RA2 = 1;
    if (eeprom_read(0x05) == '1')
    {
        printf("\t\tACTIVE");
    }
    else
    {
        printf("\t\tINACTIVE");
    }
    printf("\n\rTemperature Alarm State: ");
    if (eeprom_read(0x04) == '1')
    {
        printf("\t\tACTIVE");
    }
    else
    {
        printf("\t\tINACTIVE");
    }
    printf("\n\rCurrent Temperature Reading: ");
    printf("\t\t%4.2f", fTemp);
    printf("\n\rTemperature Alarm Threshold: ");
    printf("\t\t%3d", setTemp);
    printf("\n\rCurrent Input Method: ");
    if (eeprom_read(0x06) == '1')
    {
        printf("\t\t\tKeypad");
    }
    else
    {
        printf("\t\t\tKeyboard");
    }
    printf("\n\n\r-------------------------------------------------------------");
}

/*******************************************************************************
 *  void menu() function is for displaying the menu onto the screen.           *
 * *****************************************************************************/
void menu(void)
{
    status();
    printf("\n\n\n\r--------------------------MAIN MENU--------------------------");
    printf("\n\n\rSelect One of the Following:");
    printf("\n\r\t1: Passcode Options");
    printf("\n\r\t2: PIR Sensor Alarm Options");
    printf("\n\r\t3: Temperature Sensor Alarm Options");
    printf("\n\r\t4: Use keyboard (terminal) as the only input");
    printf("\n\r\t5: Use keypad as the only input (A = Enter Key)");
    printf("\n\n\r\t0: Refresh Main Menu");
    printf("\n\n\rInput: ");
    //disableKeyboard();
}

/*******************************************************************************
 *  void options() function is for selecting an option from the menu and then  *
 *  proceed to the option selected.                                            *
 * *****************************************************************************/
void options(void)
{    
    while(PIR1bits.RCIF == 0);
    input = RCREG;
    while (input <= 47 || input >= 54)
    {
        while(PIR1bits.RCIF == 0);
        input = RCREG;
    }
    while(TXSTAbits.TRMT == 0);
    TXREG = RCREG; 
    enterKey();
       
    if (input == '1')
    {
        passCodeOptions();
    }
    else if (input == '2')
    {
        PIRSensorOptions();
    }
    else if (input == '3')
    {
        tempSensorOptions();
    }
    else if (input == '4')
    {
        //use keyboard as only input
        keyPadFlag = '0';
        eeprom_write(0x06, keyPadFlag);
    }
    else if (input == '5')
    {
        // us keypad as input
        keyPadFlag = '1';
        eeprom_write(0x06, keyPadFlag);
        menu();
        keyPad();
    }
    else if (input == '0')
    {
        // return to main menu     
    }   
}

/*******************************************************************************
 *  void passCodeOptions() function is for either selectino to change the      *
 *  passcode or returning to the main menu.                                    *
 * *****************************************************************************/
void passCodeOptions(void)
{
    printf("\033[2J");
    printf("\033[0;0H");
    status();
    printf("\n\n\n\r----------------------PASSCODE MENU--------------------------");
    printf("\n\n\rSelect one of the following:");
    printf("\n\r\t1: Change Passcode");
    printf("\n\n\r\t0: Return to Main Menu");
    printf("\n\rInput: ");
    while(PIR1bits.RCIF == 0);
    input = RCREG;
    while (input <= 47 || input >= 50)
    {
        while(PIR1bits.RCIF == 0);
        input = RCREG;
    }
    while(TXSTAbits.TRMT == 0);
    TXREG = RCREG; 
    enterKey();
    if(input == '1')
    {
        // change passcode
        changePassCode();
    }
    else if (input == '0')
    {
        //return to main menu       
    }
}

/*******************************************************************************
 *  void PIRSensorOptions() is used for enabling and disabling the PIR Sensor.  *
 * *****************************************************************************/
void PIRSensorOptions(void)
{
    status();
    printf("\n\n\n\r------------------PIR SENSOR ALARM MENU----------------------");
    printf("\n\n\rSelect one of the following:");
    printf("\n\r\t1: Enable PIR Sensor Alarm");
    printf("\n\r\t2: Disable PIR Sensor Alarm");
    printf("\n\n\r\t0: Return to Main Menu");
    printf("\n\rInput: ");
    while(PIR1bits.RCIF == 0);
    input = RCREG;
    while (input <= 47 || input >= 51)
    {
        while(PIR1bits.RCIF == 0);
        input = RCREG;
    }
    while(TXSTAbits.TRMT == 0);
    TXREG = RCREG; 
    enterKey();
    if(input == '1')
    {
        // enable PIR Senor alarm
        PIRflag = '1';
        eeprom_write(0x05, PIRflag);
    }
    else if (input == '2')
    {
        // disable PIR sensor alarm
        PIRflag = '0';
        eeprom_write(0x05, PIRflag);        
    }
    else if (input == '0')
    {
        // return to main menu
    }
}

/*******************************************************************************
 *  void tempSensorOptions() function is used for enabling and disabling the   *
 *  Temperature Sensor. It is also used for setting the temperature threshold. *
 * *****************************************************************************/
void tempSensorOptions(void)
{
    status();
    printf("\n\n\n\r--------------TEMPERATURE SENSOR ALARM MENU------------------");
    printf("\n\n\rSelect one of the following:");
    printf("\n\r\t1: Enable Temperature Sensor Alarm");
    printf("\n\r\t2: Disable Temperature Sensor Alarm");
    printf("\n\r\t3: Change Temperature Sensor Alarm Threshold");
    printf("\n\n\r\t0: Return to Main Menu");
    printf("\n\rInput: ");
    while(PIR1bits.RCIF == 0);
    input = RCREG;
    while (input <= 47 || input >= 52)
    {
        while(PIR1bits.RCIF == 0);
        input = RCREG;
    }
    while(TXSTAbits.TRMT == 0);
    TXREG = RCREG; 
    enterKey();
    if(input == '1')
    {
        // enable temperature sensor alarm
        tempFlag = '1';
        eeprom_write(0x04, tempFlag);
        TMR0IE = 1;  // set timer 0 interrupt enable bit
        TMR0IF = 0;  // clear timer 0 interrupt flag bit (overflow)
    }
    else if (input == '2')
    {
        // disable temperature sensor alarm
        tempFlag = '0';
        eeprom_write(0x04, tempFlag);
        TMR0IE = 0;  // set timer 0 interrupt enable bit
        TMR0IF = 0;  // clear timer 0 interrupt flag bit (overflow)
        
    }
    else if (input == '3')
    {
        //change temperature sensor alarm threshold
        printf("\n\rPress enter after 2 digits");
        printf("\n\rEnter Temperature (F): ");
        while(PIR1bits.RCIF == 0);
        setTemp1 = RCREG;
        while (input <= 47 || input >= 58)
        {
            while(PIR1bits.RCIF == 0);
            setTemp1 = RCREG;
        }
        while(TXSTAbits.TRMT == 0);
        TXREG = RCREG;
        eeprom_write(0x07, setTemp1);
        
        while(PIR1bits.RCIF == 0);
        setTemp2 = RCREG;
        while (input <= 47 || input >= 58)
        {
            while(PIR1bits.RCIF == 0);
            setTemp2 = RCREG;
        }
        while(TXSTAbits.TRMT == 0);
        TXREG = RCREG;
        eeprom_write(0x08, setTemp2);
        
        
        
        setTemp = (setTemp1 - 48)*10 + (setTemp2 - 48);
        
        enterKey();
        
        
        
        printf("\n\r--->NEW TEMPERATURE ACCEPTED<---");
        __delay_ms(1000);
        
    }
    else if (input == '0')
    {
        // return to main menu
    }
}


/*******************************************************************************
 *  void changePassCode() function is used for changing the passcode, only if  *
 *  the original passcode is entered first due to a security check.            *
 * *****************************************************************************/
void changePassCode(void)
{
    status();
    printf("\n\n\n\r----------------------PASSCODE MENU--------------------------");
    printf("\n\r--------------------[Change Passcode]------------------------");
    printf("\n\n\rEnter Current Passcode: ");
    comparisonPassCode();
    
    count = 0;
    for (k = 0; k < 4; k++)
    {
        if (passWord1[k] == eeprom_read(0x00 + k))
        {
            count++;
        }
        if (count == 4)
        {
            printf("\n\rEnter New Passcode: ");
            passCode();
            printf("\n\r--->NEW PASSCODE ACCEPTED<---");
            __delay_ms(2000);
            
        }
        if (k == 3 && count != 4)
        {
            count = 0;
            printf("\n\rWrong passcode, Please try again");
            __delay_ms(2000);
        }
    }    
}

/*******************************************************************************
 *  void passCode() is used for entering the original passcode and storing
 *  it in EEPROM.
 * *****************************************************************************/
void passCode(void)
{
    for (i = 0; i < 4; i++)
    {
        while(PIR1bits.RCIF == 0);
        passWord[i] = RCREG;
        while (passWord[i] < 48 || passWord[i] > 57)
        {
            while(PIR1bits.RCIF == 0);
            passWord[i] = RCREG;
        }
        eeprom_write(0x00 + i, passWord[i]);
        while(TXSTAbits.TRMT == 0);
        TXREG = RCREG;       
    }
    enterKey();
}

/*******************************************************************************
 *  void checkCode() is used for checking if the passcode that is entered      *
 *  is the same one as the original password that is stored in EEPROM          *
 * *****************************************************************************/
void checkCode(void)
{
    for (k = 0; k < 4; k++)
        {      
            while(PIR1bits.RCIF == 0);
            tryPassWord[k] = RCREG;
            while (tryPassWord[k] < 48 || tryPassWord[k] > 57)
            {
                while(PIR1bits.RCIF == 0);
                tryPassWord[k] = RCREG;
            }
            while(TXSTAbits.TRMT == 0);
            TXREG = RCREG;          
        } 
        enterKey();
        for (k = 0; k < 4; k++) 
        {
            if (tryPassWord[k] == eeprom_read(0x00 + k)) 
            {
                count++;
            }
            if (k == 3 && count != 4) 
            {
                count = 0;
                printf("\n\rWrong passcode, Please try again");
                __delay_ms(2000);
            }
        }
}

/*******************************************************************************
 *  void ComparisonPassCode() is used for storing the passcode that is entered *
 *  when the user changes the passcode.                                        *
 * *****************************************************************************/
void comparisonPassCode(void)
{
    for (k = 0; k < 4; k++)
    {      
        while(PIR1bits.RCIF == 0);
        passWord1[k] = RCREG;
        while (passWord1[k] < 48 || passWord1[k] > 57)
        {
            while(PIR1bits.RCIF == 0);
            passWord1[k] = RCREG;
        }
        while(TXSTAbits.TRMT == 0);
        TXREG = RCREG;          
    }
    enterKey();
}

// putch(char C) function is for printing a character onto the screen.
void putch(char c)
{
    while(PIR1bits.TXIF == 0) continue; 
    TXREG = c;
}

// enterKey() function is for waiting until user presses enter button
void enterKey(void)
{
    while(PIR1bits.RCIF == 0);
    enter = RCREG;   
    while (enter != 0X0d)
    {
        while(PIR1bits.RCIF == 0);
        enter = RCREG;
    }
}

void disableKeypad(void)
{
    PORTBbits.RB1 = 0;
    PORTDbits.RD4 = 0;
    PORTDbits.RD5 = 0;
    PORTDbits.RD6 = 0;
}

