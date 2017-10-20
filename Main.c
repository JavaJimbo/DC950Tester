/***********************************************************************************************************
 * DC950 TESTER
 * Written for PIC32MX795 using XC32 compiler V 1.30
 * 
 * 10-17-17: 
 * 
 * %SEND:A>STRING[CR]
 * %RELY:1>ON[CR]
 * %PWM:X>1234[CR]
 * 1 volt: PWM = 1227
 * 2 volts: PWM = 2480
 * 3 volts: PWM = 3732
 * 4 volts: PWM = 4982
 * 5 volts: PWM = 6215
 * 
 * 10-20-17: For UBW32 board and PIC795 with diagnostics
 ************************************************************************************************************/

#define true TRUE
#define false FALSE

#define CR 13
#define LF 10
#define BACKSPACE 8

/** INCLUDES *******************************************************/
#include <XC.h>
#include "Delay.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier $$$$
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select


#define MAXPWM 6500

// UART FOR PC SERIAL PORT
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR


// UART FOR HP34401 MULTIMETER UART
#define HPuart UART4
#define HPbits U4STAbits
#define HP_VECTOR _UART_4_VECTOR


#define MAXBUFFER 128
unsigned char HOSTRxBuffer[MAXBUFFER];
unsigned char HOSTRxBufferFull = false;
unsigned char HYPERRxBuffer[MAXBUFFER];

#define MAXBUFFER 128
unsigned char HPRxBuffer[MAXBUFFER];
unsigned char HPRxBufferFull = false;


unsigned char controlCommand = 0;
unsigned short HPRxIndex = 0;
unsigned short HPtestIndex = 0;
short numRXxInterrrupts = 0;
short numHPRXxInterrrupts = 0;

/** PRIVATE PROTOTYPES *********************************************/
void InitializeSystem(void);
unsigned char processInputString(unsigned char *ptrBuffer);
unsigned char executeCommand(unsigned char *ptrCommand, unsigned char *ptrValue);
unsigned char setPWM(unsigned char *ptrPWMstring);
unsigned char setOutput(unsigned char *ptrPin, unsigned char *ptrPinState);
unsigned char sendMeasCommand(void);
unsigned char setRemoteHP(void);
unsigned char resetHP(void);

int main(void) {

    InitializeSystem();
    
    DelayMs(100);
    
    printf("\r\rINTERFACE BOARD START\r\r");
   
    while (1) {
        if (HOSTRxBufferFull) {
            HOSTRxBufferFull = false;            
            if (!processInputString(HOSTRxBuffer)) printf(" Syntax error");            
        }
        if (HPRxBufferFull) {
            HPRxBufferFull = false;
            printf("HP: %s", HPRxBuffer);
        }
    }
}

void InitializeSystem(void) {
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    // Set up HOST UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define SYS_FREQ 80000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 9600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);
    
    // Set up HP34401 MULTI HP UART
    UARTConfigure(HPuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HPuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HPuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2);
#define SYS_FREQ 80000000
    UARTSetDataRate(HPuart, SYS_FREQ, 9600);
    UARTEnable(HPuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HP UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HPuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HPuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HPuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HPuart), INT_SUB_PRIORITY_LEVEL_0);


    // SET UP PWM OUTPUT
    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenTimer2(T2_ON | T2_PS_1_1 | T2_SOURCE_INT, 0x1FFF);
    SetDCOC1PWM(0);

    PORTClearBits(IOPORT_B, BIT_12 | BIT_13 | BIT_14 | BIT_15);
    mPORTBSetPinsDigitalOut(BIT_12 | BIT_13 | BIT_14 | BIT_15);
    
    PORTSetPinsDigitalIn(IOPORT_E, BIT_1);
    PORTSetPinsDigitalOut(IOPORT_E, BIT_0);
    PORTSetBits(IOPORT_E, BIT_0);

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

}//end UserInit


unsigned char processInputString(unsigned char *ptrBuffer) {    
    unsigned char *ptrCommand, *ptrValue;
    unsigned char delimiters[] = "%#> \r";    

    ptrCommand = strtok(ptrBuffer, delimiters);
    if (!ptrCommand) return (false);

    ptrValue = strtok(NULL, delimiters);

    printf("\r\rCommand: ");
    printf("\r%s, ", ptrCommand);
    printf("%s", ptrValue);
    printf("\r\rResult: ");

    if (executeCommand(ptrCommand, ptrValue))    
        return (true);
    return (false);
}



unsigned char executeCommand(unsigned char *ptrCommand, unsigned char *ptrValue) {
    if (strstr(ptrCommand, "HP_RELAY_ON")){
        PORTSetBits(IOPORT_B, BIT_15);
        printf("\rHP RELAY ON");
    }
    else if (strstr(ptrCommand, "HP_RELAY_OFF")){
        PORTClearBits(IOPORT_B, BIT_15);        
        printf("\rHP RELAY OFF");
    }
    else if (strstr(ptrCommand, "PWM"))
        setPWM(ptrValue);
    else if (strstr(ptrCommand, "TTL_IN")) {
        if (PORTReadBits(IOPORT_E, BIT_1)) 
            printf("\rTTL IN = HIGH");
        else printf("\rTTL IN = LOW");                
    }
    else if (strstr(ptrCommand, "MEAS?")) 
        sendMeasCommand();   
    else if (strstr(ptrCommand, "RESET")) 
        resetHP();    
    else if (strstr(ptrCommand, "REMOTE")) 
        setRemoteHP();              
    else if (strstr(ptrCommand, "SET_TTL_HIGH")){
        PORTSetBits(IOPORT_E, BIT_0);
        printf("\rTTL OUTPUT = HIGH");
    } 
    else if (strstr(ptrCommand, "SET_TTL_LOW")){
        PORTClearBits(IOPORT_E, BIT_0);
        printf("\rTTL OUTUPUT = LOW");
    } 
    else {
        printf("\rNo command found");
        return (false);
    }
    return (true);
}


unsigned char setPWM(unsigned char *ptrPWMvalue) {
    unsigned short i, strLength, PWMvalue = 0;

    strLength = strlen(ptrPWMvalue);
    for (i = 0; i < strLength; i++) {
        if (i >= MAXBUFFER) return (false);
        if (!isdigit(ptrPWMvalue[i])) return (false);
    }
    PWMvalue = atoi(ptrPWMvalue);
    if (PWMvalue > MAXPWM) PWMvalue = MAXPWM;
    SetDCOC1PWM(PWMvalue);
    printf("\rSET PWM = %d", PWMvalue);
}


// HP UART interrupt handler it is set at priority level 2
void __ISR(HP_VECTOR, ipl2) IntHPUartHandler(void) {
    unsigned char ch;
    

    if (HPbits.OERR || HPbits.FERR) {
        if (UARTReceivedDataIsAvailable(HPuart))
            ch = UARTGetDataByte(HPuart);
        HPbits.OERR = 0;
        HPRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HPuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HPuart));
        numHPRXxInterrrupts++;
        if (UARTReceivedDataIsAvailable(HPuart)) {
            ch = toupper(UARTGetDataByte(HPuart));
            if (ch != '\n'){
                if (HPRxIndex < MAXBUFFER)
                    HPRxBuffer[HPRxIndex++] = ch;
                if (ch == CR) {
                    HPRxBufferFull = true;
                    if (HPRxIndex < MAXBUFFER) {
                        HPRxBuffer[HPRxIndex] = '\0';
                        HPRxBufferFull = true;
                    }
                    HPRxIndex = 0;
                }
            }
        }
        if (INTGetFlag(INT_SOURCE_UART_TX(HPuart))) {
            INTClearFlag(INT_SOURCE_UART_TX(HPuart));
        }
    }
}

unsigned char sendToUART(unsigned char *ptrUARTname, unsigned char *ptrUARTpacket) {
    short i;
    unsigned char ch;

    
    if (strstr(ptrUARTname, "HP") && strlen(ptrUARTpacket) < MAXBUFFER) {        
        i = 0;
        do {
            ch = ptrUARTpacket[i++];
            if (ch == '\0') break;
            
            while (!UARTTransmitterIsReady(HPuart));
            UARTSendDataByte(HPuart, ch);            

            DelayMs(1);
        } while (i < MAXBUFFER);
        printf("\rSENT: %s", ptrUARTpacket);
        return (true);
    }
    
    return (false);
}



#define CR 13
#define BACKSPACE 8
// HOST UART interrupt handler it is set at priority level 2

void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    unsigned char ch;
    static unsigned short HOSTRxIndex = 0;

    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));     
        numRXxInterrrupts++;
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = toupper(UARTGetDataByte(HOSTuart));
            if (HPtestIndex < MAXBUFFER) HYPERRxBuffer[HPtestIndex++] = ch;
            if (ch == '\n' || ch == 0);            
            else if (ch == BACKSPACE) {
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, ' ');
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, BACKSPACE);
                if (HOSTRxIndex > 0) HOSTRxIndex--;
            }            
             else if (ch == CR) {
                HOSTRxBuffer[HOSTRxIndex] = '\0'; // $$$$
                HOSTRxBufferFull = true;
                HOSTRxIndex = 0;
            } else if (ch < 27) controlCommand = ch;
            else if (HOSTRxIndex < MAXBUFFER)
                HOSTRxBuffer[HOSTRxIndex++] = ch;
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
}

unsigned char sendMeasCommand(void){
unsigned char strCommand[] = ":MEAS?\r\n";
    short i;
    unsigned char ch;
        i = 0;
        do {
            ch = strCommand[i++];        
            if (ch == '\0') break;
            while (!UARTTransmitterIsReady(HPuart));
            UARTSendDataByte(HPuart, ch);            
        } while (i < MAXBUFFER);
        printf("\rSENT: %s", strCommand);
        return (true);
    return(true);
}


unsigned char resetHP(void){
unsigned char strCommand[] = "*RST\r\n";
    short i;
    unsigned char ch;
        i = 0;
        do {
            ch = strCommand[i++];        
            if (ch == '\0') break;
            while (!UARTTransmitterIsReady(HPuart));
            UARTSendDataByte(HPuart, ch);            
        } while (i < MAXBUFFER);
        printf("\rSENT: %s", strCommand);
        return (true);
    return(true);
}

unsigned char setRemoteHP(void){
unsigned char strCommand[] = ":SYST:REM\r\n";
    short i;
    unsigned char ch;
        i = 0;
        do {
            ch = strCommand[i++];        
            if (ch == '\0') break;            
            while (!UARTTransmitterIsReady(HPuart));
            UARTSendDataByte(HPuart, ch);            
        } while (i < MAXBUFFER);
        printf("\rSENT: %s", strCommand);
        return (true);
    return(true);
}