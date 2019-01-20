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
 * 10-21-17: Added CRC 
 * 10-22-17: Cleaned up, added sendToUART(), so that all commands from Host get replies
 * 10-30-17: Fixed bug: processInputString() was being sent twice
 * 10-31-17: Removed all code for second and third UARts. Now only HOST Uart is used.
 * 12-12-17: Revised for DC950 Test System board Rev 1.0
 * 1-16-18:  Added FAULT signal input check
 ************************************************************************************************************/

#define true TRUE
#define false FALSE

#define CR 13
#define LF 10
#define BACKSPACE 8

#define RELAY_K3_ON() PORTSetBits(IOPORT_B, BIT_13)
#define RELAY_K2_ON() PORTSetBits(IOPORT_B, BIT_14)
#define RELAY_K1_ON() PORTSetBits(IOPORT_B, BIT_15)

#define RELAY_K3_OFF() PORTClearBits(IOPORT_B, BIT_13)
#define RELAY_K2_OFF() PORTClearBits(IOPORT_B, BIT_14)
#define RELAY_K1_OFF() PORTClearBits(IOPORT_B, BIT_15)

/** INCLUDES *******************************************************/
#include <XC.h>
#include "Delay.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "Definitions.h"

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
#pragma config DEBUG    = ON


#define MAXPWM 6500

// UART FOR PC SERIAL PORT
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR


#define BUFFERSIZE 128
unsigned char HOSTRxBuffer[BUFFERSIZE];
unsigned char HOSTRxBufferFull = false;
unsigned char HOSTTxBuffer[BUFFERSIZE];
unsigned char HOSTTxBufferFull = false;

/** PRIVATE PROTOTYPES *********************************************/
void InitializeSystem(void);
unsigned char   processInputString(unsigned char *ptrBuffer);
unsigned char   executeCommand(unsigned char *ptrCommand, unsigned char *ptrValue);
unsigned char   setPWM(unsigned char *ptrPWMstring);
unsigned char   setOutput(unsigned char *ptrPin, unsigned char *ptrPinState);
unsigned char   diagnosticsPrintf(unsigned char *ptrString);
unsigned char   sendToUART(unsigned char UartID, unsigned char *ptrUARTpacket);
extern BOOL     CRCcheck(char *ptrPacket);
extern UINT16   CRCcalculate(char *ptrPacket, BOOL addCRCtoPacket);

unsigned char replyToHost(unsigned char *ptrMessage){
    if (ptrMessage == NULL) return (FALSE);
    if (strlen(ptrMessage) > BUFFERSIZE) return (FALSE);
    strcpy(HOSTTxBuffer, ptrMessage);
    CRCcalculate(HOSTTxBuffer, true);
    sendToUART(HOSTuart, HOSTTxBuffer);    
    return(TRUE);
}

int main(void) {         
    InitializeSystem();    
    
    RELAY_K1_OFF();
    RELAY_K2_OFF();
    RELAY_K3_OFF();
    
    while (1) {
        if (HOSTRxBufferFull) {
            HOSTRxBufferFull = false;               
            if (!CRCcheck(HOSTRxBuffer))
               replyToHost("CRC ERROR");                        
            else 
            if (!processInputString(HOSTRxBuffer))
                replyToHost("COMMAND ERROR");                                    
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

    // SET UP PWM OUTPUT
    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenTimer2(T2_ON | T2_PS_1_1 | T2_SOURCE_INT, 0x1FFF);
    SetDCOC1PWM(0);

    PORTClearBits(IOPORT_B, BIT_12 | BIT_13 | BIT_14 | BIT_15);
    mPORTBSetPinsDigitalOut(BIT_12 | BIT_13 | BIT_14 | BIT_15);

    PORTSetPinsDigitalIn(IOPORT_B, BIT_1);
    PORTSetPinsDigitalOut(IOPORT_B, BIT_0);
    PORTSetBits(IOPORT_E, BIT_0);
    PORTSetPinsDigitalIn(IOPORT_D, BIT_15);  // Initialize FAULT signal input

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

}//end UserInit

unsigned char processInputString(unsigned char *ptrBuffer) {
    unsigned char *ptrCommand, *ptrValue;
    unsigned char delimiters[] = "$>[\r";

    ptrCommand = strtok(ptrBuffer, delimiters);
    if (!ptrCommand) return (false);

    ptrValue = strtok(NULL, delimiters);

    if (executeCommand(ptrCommand, ptrValue))
        return (true);
    return (false);
}

unsigned char executeCommand(unsigned char *ptrCommand, unsigned char *ptrValue) {
    if (strstr(ptrCommand, "PWM"))
        setPWM(ptrValue);
    else if (strstr(ptrCommand, "TTL_IN")) {
        if (PORTReadBits(IOPORT_B, BIT_1)) replyToHost("OK");            
        else replyToHost("FAULT");        
        return(true);
    }
    else if (strstr(ptrCommand, "FAULT_IN")){
        if (PORTReadBits(IOPORT_D, BIT_15)) replyToHost("HIGH");
        else replyToHost("LOW");
        return (true);
    }
    else if (strstr(ptrCommand, "TTL_HIGH")) {
        PORTSetBits(IOPORT_B, BIT_0);        
    } else if (strstr(ptrCommand, "TTL_LOW")) {
        PORTClearBits(IOPORT_B, BIT_0);
    } else if (strstr(ptrCommand, "LAMP")) {                
        RELAY_K2_OFF();
        RELAY_K3_ON();       
    } else if (strstr(ptrCommand, "CTRL")) {                
        RELAY_K2_ON();
        RELAY_K3_OFF();                
    } else if (strstr(ptrCommand, "VREF")) {                
        RELAY_K2_OFF();
        RELAY_K3_OFF();         
    } else if (strstr(ptrCommand, "INHIBIT")) {
        if (strstr(ptrValue, "ON")) RELAY_K1_ON();
        else RELAY_K1_OFF();
    } else if (strstr(ptrCommand, "RESET")) {   
        RELAY_K1_OFF();
        RELAY_K2_OFF();
        RELAY_K3_OFF();
        PORTSetBits(IOPORT_E, BIT_0); // Make sure filter actuator is ON
        setPWM("0");
    } else {
        return (false);
    }
    replyToHost("OK");
    return (true);
}

unsigned char setPWM(unsigned char *ptrPWMvalue) {
    unsigned short i, strLength, PWMvalue = 0;
    
    if (ptrPWMvalue == NULL) return (false);
        
    strLength = strlen(ptrPWMvalue);
    for (i = 0; i < strLength; i++) {
        if (i >= BUFFERSIZE) return (false);
        if (!isdigit(ptrPWMvalue[i])) return (false);
    }
    PWMvalue = atoi(ptrPWMvalue);
    if (PWMvalue > MAXPWM) PWMvalue = MAXPWM;
    SetDCOC1PWM(PWMvalue);
}


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
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = toupper(UARTGetDataByte(HOSTuart));            
            if (ch == '$') HOSTRxIndex = 0;
            if (ch == LF || ch == 0);
            else if (ch == BACKSPACE) {
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, ' ');
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, BACKSPACE);
                if (HOSTRxIndex > 0) HOSTRxIndex--;
            } else if (ch == CR) {
                if (HOSTRxIndex < (BUFFERSIZE - 1)) {
                    HOSTRxBuffer[HOSTRxIndex] = CR;
                    HOSTRxBuffer[HOSTRxIndex + 1] = '\0'; 
                    HOSTRxBufferFull = true;
                }
                HOSTRxIndex = 0;
            }                
            else if (HOSTRxIndex < BUFFERSIZE)
                HOSTRxBuffer[HOSTRxIndex++] = ch;
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
}


unsigned char sendToUART(unsigned char UartID, unsigned char *ptrUARTpacket){
    short i;
    unsigned char ch;

    if (strlen(ptrUARTpacket) < BUFFERSIZE) {
        i = 0;
        do {
            ch = ptrUARTpacket[i++];
            if (!ch) break;
            while (!UARTTransmitterIsReady(UartID));
            UARTSendDataByte(UartID, ch);

        } while (i < BUFFERSIZE);
        return (true);
    }
    else return (false);
}

