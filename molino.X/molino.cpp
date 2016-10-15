/* 
 * File:   molino.cpp
 * Author: andres
 *
 * Created on 8 de octubre de 2016, 09:47
 */

// PIC32MX230F064B Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = OFF           // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = OFF          // USB VBUS ON Selection (Controlled by Port Function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_1         // USB PLL Input Divider (1x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_8         // System PLL Output Clock Divider (PLL Divide by 8)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enable, FSCM Enabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = ON              // JTAG Enable (JTAG Port Enabled)
#pragma config ICESEL = ICS_PGx3        // ICE/ICD Comm Channel Select (Communicate on PGEC3/PGED3)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <sys/attribs.h>


#include <cstdlib>

using namespace std;

/*
 * 
 */

void T1_config(void);
void Port_config(void);
void Enviar_msg(void);
void inicializar_msg(void);
void T23_config(void);
void analizar_msg(void);

unsigned i=0;
unsigned indice_rx=0;
bool recibiendo=false;
bool fin_rx=false;
unsigned data_in;

unsigned indice_tx=0;
bool enviando=false;

unsigned direccion_propia=2;

struct{
    unsigned    preambulo:1;
    unsigned    direccion:3;
    unsigned    codigo:3;
    unsigned    datos:8;
    unsigned    cola:1;
} msg_tx;

struct{
    unsigned    direccion:3;
    unsigned    codigo:3;
    unsigned    datos:8;
} msg_rx;


int main(int argc, char** argv) {
__builtin_enable_interrupts();
 
    INTCON=0x1000;
    inicializar_msg();
    Port_config();
    T1_config();
    //T23_config();  //Para molino no es necesario
    while(1){
        i++;
        if(fin_rx){
            analizar_msg();
            fin_rx=false;
        }
    }
    return 0;
}

extern "C"{
void __ISR(_TIMER_1_VECTOR, IPL6AUTO) Timer1Handler(void)
{
        
        //LATBbits.LATB14=!LATBbits.LATB14;
        
        
        if(recibiendo){
            T1CONbits.ON=0;
            PR1=0x9896;
            T1CONbits.ON=1;
            
            data_in=PORTBbits.RB7;
            switch(indice_rx){
                case 1:
                    msg_rx.direccion|=data_in<<2;
                    indice_rx++;
                    break;
                case 2:
                    msg_rx.direccion|=data_in<<1;
                    indice_rx++;
                    break; 
                case 3:
                    msg_rx.direccion|=data_in<<0;
                    indice_rx++;
                    break;
                case 4:
                    msg_rx.codigo|=data_in<<2;
                    indice_rx++;
                    break;
                case 5:
                    msg_rx.codigo|=data_in<<1;
                    indice_rx++;
                    break; 
                case 6:
                    msg_rx.codigo|=data_in<<0;
                    indice_rx++;
                    break;
                case 7:
                    msg_rx.datos|=data_in<<7;
                    indice_rx++;
                    break;
                case 8:
                    msg_rx.datos|=data_in<<6;
                    indice_rx++;
                    break; 
                case 9:
                    msg_rx.datos|=data_in<<5;
                    indice_rx++;
                    break;
                case 10:
                    msg_rx.datos|=data_in<<4;
                    indice_rx++;
                    break;
                case 11:
                    msg_rx.datos|=data_in<<3;
                    indice_rx++;
                    break; 
                case 12:
                    msg_rx.datos|=data_in<<2;
                    indice_rx++;
                    break;
                case 13:
                    msg_rx.datos|=data_in<<1;
                    indice_rx++;
                    break; 
                case 14:
                    msg_rx.datos|=data_in<<0;
                    indice_rx++;
                    break;
                case 15:
                    indice_rx++;
                    break;
                default:
                    T1CONbits.ON=0;
                    recibiendo=false;
                    fin_rx=true;
                    IFS0bits.INT0IF=0;
                    IEC0bits.INT0IE=1;
            }
        }else if(enviando){
            switch (indice_tx){
        case 0:
            LATBbits.LATB14=msg_tx.preambulo;
            indice_tx++;
            break;
        case 1:
            LATBbits.LATB14=(msg_tx.direccion >> 2) & 1;
            indice_tx++;
            break;
        case 2:
            LATBbits.LATB14=(msg_tx.direccion >> 1) & 1;
            indice_tx++;
            break;
        case 3:
            LATBbits.LATB14=(msg_tx.direccion >> 0) & 1;
            indice_tx++;
            break;
        case 4:
            LATBbits.LATB14=(msg_tx.codigo >> 2) & 1;
            indice_tx++;
            break;
        case 5:
            LATBbits.LATB14=(msg_tx.codigo >> 1) & 1;
            indice_tx++;
            break;
        case 6:
            LATBbits.LATB14=(msg_tx.codigo >> 0) & 1;
            indice_tx++;
            break;
        case 7:
            LATBbits.LATB14=(msg_tx.datos >> 7) & 1;
            indice_tx++;
            break;
        case 8:
            LATBbits.LATB14=(msg_tx.datos >> 6) & 1;
            indice_tx++;
            break;
        case 9:
            LATBbits.LATB14=(msg_tx.datos >> 5) & 1;
            indice_tx++;
            break;
        case 10:
            LATBbits.LATB14=(msg_tx.datos >> 4) & 1;
            indice_tx++;
            break;
        case 11:
            LATBbits.LATB14=(msg_tx.datos >> 3) & 1;
            indice_tx++;
            break;
        case 12:
            LATBbits.LATB14=(msg_tx.datos >> 2) & 1;
            indice_tx++;
            break;
        case 13:
            LATBbits.LATB14=(msg_tx.datos >> 1) & 1;
            indice_tx++;
            break;
        case 14:
            LATBbits.LATB14=(msg_tx.datos >> 0) & 1;
            indice_tx++;
            break;
        case 15:
            LATBbits.LATB14=msg_tx.cola;
            indice_tx++;
            break;
        default:
            T1CONbits.TON=0;
            IFS0bits.INT0IF=0;
            IEC0bits.INT0IE=1;
            enviando=false;
            indice_tx=0;
            LATBbits.LATB15=0;
            break;            
    }
    }
    IFS0bits.T1IF=0;
        
}
void __ISR(_EXTERNAL_0_VECTOR, IPL6AUTO) External0Handler(void)
{
    IEC0bits.INT0IE=0;
    IFS0bits.INT0IF=0;
    //LATBbits.LATB14=!LATBbits.LATB14;
    
    recibiendo=true;
    indice_rx=1;
    msg_rx.direccion=0;
    msg_rx.codigo=0;
    msg_rx.datos=0;
    
    T1CON=0x30;  //Pone el Timer 1 en 1.5 segundos 
    PR1=0xE4E2;
    IEC0bits.T1IE=1;
    IPC1=0x1b;
    T1CONbits.ON=1;
    
}
void __ISR(_TIMER_3_VECTOR, IPL6AUTO) Timer3Handler(void)
{
    msg_tx.preambulo=0;
    msg_tx.direccion=2;
    msg_tx.codigo=1;
    msg_tx.datos++;
    msg_tx.cola=1;
    Enviar_msg();
    IFS0bits.T3IF=0;
}

}


void T1_config(void)
{
    T1CON=0x30;
    PR1=0x9896;
    IEC0bits.T1IE=1;
    IPC1=0x1b;
    T1CONbits.ON=0;
}

void Port_config(void){
    TRISB=0;
    TRISBbits.TRISB5=0; //Se activa mientras se envia un mensaje
    TRISBbits.TRISB7=1; //Para entrada de datos serie
    
    INTCONbits.INT0EP=1;
    IFS0bits.INT0IF=0; //Interrupcion entrada de datos serie
    IPC0=0x15000000;
    IEC0bits.INT0IE=1;
    LATB=0;
    LATBbits.LATB14=0;
}

void Enviar_msg(void){
    enviando=true;
    T1CONbits.ON=1;
    indice_tx=0;
    IEC0bits.INT0IE=0;
    LATBbits.LATB15=1;
}

void inicializar_msg(void){
    msg_tx.preambulo=1;
    msg_tx.direccion=1;
    msg_tx.codigo=1;
    msg_tx.datos=0;
    msg_tx.cola=0;
}

void T23_config(void){
    T2CON=0x78;
    //T2CON=0x58;
    //PR3=0x23;
    PR2=0x23C346;
    IFS0bits.T3IF=0;
    IEC0bits.T3IE=1;
    IPC3bits.T3IP=1;
    IPC3bits.T3IS=1;
    T2CONbits.ON=1;    
}

void analizar_msg(void){
    if(msg_rx.direccion==direccion_propia){
        if(msg_rx.codigo==1){
            msg_tx.direccion=1;
            msg_tx.codigo=2;
            msg_tx.datos=5;
            Enviar_msg();
        }
    }
}