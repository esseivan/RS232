/*******************************************************************************
 * File:    EUSART_Reception.c
 * Author:  Morand Nathann
 * Company: EMF (Ecole des Métiers de Fribourg) http://www.emf.ch
 * Desc.:   Ce programme permet de recevoir des données
 *          au moyen de EUSART du PicPocket2 modifié Gauch de manière asynchrone
 *          en utilisant RS-232
 * Date :   20161112 Modifications
 *          20161111 Modifications
 *          20161110 Modifications
 *          20161106 Modifications
 *          20161105 Modifications
 *          20161104 Création (YYYYMMDD)
 ******************************************************************************/

#include <xc.h>                                 // include processor files - each processor file is guarded.  
#include "LCD.h"           

/* Compilation Directives *****************************************************/

//****************************
// Configuration Bit Settings
//****************************
// CONFIG1
#pragma config FOSC = INTOSC    				// Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       				// Watchdog Timer Enable (WDT disabLED)
#pragma config PWRTE = ON       				// Power-up Timer Enable (PWRT enabLED) Program sur pic : ON
#pragma config MCLRE = ON       				// MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         				// Flash Program Memory Code Protection (Program memory code protection is disabLED)
#pragma config CPD = OFF        				// Data Memory Code Protection (Data memory code protection is disabLED)
#pragma config BOREN = OFF      				// Brown-out Reset Enable (Brown-out Reset disabLED)
#pragma config CLKOUTEN = OFF   				// Clock Out Enable (CLKOUT function is disabLED. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       				// Internal/External KEYover (Internal/External KEYover mode is disabLED)
#pragma config FCMEN = OFF      				// Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabLED)

// CONFIG2				
#pragma config WRT = OFF        				// Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      				// PLL Enable (4x PLL disabLED)
#pragma config STVREN = ON      				// Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = HI        				// Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        				// Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

/* Constant Definitions *******************************************************/

//****************************
// Frequency Settings
//****************************
#define _XTAL_FREQ  16000000    				// this is used by the __delay_us(xx)  

//****************************
// Peripheral Define
//****************************
#define PERIPHERAL_KEY1 PORTAbits.RA7   		// CMD1
#define PERIPHERAL_KEY2 PORTAbits.RA6   		// CMD2
#define PERIPHERAL_KEY3 PORTBbits.RB5   		// CMD3 (Utilisé par transmission RS232)
#define PERIPHERAL_KEY4 PORTBbits.RB4   		// CMD4

#define PERIPHERAL_LED LATBbits.LATB3           // Etat de sortie de la led
#define PERIPHERAL_BUZZER LATAbits.LATA4  		// Etat de sortie du buzzer

//****************************
// Registers Define
//****************************
//PORTA & PORTB
#define MODEPA 0x00     						// Aucune entrée analogique sur Port A
#define MODEPB 0x00     						// Aucune entrée analogique sur Port B
#define STATPA 0x10     						// IBuzzer = min, Boutons sécurisés, LCDBus = [0000]
#define STATPB 0x08     						// Boutons sécurisés, Led = off, LCD Disable
#define IOPORTA 0xC0    						// RA6,RA7 en entrée (CMD2,CMD1)
#define IOPORTB 0xF1    						// RB0,RB4,RB5 en entrée (INT, CMD4, CMD3)

// TXSTA, RCSTA, BAUDCON
#define EUSART_MODETRANSMIT 0x00       			// Transmit off, async, low speed
#define EUSART_MODERECEIVE 0x80        			// SerialPort on, 8bit, receive off
#define EUSART_MODEBAUDRATE 0x00       			// Non-inverted data, 8-bit baud rate

// Baud rate generator (9600b/s)
#define BRGHIGH 0x01            				// Baud rate generator High = 1h
#define BRGLOW 25               				// Baud rate generator Low = 25d

//****************************
// EUSART Define
//****************************
// Reception
#define EUSART_INTRECEIVE PIE1bits.RCIE			// Enable RCIF interruption
#define EUSART_INTRECEIVEFLAG PIR1bits.RCIF		// Flag RCREG have new data

#define MSGMAXSIZE 0x08            				// Max size (in byte) RS-232 can save

// Register bits define
#define EUSART_CREN RCSTAbits.CREN				// Enable continuous reception
#define EUSART_OERR RCSTAbits.OERR              // Flag Overrun error
#define EUSART_SPEN RCSTAbits.SPEN				// Enable buffer

// Pins
#define EUSART_IOPINTX TRISBbits.TRISB5			// IO TX
#define EUSART_IOPINRX TRISBbits.TRISB1			// IO RX

// Pins 2
#define INITINT INTCONbits.INTE 				// Enable interrupt from RB0
#define INITGIE INTCONbits.GIE  				// Enable General interuption
#define INITPEIE INTCONbits.PEIE 				// Enable Peripheral interrupt

//****************************
// Others Define
//****************************
// Messages
#define Waiting " Waiting"                      // Message d'attente
#define Idle "RS-232"                           // Message RS-232
#define Receiver "Receiver"                     // Message de mode
#define Received "Received"                     // Message quand reçu

/* Structures Declarations ****************************************************/
struct _Flags                                   // Structure pour les flags
{
    unsigned char DataRC:1;                     // Flag Data received
    unsigned char ReceiveOFF:1;                 // Flag Reception off
};

struct _Flags Flags1;                           // Definition de Flags1

/* Global Variable Declarations ***********************************************/
unsigned char DataReceived[MSGMAXSIZE] = 0;     // Tableau servant de buffer aux données reçues
unsigned char DataCounter = 0;					// Contient l'addresse du buffer

/* Function Declarations (Prototypes)******************************************/
void EUSART_Initialize(void);                   // Initialisation de l'EUSART
void INT_Initialize(void);                      // Initialisation des interruptions
void PIC_Initialize(void);              		// Initialisation du pic
void Receive(void);                             // Reception de données

/* Function Implementations ***************************************************/

//****************************
// EUSART Initialisation
//****************************
void EUSART_Initialize(void)                    // Initialisation de l'EUSART
{
    RCSTA = EUSART_MODERECEIVE;                	// Receive control register
    TXSTA = EUSART_MODETRANSMIT;               	// Transmit control register
    BAUDCON = EUSART_MODEBAUDRATE;             	// Baud rate generator control register
    SPBRGH = BRGHIGH;                           // Baud rate generator register High
    SPBRGL = BRGLOW;                            // Baud rate generator register Low
    APFCON0bits.RXDTSEL = 0;            		// Pin des data = 0:RB1 , 1:RB2
    APFCON1bits.TXCKSEL = 1;            		// Pin de la clock = 0:RB2 , 1:RB5
}

//------------------------------------------------------------------------------

//****************************
// Interrupt Initialisation
//****************************
void INT_Initialize(void)                       // Initialisation des interruptions
{
    INITGIE = 1;                                // Enable Genral Interrupt
    INITPEIE = 1;                               // Enable peripheral interrupts
}

//------------------------------------------------------------------------------

//****************************
// PIC Initialisation
//****************************
void PIC_Initialize(void)                       // Initialisation du pic
{ 
	//setups the Oscillator of the microcontroller
	OSCCONbits.SPLLEN = 0;                      // PLL is disabled
    OSCCONbits.IRCF = 0x0F;                     // select OSC frequency = 16Mhz
    OSCCONbits.SCS = 0x02;                      // select internal oscillator block
	
	//setups the Ports of the microcontroller
    ANSELA = MODEPA;                            // Analog Config Port A
    ANSELB = MODEPB;                            // Analog Config Port B    
    LATA = STATPA;                              // Etat initial de Port A
    LATB = STATPB;                              // Etat initial de Port B
    TRISA = IOPORTA;                            // Port A as input
    TRISB = IOPORTB;                            // Port B as input    
    
	//setups the Pull-Up of the microcontroller
	WPUB = 0x00;                                // Disable pull-up on le port B
    WPUBbits.WPUB6 = 1;                         // Enable pull-up on port B
    WPUBbits.WPUB7 = 1;                         // Enable pull-up on port B
    OPTION_REGbits.nWPUEN = 0;                  // Global Activation of the pull-up on Port B
}

//------------------------------------------------------------------------------

//****************************
// Ecoute les message en RS-232
//****************************
void Receive(void)                              // Reception de données
{
    EUSART_IOPINRX = 1;                         // Met la pin en entrée pour la reception
    EUSART_INTRECEIVE = 1;                      // Enable Reception interruption
    EUSART_CREN = 1;                            // Activate continuous reception
    
    while(!Flags1.ReceiveOFF)                   // Tant que données reçues < 8
    {                                           // Blink LED @ 4Hz
        __delay_ms(125);
        PERIPHERAL_LED = (!PERIPHERAL_LED);
        
        if(EUSART_OERR)                         // Si Overrunn error
        {
            EUSART_CREN = 0;                    // CREN = 0
            __delay_ms(1);
            EUSART_CREN = 1;                    // CREN = 1
        }
    }
    
    PERIPHERAL_LED = 1;                         // Eteint la led
}

//------------------------------------------------------------------------------

/* Main Function Implementation ***********************************************/

void main(void)
{       
//****************************
// Initialisation
//****************************
	PIC_Initialize();                           // Initialisation des entrés / sorties
    LCD_Initialize();                           // Initialisation du LCD
    EUSART_Initialize();                        // Initialisation de l'EUSART
    INT_Initialize();                           // Initialisation des interruptions
    LCD_SetCursor(0,0);                         // Hide cursor, blink off
    
    Flags1.ReceiveOFF = 0;                      // Mise à 0 des flags
    Flags1.DataRC = 0;
    
    EUSART_IOPINTX = 1;                         // TX not used, input
    EUSART_SPEN = 1;                            // Enable buffer
    
//****************************
// Implementation
//****************************
    LCD_Clear();                                // Clear LCD
    LCD_WriteMessage(Receiver);                 // Affiche message "Receiver"
    LCD_SetEntry(1,0);                          // Se déplacer à la ligne 2
    LCD_WriteMessage(Idle);                     // Affiche le message de démarrage

    while(PERIPHERAL_KEY4);                     // Attente de CMD4 pressé
    __delay_ms(5);                              // Debouncing
    while(PERIPHERAL_KEY4 == 0);                // Attente que CMD4 relaché

    LCD_Clear();                                // Clear LCD
    LCD_WriteMessage(Receiver);                 // Affiche message "Receiver"
    LCD_SetEntry(1,0);                          // Se déplacer à la ligne 2 
    LCD_WriteMessage(Waiting);                  // Afficher message d'attente
    
    Receive();                                  // Reception de données

    while(1);                                   // Arrêt du programme une fois données reçues
    
//****************************
// Garde-Fou
//****************************
    while (1);                                  // Garde-Fou  ->  Boucle infinie
}

/* Interrupt Function Implementation ******************************************/

void interrupt ISR(void)
{
//****************************
// Interruption a la reception de message
//**************************** 
    if(EUSART_INTRECEIVE && EUSART_INTRECEIVEFLAG)  // Interuption Data received
    {
        EUSART_INTRECEIVE = 0;                  // Reception interruption off
        unsigned char Temp = RCREG;             // Copie les données reçues dans une variable temporaire
        DataReceived[DataCounter] = (Temp^0xFF);// Place les données reçues inversées dans un tableau
        
        if(DataReceived[DataCounter] == 0x00)   // Test valeur reçue est nulle
        {
            DataCounter--;                      // Décrémente l'addresse du tableau pour supprimer cette donnée
        } else {
            Flags1.DataRC = 1;                  // Si data reçue pas nulle, set flag Data received
        }
        
        DataCounter++;                          // Incrémente l'adresse du tableau de 1
        
                                                // Test si c'est le tableau à 8 bytes et que une valeur n'est pas nulle
        if((DataCounter >= 8) && Flags1.DataRC == 1)
        {
            EUSART_IOPINRX = 0;                 // Met la pin en sortie pour utiliser l'afficheur LCD
            EUSART_SPEN = 0;                    // Désactive le buffer, et donc la réception
            __delay_ms(10);                     // Délai pour éviter la transmission parasite au LCD
            LCD_Clear();                        // Efface l'écran
            LCD_WriteMessage(Received);         // Affiche message quand données reçues
            LCD_SetEntry(1,0);                  // Se déplacer à la 2e ligne
            LCD_WriteMessage(DataReceived);     // Afficher le message reçu
            Flags1.ReceiveOFF = 1;              // Termine la réception de données
        }
        
        EUSART_INTRECEIVE = 1;                  // Reception interruption on
    }
}

/**************************************************************** End-Of-File */