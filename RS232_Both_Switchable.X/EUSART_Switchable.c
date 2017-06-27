/*******************************************************************************
 * File:    EUSART_Switchable.c
 * Author:  Esseiva Nicolas
 * Company: EMF (Ecole des M�tiers de Fribourg) http://www.emf.ch
 * Desc.:   Ce programme permet de transmettre ou recevoir des donn�es
 *          au moyen de EUSART du PicPocket2 modifi� Gauch de mani�re asynchrone
 *          en utilisant RS-232
 * Date :   20161112 Modifications
 *          20161111 Modifications
 *          20161110 Modifications
 *          20161106 Modifications
 *          20161105 Modifications
 *          20161104 Cr�ation (YYYYMMDD)     
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
#define PERIPHERAL_KEY3 PORTBbits.RB5   		// CMD3 (Utilis� par RS232, ne pas utiliser)
#define PERIPHERAL_KEY4 PORTBbits.RB4   		// CMD4
#define PRESSED 0                               // Bouton actif � 0


#define PERIPHERAL_LED LATBbits.LATB3   		// Etat de sortie de la led
#define PERIPHERAL_BUZZER LATAbits.LATA4  		// Etat de sortie du buzzer

//****************************
// Registers Define
//****************************
//PORTA & PORTB
#define MODEPA 0x00     						// Aucune entr�e analogique sur Port A
#define MODEPB 0x00     						// Aucune entr�e analogique sur Port B
#define STATPA 0x10     						// IBuzzer = min, Boutons s�curis�s, LCDBus = [0000]
#define STATPB 0x08     						// Boutons s�curis�s, Led = off, LCD Disable
#define IOPORTA 0xC0    						// RA6,RA7 en entr�e (CMD2,CMD1)
#define IOPORTB 0xF1    						// RB0,RB4,RB5 en entr�e (INT, CMD4, CMD3)

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
// Transmission
#define EUSART_INTTRANSMITFLAG PIR1bits.TXIF   	// Flag TXREG empty/full
#define EUSART_TSREGFLAG TXSTAbits.TRMT         // Flag transmit ready

// Reception
#define EUSART_INTRECEIVE PIE1bits.RCIE			// Enable RCIF interruption
#define EUSART_INTRECEIVEFLAG PIR1bits.RCIF		// Flag RCREG have new data

#define MSGMAXSIZE 0x08            				// Max size (in byte) RS-232 can save

// Register bits define
#define EUSART_CREN RCSTAbits.CREN				// Enable continuous reception
#define EUSART_OERR RCSTAbits.OERR              // Flag Overrun error
#define EUSART_TXEN TXSTAbits.TXEN				// Enable transmission
#define EUSART_SPEN RCSTAbits.SPEN				// Enable buffer
#define EUSART_SCKP BAUDCONbits.SCKP            // Data inverted or non-inverted

// Pins
#define EUSART_IOPINTX TRISBbits.TRISB5			// IO TX
#define EUSART_IOPINRX TRISBbits.TRISB1			// IO RX

// Pins 2
#define INITGIE INTCONbits.GIE  				// Enable General interuption
#define INITPEIE INTCONbits.PEIE 				// Enable Peripheral interrupt

// ASCII Letters (Pour le passage de MAJ � min en laissant appuy� CMD2)
#define ASCII_A_MAJ 0x41            			// Lettre A en ASCII
#define ASCII_Z_MAJ 0x5A            			// Lettre Z en ASCII
#define ASCII_A_MIN 0x61            			// Lettre a en ASCII
#define ASCII_Z_MIN 0x7A            			// Lettre z en ASCII

//****************************
// Others Define
//****************************
// Messages
#define MSG_DEFAULT_L1 "World  !"      			// Message par d�faut sur la ligne 1
#define MSG_DEFAULT_L2 "> C   Tx"     			// Message par d�faut sur la ligne 2

#define Waiting " Waiting"                      // Message d'attente
#define Idle "RS-232"                           // Message RS-232
#define Receiver "Receiver"                     // Message de mode
#define Received "Received"                     // Message quand re�u

// Max values
#define MENU_ASCII_MAX 0x7F            			// Valeur ASCII maximale (fl�che vers la gauche)
#define MENU_ASCII_MIN 0x20                     // Valeur ASCII minimale (espace)

/* Structures Declarations ****************************************************/
struct _Flags                                   // Structure pour les flags
{
    unsigned char DataRC:1;                     // Flag Data received
    unsigned char ReceiveOFF:1;                 // Flag Reception off
};

struct _Flags Flags1;                           // Definition de Flags1

/* Global Variable Declarations ***********************************************/
unsigned char MenuRow = 0;						// Emplacement du curseur
unsigned char MenuChar[] = MSG_DEFAULT_L1;		// Message du menu modifiable
unsigned char DataReceived[MSGMAXSIZE] = 0;     // Tableau servant de buffer aux donn�es re�ues
unsigned char DataCounter = 0;					// Contient l'addresse du buffer

/* Function Declarations (Prototypes)******************************************/
void ChangeChar (void);                         // Change le caract�re
void EUSART_Initialize(void);                   // Initialisation de RS-232
void INT_Initialize(void);                      // Initialisation des interruptions
void PIC_Initialize(void);              		// Initialisation du pic
void Receive(void);                             // Reception de donn�es
void ShiftCursorRight(void);                    // D�place le curseur vers la droite
void TransmitMsg(unsigned char Msg[]);  		// Transmet un message

void ModeTransmit(void);                        // Pic en mode transmission
void ModeReceive(void);                         // Pic en mode reception
void Reset(void);                               // Red�marrer le pic

/* Function Implementations ***************************************************/

//****************************
// Change Character
//****************************
void ChangeChar (void){                         // Change le caract�re
                                                // CMD2 press� longuement : MAJ -> min | min -> MAJ
    __delay_ms(225);                            // Detect long press or short press

                                                // Si KEY2 toujours press� et valeur entre 'A' et 'Z'
    if((MenuChar[MenuRow] >= ASCII_A_MAJ) && (MenuChar[MenuRow] <= ASCII_Z_MAJ) && PERIPHERAL_KEY2 == 0)
    {
        MenuChar[MenuRow] += 0x20;              // Passage MAJ � min : +20h
    }
    else 
    {                                           // Si KEY2 toujours press� et valeur entre 'a' et 'z'
        if((MenuChar[MenuRow] >= ASCII_A_MIN) && (MenuChar[MenuRow] <= ASCII_Z_MIN) && PERIPHERAL_KEY2 == 0)
        {
            MenuChar[MenuRow] -= 0x20;          // Passage min � MAJ : -20h
        }
        else
        {
            if(MenuChar[MenuRow] >= MENU_ASCII_MAX) // Si valeur plus grand que valeur max
            {
                MenuChar[MenuRow] = MENU_ASCII_MIN; // Placer la valeur � la valeur min
            }
            else
            {
                MenuChar[MenuRow]++;            // Sinon, incr�menter de 1
            }
        }
    }

    LCD_Home();                                 // Retourne � la ligne 1, position 0
    LCD_WriteMessage(MenuChar);                 // Affiche le message chang�
    LCD_SetEntry(0,MenuRow);                    // Retourne � l'emplacement du curseur
}

//****************************
// EUSART Initialization
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
void Receive(void)                              // Reception de donn�es
{
    EUSART_IOPINRX = 1;                         // Met la pin en entr�e pour la reception
    EUSART_INTRECEIVE = 1;                      // Enable Reception interruption
    EUSART_CREN = 1;                            // Activate continuous reception
    
    while(!Flags1.ReceiveOFF)                   // Tant que donn�es re�ues < 8
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

//****************************
// Shift Cursor to the right
//****************************
void ShiftCursorRight(void)                     // D�place le curseur vers la droite
{
    if(MenuRow >= 0x07)                         // Si on va sortir de l'�cran, passer � 0
        MenuRow = 0;
    else                                        // Sinon, incr�menter
        MenuRow++;
    LCD_SetEntry(0,MenuRow);                    // Placer le curseur
}

//------------------------------------------------------------------------------

//****************************
// Transmet a Message
//****************************
void TransmitMsg(unsigned char Msg[])   		// Transmet un message de taille ilimit�e
{
    EUSART_IOPINTX = 0;                         // Pins en sortie
    EUSART_TXEN = 1;                            // Enable transmission
    
    unsigned char Counter;
    for (Counter = 0; Msg[Counter] != 0; Counter++) {
        while(!EUSART_INTTRANSMITFLAG);         // Tant que TXREG est plein
        __delay_ms(2);                          // D�lai anti-parasites
        TXREG = (Msg[Counter] ^ 0xFF);          // Transmet le caract�re du tableau invers�
    }
    while(!EUSART_TSREGFLAG);                   // Attendre qu'une nouvelle transmission est pr�te
    
    EUSART_TXEN = 0;                            // Disable transmission
    EUSART_IOPINTX = 1;                         // Pins en entr�e
    __delay_ms(5);                              // D�lai anti-parasites
}

//------------------------------------------------------------------------------

/* Main Function Implementation ***********************************************/
void main(void)
{   
//****************************
// Initialisation
//****************************
	PIC_Initialize();                           // Initialisation des entr�s / sorties
    
    if (PERIPHERAL_KEY1)                        // Si CMD1 n'est pas press� pendant le d�marrage
    {
        ModeReceive();                          // Mettre en mode R�ception
    }
    else
    {                                           // Sinon
        while(!PERIPHERAL_KEY1);                // Attendre que CMD1 rel�ch�
        ModeTransmit();                         // Mettre en mode Transmission
    }
    
//****************************
// Garde-Fou
//****************************
    while (1);                                  // Garde-Fou  ->  Boucle infinie
}

/* Transmission mode Implementation ********************************************/

void ModeTransmit(void)                         // Mode : transmission
{
//****************************
// Initialisation
//****************************
    LCD_Initialize();                           // Initialisation du LCD
    EUSART_Initialize();                        // Initialisation de l'EUSART
    
    EUSART_IOPINRX = 0;                         // RX not used, output
    EUSART_SPEN = 1;                            // Enable Buffer
  				
												// Afficher le message par d�faut
    LCD_Clear();                                // Clear LCD
    LCD_SetCursor(1,0);                         // Show cursor, blink off
    LCD_WriteMessage(MSG_DEFAULT_L1);           // Afficher message par d�faut sur ligne 1
    LCD_SetEntry(1,0);                          // Se d�placer � la ligne 2
    LCD_WriteMessage(MSG_DEFAULT_L2);           // Afficher le texte sur la ligne 2
    LCD_Home();                                 // Retourner � la ligne 1, position 0
        
//****************************
// Implementation
//**************************** 

    while(1)                                    // Boucle infinie
    {
        
        if(PERIPHERAL_KEY1 == PRESSED)        
        {   //****************************
            // CMD1 press� : d�placer le curseur vers la droite
            //****************************         
            
            ShiftCursorRight();                 // D�placer le curseur
            __delay_ms(25);                     // Debouncing
            while(PERIPHERAL_KEY1 == PRESSED);  // Attendre que touche rel�ch�e
            
        }
        else
        {
            if(PERIPHERAL_KEY2 == PRESSED)    
            {   //****************************
                // CMD2 press� : Changer de caract�re
                // CMD2 press� longuement : MAJ -> min | min -> MAJ
                //****************************  
                
                ChangeChar();                   // Changer le caract�re
                __delay_ms(25);                 // Debouncing
                while(PERIPHERAL_KEY2 == PRESSED); // Attendre que touche rel�ch�e
            }
            else
            {
                if(PERIPHERAL_KEY4 == 0)
                {   //****************************
                    // CMD4 press� : Transmettre le message
                    //****************************  
                    
                    PERIPHERAL_LED = 0;         // Allumer le t�moin lumineux
                    TransmitMsg(MenuChar);      // Transmission du message en mode invers�
                    while(PERIPHERAL_KEY4 == 0);// Attendre que touche rel�ch�e
                    PERIPHERAL_LED = 1;         // Eteindre le t�moin lumineux
                }
            }
        }
    }
}

/* Reception mode Implementation **********************************************/

void ModeReceive(void)
{                                               // Mode : reception
//****************************
// Initialisation
//****************************
    LCD_Initialize();                           // Initialisation du LCD
    EUSART_Initialize();                        // Initialisation de l'EUSART
    INT_Initialize();                           // Initialisation des interruptions
    LCD_SetCursor(0,0);                         // Hide cursor, blink off
    
    Flags1.ReceiveOFF = 0;                      // Mise � 0 des flags
    Flags1.DataRC = 0;
    
    EUSART_IOPINTX = 1;                         // TX not used, input
    EUSART_SPEN = 1;                            // Enable buffer
    
//****************************
// Implementation
//****************************
    LCD_Clear();                                // Clear LCD
    LCD_WriteMessage(Receiver);                 // Affiche message "Receiver"
    LCD_SetEntry(1,0);                          // Se d�placer � la ligne 2
    LCD_WriteMessage(Idle);                     // Affiche le message de d�marrage

    while(PERIPHERAL_KEY4);                     // Attente de CMD4 press�
    __delay_ms(5);                              // Debouncing
    while(PERIPHERAL_KEY4 == 0);                // Attente que CMD4 relach�

    LCD_Clear();                                // Clear LCD
    LCD_WriteMessage(Receiver);                 // Affiche message "Receiver"
    LCD_SetEntry(1,0);                          // Se d�placer � la ligne 2 
    LCD_WriteMessage(Waiting);                  // Afficher message d'attente
    
    Receive();                                  // Reception de donn�es
    
    while(1)                                    // Arr�t du programme une fois donn�es re�ues
    {
        if(!PERIPHERAL_KEY4)                    // Possibilit�e de reset le pic en appuyant sur CMD4
        {
            while(!PERIPHERAL_KEY4);            // Attendre que CMD4 rel�ch�
            Reset();                            // Reset
        }
    }
    
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
    if(EUSART_INTRECEIVE && EUSART_INTRECEIVEFLAG) // Interuption Data received
    {
        EUSART_INTRECEIVE = 0;                  // Reception interruption off
        unsigned char Temp = RCREG;             // Copie les donn�es re�ues dans une variable temporaire
        DataReceived[DataCounter] = (Temp^0xFF);// Place les donn�es re�ues invers�es dans un tableau
        
        if(DataReceived[DataCounter] == 0x00)   // Test valeur re�ue est nulle
        {
            DataCounter--;                      // D�cr�mente l'addresse du tableau pour supprimer cette donn�e
        }
        else
        {
            Flags1.DataRC = 1;                  // Si data re�ue pas nulle, set flag Data received
        }
        
        DataCounter++;                          // Incr�mente l'adresse du tableau de 1
        
                                                // Test si c'est le tableau � 8 bytes et que une valeur n'est pas nulle
        if((DataCounter >= 8) && Flags1.DataRC == 1)
        {
            EUSART_IOPINRX = 0;                 // Met la pin en sortie pour utiliser l'afficheur LCD
            EUSART_SPEN = 0;                    // D�sactive le buffer, et donc la r�ception
            __delay_ms(10);                     // D�lai pour �viter la transmission parasite au LCD
            LCD_Clear();                        // Efface l'�cran
            LCD_WriteMessage(Received);         // Affiche message quand donn�es re�ues
            LCD_SetEntry(1,0);                  // Se d�placer � la 2e ligne
            LCD_WriteMessage(DataReceived);     // Afficher le message re�u
            Flags1.ReceiveOFF = 1;              // Termine la r�ception de donn�es
        }
        
        EUSART_INTRECEIVE = 1;                  // Reception interruption on
    }
}

/* Reset function  Implementation ********************************************/

void Reset(void)                                // Red�marre le pic
{
    asm("reset");                               // Reset
}
/**************************************************************** End-Of-File */