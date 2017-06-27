/*******************************************************************************
 * File:    LCD.h
 * Author:  Esseiva Nicolas
 * Company: EMF (Ecole des Métiers de Fribourg) http://www.emf.ch
 * Desc.:   Ce programme permet d'utiliser les fonctions de base de
 *          l'afficheur LCD EA-DIPS082 du PicPocket2 modifié Gauch
 * Date :   20161104 Modification
 *          20161103 Création (YYYYMMDD)     
 ******************************************************************************/

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

/* Compilation Directives *****************************************************/

//****************************
// Project Frequency Settings
//****************************
#define _XTAL_FREQ  16000000                    // this is used by the __delay_us(xx)  

/* Constant Definitions *******************************************************/


#define LCD_E LATBbits.LATB2                    // Entrée de validation du registre (E)
#define LCD_RS LATBbits.LATB1                   // Sélection commande/caractère
#define LCDRESET4B 0x03                         // Clé d'initialisation 4bits
#define LCDMODE4B 0x02                          // Indicateur de mode 4bits
#define LCDFUNCSET 0x28                         // Display 2 lines, 5x7 dots
#define LCDDISPON 0x0F                          // Display On, Show Cursor, Blink on
#define LCDCLEAR 0x01                           // Clear Display
#define LCDENTRY 0x06                           // Increment Adress, No Shift
#define LCDHOME 0x02                            // Placer le curseur à 0,0
#define LCDCURSOR 0x0C                          // Réglages du curseur
#define LCDSETDDRAM 0x80                        // Placer le curseur à une position précise

/* Global Variable Declarations ***********************************************/

/* Function Declarations (Prototypes)******************************************/
void LCD_Clear(void);                           // Efface l'écran
void LCD_Home(void);                            // Placer le curseur à 0,0
void LCD_Initialize(void);                      // Initialisation de l'afficheur LCD
void LCD_SetCursor(unsigned char View,unsigned char Blink); // Réglages curseur
void LCD_SetEntry(unsigned char Line,unsigned char Row); // Placer le curseur
void LCD_Write4b(unsigned char Code);           // Write a nibble to the LCD
void LCD_Write8b(unsigned char Code);           // Write a byte to the LCD
void LCD_WriteChar(unsigned char Char);         // Write a Data to the LCD
void LCD_WriteCommand(unsigned char Cmd);       // Write a Command to the LCD
void LCD_WriteDCBToASCII(unsigned char DataDCB); // Write a DCB to the LCD
void LCD_WriteMessage(unsigned char Msg[]);     // Write a string to the LCD

void Tests(void);                               // Tester les fonctions

    /* Function Implementations ***************************************************/

    void LCD_Clear(void)                        // Efface l'écran
    {
        LCD_WriteCommand(LCDCLEAR);             // Effacer l'écran
    }

    //------------------------------------------------------------------------------

    void LCD_Home(void)                         // Retourne au début
    {
        LCD_WriteCommand(LCDHOME);              // Placer le curseur à 0,0
    }

    //------------------------------------------------------------------------------

    void LCD_Initialize(void)                   // Initialisation de l'afficheur LCD
    {
        __delay_ms(120);                        // Attendre au moins 100ms
        LCD_Write4b(LCDRESET4B);                // Envoi de la 1ère clé d'initialisation
        __delay_ms(5);                          // Attendre au moins 4.1ms
        LCD_Write4b(LCDRESET4B);                // Envoi de la 2ème clé d'initialisation
        __delay_us(120);                        // Attendre au moins 100us
        LCD_Write4b(LCDRESET4B);                // Envoi de la 3ème clé d'initialisation
        __delay_us(120);                        // Attendre au moins 100us
        LCD_Write4b(LCDMODE4B);                 // Mise en mode 4bits
        __delay_us(120);                        // Attendre au moins 100us

        LCD_WriteCommand(LCDFUNCSET);           // Function set
        LCD_WriteCommand(LCDDISPON);            // Display On Off
        LCD_WriteCommand(LCDCLEAR);             // Clear Display
        LCD_WriteCommand(LCDENTRY);             // Entry mode set
    }
    //------------------------------------------------------------------------------

    void LCD_SetCursor(unsigned char View,unsigned char Blink) // Réglages curseur
    {
        unsigned char Cmd = LCDCURSOR;

        if (View)
            Cmd |= 0x02;
        if (Blink)
            Cmd |= 0x01;
        LCD_WriteCommand(Cmd);
    }

    //------------------------------------------------------------------------------
    void LCD_SetEntry(unsigned char Line,unsigned char Row)    // Placer le curseur
    {
        unsigned char Cmd = LCDSETDDRAM;

        if(!Line)                               // Si Line = 0
        {
            if (Row > 7)
            {
                Cmd += 7;
            } else 
            {
                Cmd += Row;
            }
        } else 
        {
            Cmd += 0x40;
            if (Row > 7)
            {
                Cmd += 7;
            } else 
            {
                Cmd += Row;
            }
        }

        LCD_WriteCommand(Cmd);
    }

    //------------------------------------------------------------------------------

    void LCD_Write4b(unsigned char Code)        // Ecrit le quartet faible dans le LCD
    {
        unsigned char Temp;                     // Variable temporaire
        
        LCD_E = 1;                              // Ouverture du port du LCD
        
        Temp = PORTA;                           // Récupération état des sorties
        Temp &= 0xF0;                           // Forcer le quartet faible à 0
        Temp |= (Code & 0x0F);                  // Insertion du quartet faible
        PORTA = Temp;                           // Ecriture dans le Port A
        
        LCD_E = 0;                              // Strobe sur Enable du LCD
    }
    //------------------------------------------------------------------------------

    void LCD_Write8b(unsigned char Code)         // Ecrit un byte dans le LCD
    {
        unsigned char Temp;                     // Variable temporaire
        
        LCD_E = 1;                              // Ouverture du bus
        
        Temp = PORTA;                           // Récupération état des sorties
        Temp &= 0xF0;                           // Forcer le quartet faible à 0
        Temp |= (Code >> 4);                    // Insertion du quartet fort
        PORTA = Temp;                           // Ecriture dans le Port A
        
        LCD_E = 0;                              // Strobe de la valeur dans le bus
        
        asm("nop");                             // Attente
        
        LCD_E = 1;                              // Ouverture du bus
        
        Temp = PORTA;                           // Récupération état des sorties
        Temp &= 0xF0;                           // Forcer le quartet faible à 0
        Temp |= (Code & 0x0F);                  // Insertion du quartet faible
        PORTA = Temp;                           // Ecriture dans le Port A
        
        LCD_E = 0;                              // Strobe de la valeur dans le bus
    }
    //------------------------------------------------------------------------------

    void LCD_WriteChar(unsigned char Char)      // Write a Data to the LCD
    {
        LCD_RS = 1;                             // Signaler une donnnée

        LCD_Write8b(Char);                      // Ecriture de la donnée
        __delay_us(50);                         // Attendre au moins 43us
    }

    //------------------------------------------------------------------------------

    void LCD_WriteCommand(unsigned char Cmd)    // Write a Command to the LCD
    {
        LCD_RS = 0;                             // Signaler une commande

        LCD_Write8b(Cmd);                       // Ecriture de la commande
        __delay_ms(2);                          // Attendre au moins 1.53ms
    }

    //------------------------------------------------------------------------------
    
    void LCD_WriteDCBToASCII(unsigned char DataDCB) // Conversion DCB en ASCII
    {
        unsigned char Temp = DataDCB;
        Temp &= 0x0F;
        Temp += 0x30;
        LCD_WriteChar(Temp);
        
        Temp = (DataDCB >> 4);
        Temp &= 0x0F;
        Temp += 0x30;
        LCD_WriteChar(Temp);
    }


    //------------------------------------------------------------------------------

    void LCD_WriteMessage(unsigned char Msg[])  // Write a string to the LCD
    {    
        unsigned char i;
        for (i = 0; Msg[i] != 0; i++) {
            LCD_WriteChar(Msg[i]);
        }
    }

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

