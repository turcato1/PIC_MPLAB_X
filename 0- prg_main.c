/*
 * File:   prg_main.c
 * Author: tomys
 *
 * Created on 16 de Abril de 2020, 20:21
 */

/*
 * PROGRAMA DE EXIBIÇÃO DE CARACTERES EM MATRIZ 5 X 7, COM USO DO CI HT16K33
 * MAPA DE CARACTERES ARMAZENADO EM EEPROM, COM CONJUNTO LIMITADO
 * CARACTERES EXIBÍVEIS: ()*+,-./0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ
 * ASCII Cód. 40 - 90 (decimal)
 * 
 * LOCALIZAÇÃO DO CARACTER NA MEMORIA EEPROM = ((COD ASCII)-40)*5
 */

/* PRAGMAS DO MICROCONTROLADOR */
/* MODELO DO MICROCONTROLADOR = ***** 18F4550 ***** */
// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator (HS))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF            // ******* Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF         // ******* PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF            // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT = ON           // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

/* INCLUDES */
#include <xc.h>

/* DEFINES */
#define ADDR_HT16K  0xE0     // Endereço I2C do chip HT16K33
#define _XTAL_FREQ  8000000  // Frequencia de clock do microcontrolador

/* VARIÁVEIS GLOBAIS */
unsigned char charmap[256];  //Mapa de caracteres espelhado na RAM
unsigned char disp_message[30];       //Mensagem a ser exibida no display

unsigned char eeprom18_read(unsigned char offset) {
    EECON1bits.EEPGD = 0; //accesses data EEPROM memory
    EECON1bits.CFGS = 0; //accesses data EEPROM memory

    EEADR = offset;

    EECON1bits.RD = 1; //initiates an EEPROM read
    Nop(); //it can be read after one NOP instruction
    
    return EEDATA;
}

void i2c_init(){
    TRISBbits.TRISB0 = 1;       //Define RB0 (SDA) como input
    TRISBbits.TRISB1 = 1;       //Define RB1 (SCL) como input
    SSPCON1bits.SSPM = 0x08; //Define interface I2C como mestre (master)    
    SSPCON1bits.SSPEN = 1;   //Habilita pinos RB1 e RB1 para serem usados como SCL e SDA       
    SSPSTATbits.SMP = 1;     //Define slew rate control bit como "standard speed"
    SSPSTATbits.CKE = 0;     //Desabilita modo SMBus (sistema derivado do I2C)
    SSPADD = 19;             //Define clock I2C = 100kHz (clock = 8 MHz/4*[SSPADD+1] )
}

void i2c_is_idle(){
    while((SSPCON2 & 0x1F) || (SSPSTAT & 0x04));
    // SSPCON2 = GCEN ACTSTAT ACKDT ACKEN RCEN PEN RSEN SEN
    //  As condições abaixo são filtradas na lógica acima:
    //  (bit 3) RCEN = 0, Receive idle
    //  (bit 2) PEN = 0, Stop Condition idle 
    //  (bit 1) RSEN = 0, Repeated Start Condition idle
    //  (bit 0) SEN = 0, Start Condition Enable idle
    //
    // SSPSTAT = SMP CKE D/A P S R/W UA BF
    // (bit 4) S = 0, Start not detected
}

void i2c_start(){
    i2c_is_idle();                 // Verifica se o status do I2C é idle
    SSPCON2bits.SEN = 1;           // Executa o start    
}

void i2c_rep_start(){
    i2c_is_idle();                 // Verifica se o status do I2C é idle
    SSPCON2bits.RSEN = 1;          // Executa o repeated start
}

void i2c_stop(){
    i2c_is_idle();                  // Verifica se o status do I2C é idle
    SSPCON2bits.PEN = 1;            // Executa o stop
}

void i2c_write(unsigned char write_data){
    i2c_is_idle();                 // Verifica se o status do I2C é idle
    SSPBUF = write_data;           // Transfere dados a serem escritos para o buffer
    while (SSPSTATbits.BF);        // Aguarda Buffer Full
    while (SSPCON2bits.ACKSTAT);   // Aguarda ACK do escravo
}

unsigned char i2c_read(unsigned char ack){
    unsigned char received_data = 0;
    
    i2c_is_idle();
    SSPCON2bits.RCEN = 1;          // Inicia a recepção
    while (!SSPSTATbits.BF);       // Aguarda buffer encher
    received_data = SSPBUF;        // Transfere dados do buffer para variável
    SSPCON2bits.ACKEN = ack;       // Responde ACK ou NAK para o escravo
    return received_data;          // Retorna na função, os dados recebidos
}

void load_charmap(){
    unsigned char i;

    for (i = 0; i < 255; i++){
        charmap[i] = eeprom18_read(i);
    }
}

void display_refresh(char message){
    char mapaddr; 
    //((COD ASCII)-40)*5
    
    if ((message >= 40) && (message <= 90)){
        mapaddr = (message - 40)*5;
        // Exibe imagem
        //Ajusta ponteiro da RAM de display e escreve dados
        i2c_rep_start();
        i2c_write(0xE0); // Escreve o endereço I2C
        i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
        i2c_write(charmap[mapaddr]); // Linha 1, Display 1
        i2c_write(0x00); // Linha 1, Display 2
        i2c_write(charmap[mapaddr+1]); // Linha 2, Display 1
        i2c_write(0x00); // Linha 2, Display 2
        i2c_write(charmap[mapaddr+2]); // Linha 3, Display 1
        i2c_write(0x00); // Linha 3, Display 2
        i2c_write(charmap[mapaddr+3]); // Linha 4, Display 1
        i2c_write(0x00); // Linha 4, Display 2
        i2c_write(charmap[mapaddr+4]); // Linha 5, Display 1
        i2c_write(0x00); // Linha 5, Display 2
        i2c_stop();
    }
    else if(message == 20){
        //Caso o caractere seja espaço, apaga todas as linhas
        i2c_rep_start();
        i2c_write(0xE0); // Escreve o endereço I2C
        i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
        i2c_write(0x00); // Linha 1, Display 1
        i2c_write(0x00); // Linha 1, Display 2
        i2c_write(0x00); // Linha 2, Display 1
        i2c_write(0x00); // Linha 2, Display 2
        i2c_write(0x00); // Linha 3, Display 1
        i2c_write(0x00); // Linha 3, Display 2
        i2c_write(0x00); // Linha 4, Display 1
        i2c_write(0x00); // Linha 4, Display 2
        i2c_write(0x00); // Linha 5, Display 1
        i2c_write(0x00); // Linha 5, Display 2
        i2c_stop();        
    } 
    else {
        //Caso o caractere não seja exibível, acende todas as linhas
        i2c_rep_start();
        i2c_write(0xE0); // Escreve o endereço I2C
        i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
        i2c_write(0xFF); // Linha 1, Display 1
        i2c_write(0x00); // Linha 1, Display 2
        i2c_write(0xFF); // Linha 2, Display 1
        i2c_write(0x00); // Linha 2, Display 2
        i2c_write(0xFF); // Linha 3, Display 1
        i2c_write(0x00); // Linha 3, Display 2
        i2c_write(0xFF); // Linha 4, Display 1
        i2c_write(0x00); // Linha 4, Display 2
        i2c_write(0xFF); // Linha 5, Display 1
        i2c_write(0x00); // Linha 5, Display 2
        i2c_stop();                
    }
    
    //Display setup = Display ON e Blinking OFF
    i2c_start();
    i2c_write(0xE0); //Escreve o endereço
    i2c_write(0x81); //Escreve o dado
    i2c_stop();  
}

void teste_display(){
    //Ajusta ponteiro da RAM de display e escreve dados
    i2c_rep_start();
    i2c_write(0xE0); // Escreve o endereço I2C
    i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0xAA); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0xAA); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0xAA); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0xAA); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0xAA); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_stop();
    //Display setup = Display ON e Blinking OFF
    i2c_start();
    i2c_write(0xE0); //Escreve o endereço
    i2c_write(0x81); //Escreve o dado
    i2c_stop();

    __delay_ms(250);

    //Ajusta ponteiro da RAM de display e escreve dados
    i2c_rep_start();
    i2c_write(0xE0); // Escreve o endereço I2C
    i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x54); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x54); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x54); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x54); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0xFF); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_write(0x00); // Escreve o dado (comando para iniciar ponteiro no zero)
    i2c_stop();
    //Display setup = Display ON e Blinking OFF
    i2c_start();
    i2c_write(0xE0); //Escreve o endereço
    i2c_write(0x81); //Escreve o dado
    i2c_stop();
    
    __delay_ms(250);

}

void setup(void){

    /* CONFIGURAÇÃO DE I/Os */
    TRISD = 0x00;
    PORTD = 0x00;    
    TRISBbits.TRISB0 = 1;       //Define RB0 (SDA) como input
    TRISBbits.TRISB1 = 1;       //Define RB1 (SCL) como input
    
    /* CONFIGURÇÃO E INICIALIZAÇÃO DE I2C*/
    i2c_init();
    
    /* CONFIGURAÇÃO E INICIALIZAÇÃO DE CHIP HT16K33 - DISPLAY */
    //System setup = Normal mode (turn ON the oscillator) = 21h
    i2c_start();
    i2c_write(0xE0); // Escreve o endereço I2C
    i2c_write(0x21); // Escreve o dado
    i2c_stop();
      
    //ROW/INT set = ROW driver output = A0h
    i2c_start();
    i2c_write(0xE0); // Escreve o endereço I2C
    i2c_write(0xA0); // Escreve o dado
    i2c_stop();
    
    //Carregamento do mapa de caracteres 5x7 EEPROM->RAM
    load_charmap();
}


/* P R O G R A M A  P R I N C I P A L */
void main(void) {
    char show_char;
    
    //Executa o setup inicial
    setup();
       
//     while(1){
//         teste_display();
//    }
    
            
//    while(1){
        
//    display_refresh(41);       
        
        
//        __delay_ms(100);
        for (show_char = 40; show_char <= 90; show_char++){
            display_refresh(show_char);
            __delay_ms(2000);
            PORTDbits.RD0 = !PORTDbits.RD0;
        }    
//    }
}
