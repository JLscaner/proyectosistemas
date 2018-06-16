/*
 * config_modules.c
 *
 *  Created on: 9 jun. 2018
 *      Author: Jorge
 */
#include "config_modules.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "tm4c123gh6pm.h"

//Configuration of module I2C
#define NumeroIntentosMax          5             // Numero maximo de intentos al enviar
#define I2C_MCS_ACK       0x00000008             // Acknowledge prendido
#define I2C_MCS_DATACK    0x00000008             // Acknowledge data
#define I2C_MCS_ADRACK    0x00000004             // Acknowledge Address
#define I2C_MCS_STOP      0x00000004             // Generar Stop
#define I2C_MCS_START     0x00000002             // Generate Start
#define I2C_MCS_ERROR     0x00000002             // Error
#define I2C_MCS_RUN       0x00000001             // I2C prendido
#define I2C_MCS_BUSY      0x00000001             // I2C ocupado
#define I2C_MCR_MFE       0x00000010             // I2C Master funcion prendido

void config_modulo_I2C(void){
    SYSCTL_RCGCI2C_R |= (1<<0); //se activa la señal de reloj del modulo 0 I2C
    SYSCTL_RCGC1_R |= (1<<12); //se activa el apoyo para funciones alternativas de I2C
    SYSCTL_RCGC2_R |= (1<<1); // se activa la señal de reloj para PORTB
    while((SYSCTL_PRGPIO_R &= (1<<1)) == 0);// se espera a que este realmente activa la señal de reloj

    GPIO_PORTB_AFSEL_R |= (1<<2)|(1<<3); //activamos las funciones alternativas de los pines PB2 y PB3
    GPIO_PORTB_DEN_R |= (1<<2)|(1<<3); //se habilitan los pines PB2 y PB3 como señales digitales
    GPIO_PORTB_ODR_R |= (1<<3); //activamos el open_drain del pin PB2 que es mi señal de data
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00003300; //se configure los pines PB2 y PB3 como I2C
    GPIO_PORTB_AMSEL_R &= ~((1<<2)|(1<<3)); //se aisla la parte analógica de estos pines
    I2C0_MCR_R = I2C_MCR_MFE; //master function enable
    I2C0_MTPR_R = 24; //se configura para 100 kbps de clock
                      // 20*(TPR+1)*20ns = 10us, con TPR=24
}

uint16_t leer_I2C_MPU6050_2bytes(uint8_t direccion_esclavo){
    /*
     * Esta función recibe dos bytes y retorna un numero
     * sin signo de 16 bits que representa la medicion del sensor
     * pero aun falta modificar este dato para que sea real
     */
    uint8_t datoH=0,datoL=0;
    uint8_t intento=1;
    //I2C1_MCS_R= (I2C1_MCS_R & ~0x17)| 0x1;                    // Bit RUN 1
    do{                                                         // MCS modo lectura
        while(I2C0_MCS_R & I2C_MCS_BUSY);                         // Esperar I2C
        I2C0_MSA_R =  (I2C0_MSA_R & ~0xFF) +                    // Limpiamos registro
                (direccion_esclavo<<1);                         // Direccion de esclavo +W/R=1
        I2C0_MSA_R|=0x1;                                        // Como lectura
        I2C0_MCS_R=(0
                |I2C_MCS_ACK                                    // ACK positivo activado
                |I2C_MCS_START                                  // Generar start/restart
                |I2C_MCS_RUN);                                  // Modo master prendido
        while(I2C0_MCS_R & I2C_MCS_BUSY);                       // Esperar I2C
        datoH=(I2C0_MDR_R & 0xFF);                              // Recibo Hight Byte del sensor
        I2C0_MCS_R=(0
                |I2C_MCS_STOP                                   // ACK positivo activado
                //|I2C_MCS_START                                // Generar start/restart
                |I2C_MCS_RUN);                                  // Modo master prendido
        while(I2C0_MCS_R&I2C_MCS_BUSY);                         // Esperar I2C
        datoL=(I2C0_MDR_R&0xFF);                                // Recibo Hight Byte del sensor
        intento++;                                              // Aumentar el contador
    }while(((I2C0_MCS_R&(I2C_MCS_ADRACK|I2C_MCS_ERROR))!=0)&&(intento<=NumeroIntentosMax));//Mientras sea menor el intento
    return (datoH<<8)+datoL;                                        // Devolvemos valor medido
}// fin leer_I2C

uint32_t escribir_I2C_MPU6050 (uint8_t direccion_esclavo ,uint8_t opecode){
    /* Esta función los Measurement code = opecode
     * según datasheet de MPU6050
     */
    while(I2C0_MCS_R & I2C_MCS_BUSY);                                // Esperar a I2C libre
    I2C0_MSA_R =  (I2C0_MSA_R & ~0xFF) + (direccion_esclavo<<1);    // Direccion de esclavo con W/R=0
    I2C0_MSA_R&=~0x1;                                               // Modo escritura
    I2C0_MDR_R=opecode&0xFF;                                        // Enviamos opecode
    I2C0_MCS_R =(0
            |I2C_MCS_STOP                                           // Generar un stop
            |I2C_MCS_START                                          // Generar Start/ Restart
            |I2C_MCS_RUN);                                          // Prender modo Master
    while(I2C0_MCS_R&I2C_MCS_BUSY);                                 // Esperar a I2C libre
    return (I2C0_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));  // Retornar algun error
}//fin escribir_I2C_BH1750

uint8_t leer_I2C_MPU6050_1byte(uint8_t direccion_esclavo){
  int retryCounter = 1;
  do{
    while(I2C0_MCS_R&I2C_MCS_BUSY){}; // wait for I2C ready
    I2C0_MSA_R = (direccion_esclavo<<1)&0xFE;    // MSA[7:1] is slave address
    I2C0_MSA_R |= 0x01;              // MSA[0] is 1 for receive
    I2C0_MCS_R = (0
                        // & ~I2C_MCS_ACK     // negative data ack (last byte)
                         | I2C_MCS_STOP     // generate stop
                         | I2C_MCS_START    // generate start/restart
                         | I2C_MCS_RUN);    // master enable
    while(I2C0_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
    retryCounter = retryCounter + 1;        // increment retry counter
  }                                         // repeat if error
  while(((I2C0_MCS_R&(I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= NumeroIntentosMax));
  return (I2C0_MDR_R&0xFF);          // usually returns 0xFF on error
}

uint32_t I2C_Send2(int8_t slave, uint8_t data1, uint8_t data2){
  while(I2C0_MCS_R&I2C_MCS_BUSY){};// wait for I2C ready
  I2C0_MSA_R = (slave<<1)&0xFE;    // MSA[7:1] is slave address
  I2C0_MSA_R &= ~0x01;             // MSA[0] is 0 for send
  I2C0_MDR_R = data1&0xFF;         // prepare first byte
  I2C0_MCS_R = (0
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                    //   & ~I2C_MCS_STOP    // no stop
                       | I2C_MCS_START    // generate start/restart
                       | I2C_MCS_RUN);    // master enable
  while(I2C0_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // check error bits
  if((I2C0_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
    I2C0_MCS_R = (0                // send stop if nonzero
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // stop
                     //  & ~I2C_MCS_START   // no start/restart
                     //  & ~I2C_MCS_RUN    // master disable
                        );
                                          // return error bits if nonzero
    return (I2C0_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
  }
  I2C0_MDR_R = data2&0xFF;         // prepare second byte
  I2C0_MCS_R = (0
                      // & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // generate stop
                      // & ~I2C_MCS_START   // no start/restart
                       | I2C_MCS_RUN);    // master enable
  while(I2C0_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // return error bits
  return (I2C0_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
}

//Configuration of MPU6050
void MPUInit(void)
{   I2C_Send2(0x68,0x19,0x07);
    //es muy recomendable que el dispositivo esté configurado para usar uno de los giroscopios (o una fuente de reloj externa)
    //como referencia de reloj para una mejor estabilidad
    I2C_Send2(0x68,0x6B,0x01);
    I2C_Send2(0x68,0x1A,0x00);
    I2C_Send2(0x68,0x1B,0x00);//rango del gyroscopio de -250°/s a +250°/s
    I2C_Send2(0x68,0x1C,0x00);//rango del acelerómetro de -2g a +2g
    I2C_Send2(0x68,0x38,0x00);
    }


//Configuration of UART0
void config_UART0(void){
    SYSCTL_RCGC1_R |= (1<<0);
    while(!(SYSCTL_PRUART_R & (1<<0)));
    SYSCTL_RCGC2_R |= (1<<0);
    while(!(SYSCTL_PRGPIO_R & (1<<0)));

    GPIO_PORTA_AMSEL_R &= ~((1<<0)|(1<<1));
    GPIO_PORTA_AFSEL_R |= (1<<0)|(1<<1);
    GPIO_PORTA_PUR_R &= ~((1<<0)|(1<<1));
    GPIO_PORTA_PDR_R &= ~((1<<0)|(1<<1));
    GPIO_PORTA_DIR_R &= ~(1<<0);
    GPIO_PORTA_DIR_R |= (1<<1);
    GPIO_PORTA_DEN_R |= (1<<0)|(1<<1);
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00)|0x00000011;

    UART0_CTL_R &= ~(1<<0);
    UART0_IBRD_R = (UART0_IBRD_R & ~0xFFFF)+104;
    UART0_FBRD_R = (UART0_FBRD_R & ~0x3F)+11;
    UART0_LCRH_R = (UART0_LCRH_R & ~0xFF)|0x70;
    UART0_CTL_R |= (1<<0);
}

//Functions to transmit and receive data by UART0 and PuTTY
//Receive one byte
char rx_UART0(void){
    while(UART0_FR_R & (1<<4));//while this bit is in 1, there isn't data in the FIFO of reception
    return (UART0_DR_R & 0xFF);
}
//Transmit one byte
void tx_UART0(char c){
    while(UART0_FR_R & (1<<5));//while this bit is in 1, there isn't data in the FIFO of transmission
    UART0_DR_R = c;
}
//Receive a string of characters of a certain length
void rx_msj_UART0(uint32_t long_buffer, char buffer[]){
    char c;
    uint32_t i=0;
    while((c = rx_UART0())!='\r'){
        if(i < long_buffer - 1){
            buffer[i]=c;
            i++;
        }
    }
    buffer[i]='0';
}
//Transmit a string of characters
void tx_msj_UART0(char msj[]){
    uint8_t i=0;
    while(msj[i]!='\0'){
        tx_UART0(msj[i]);
        i++;
    }
}
//Commands for enter and new line: 0x0D and 0x0A

//Configuration of SysTick to get variation of time (10ms)
void config_systick(void){
    NVIC_ST_CTRL_R &= ~(1<<0);
    NVIC_ST_RELOAD_R = (NVIC_ST_RELOAD_R & ~0xFFFFFF)+159999;
    NVIC_ST_CURRENT_R |= 0x07;
    NVIC_ST_CTRL_R = (NVIC_ST_CTRL_R & ~0x7)+5;
}

uint16_t dato(uint8_t regH, uint8_t regL){
    uint8_t valor_1, valor_2, error;
    uint16_t x;
    error = escribir_I2C_MPU6050 (0x68 ,regH);
    valor_1 = leer_I2C_MPU6050_1byte(0x68);
    escribir_I2C_MPU6050 (0x68 ,regL);
    valor_2 = leer_I2C_MPU6050_1byte(0x68);
    x = (valor_1<<8) + valor_2;
    return x;
}

float transformar(uint16_t medicion , uint16_t rango){
    float valor_real;
    float rangoflotante = rango;
    float medicionflotante = medicion;
    valor_real = (rangoflotante*(medicionflotante - 1023/2))/(1023/2);
    return valor_real;
}

// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
   int i = 0;
   while (x)
   {
       str[i++] = (x%10) + '0';
       x = x/10;
   }

   // If number of digits required is more, then
   // add 0s at the beginning
   while (i < d)
       str[i++] = '0';

   reverse(str, i);
   str[i] = '\0';
   return i;
}
// Converts a floating point number to string. (char *res === array[])
uint16_t ftoa(float n, char *res, int afterpoint)
{
   //if the number is negative:
    uint16_t negativo=0;
    if(n<0){
       n=n*-1;
       negativo=1;
   }


   // Extract integer part
   int ipart = (int)n;

   // Extract floating part
   float fpart = n - (float)ipart;

   // convert integer part to string
   int i = intToStr(ipart, res, 0);

   // check for display option after point
   if (afterpoint != 0)
   {
       res[i] = '.';  // add dot

       // Get the value of fraction part upto given no.
       // of points after dot. The third parameter is needed
       // to handle cases like 233.007
       fpart = fpart * pow(10, afterpoint);

       intToStr((int)fpart, res + i + 1, afterpoint);
   }
   return negativo;
}

float Algoritmo_Posicion( float accel, uint32_t cuenta){
    float tiempo=0, vel=0, pos=0;
    while(cuenta!=0){
        tiempo = tiempo + 0.01;
        cuenta--;
    }
    vel = vel + accel*tiempo;
    pos = pos + vel*tiempo+ 0.5*accel*(tiempo*tiempo);
    return pos;
}





























