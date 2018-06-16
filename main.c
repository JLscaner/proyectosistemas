#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "config_modules.h"

void main(void){
    config_modulo_I2C();
    config_UART0();
    config_systick();
    MPUInit();
    int16_t a_x = 0,  a_y = 0, signo=0, cuenta=0;
    float accel_x = 0, accel_y = 0, pos_x=0;
    char res[5]={0};
    while(1){
        a_x = dato(0x3B, 0x3C);
        accel_x = transformar((a_x &= 0x03FF), 19.6);
        a_y = dato(0x3D, 0x3E);
        accel_y = transformar((a_y &= 0x03FF), 19.6);
        pos_x = Algoritmo_Posicion(accel_x, cuenta);
        signo = ftoa(pos_x ,res, 2);
        if(signo==1){
            tx_UART0('-');
        }
        tx_msj_UART0(res);
        tx_UART0(0x0D);
        tx_UART0(0x0A);
        if(!(NVIC_ST_CTRL_R & (1<<16))){
            cuenta++;
        }
    }
}

/*        a_z = dato(0x3F, 0x40);
        accel_z = transformar((a_z&=0x03FF), 19.6);
        g_x = dato(0x43, 0x44);
        gyro_x = transformar((g_x&=0x03FF), 250);
        g_y = dato(0x45, 0x46);
        gyro_y = transformar((g_y&=0x03FF), 250);
        g_z = dato(0x47, 0x48);
        gyro_z = transformar((g_z&=0x03FF), 250);
 *
 */
