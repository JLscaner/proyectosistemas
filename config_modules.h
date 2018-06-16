/*
 * config_modules.h
 *
 *  Created on: 9 jun. 2018
 *      Author: Jorge
 */

#ifndef CONFIG_MODULES_H_
#define CONFIG_MODULES_H_
#include "config_modules.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "tm4c123gh6pm.h"

void config_modulo_I2C(void);
uint16_t leer_I2C_MPU6050_2bytes(uint8_t direccion_esclavo);
uint32_t escribir_I2C_MPU6050 (uint8_t direccion_esclavo ,uint8_t opecode);
uint8_t leer_I2C_MPU6050_1byte(uint8_t direccion_esclavo);
uint32_t I2C_Send2(int8_t slave, uint8_t data1, uint8_t data2);
void MPUInit(void);
void config_UART0(void);
char rx_UART0(void);
void tx_UART0(char c);
void rx_msj_UART0(uint32_t long_buffer, char buffer[]);
void tx_msj_UART0(char msj[]);
void config_systick(void);
uint16_t dato(uint8_t regH, uint8_t regL);
float transformar(uint16_t medicion , uint16_t rango);
void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
uint16_t ftoa(float n, char *res, int afterpoint);
float Algoritmo_Posicion( float accel, uint32_t cuenta);

#endif /* CONFIG_MODULES_H_ */
