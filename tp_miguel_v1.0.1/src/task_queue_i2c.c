/* Copyright 2017-2018, Eric Pernia
 * All rights reserved.
 *
 * This file is part of sAPI Library.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*==================[inclusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// sAPI header
#include "sapi.h"
#include "FreeRTOSConfig.h"

/* Demo includes. */
#include "supporting_functions.h"

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/

// C++ version 0.4 char* style "itoa":
// Written by Lukás Chmela
// Released under GPLv3.

char* itoa(int value, char* result, int base) {
   // check that the base if valid
   if (base < 2 || base > 36) { *result = '\0'; return result; }

   char* ptr = result, *ptr1 = result, tmp_char;
   int tmp_value;

   do {
      tmp_value = value;
      value /= base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
   } while ( value );

   // Apply negative sign
   if (tmp_value < 0) *ptr++ = '-';
   *ptr-- = '\0';
   while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
   }
   return result;
}

/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea
void tarea_lcd( void* pvParameters);
void tarea_tem(void * pvParameters);
void tarea_hum(void * pvParameters);
static void tarea_led(void* pvParameters);
uint16_t leer_temperatura();
uint16_t leer_humedad();
//float leer_temperatura();

QueueHandle_t queue_datos;
QueueHandle_t queue_hum;

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void )
{

    // ---------- CONFIGURACIONES ------------------------------
    // Inicializar y configurar la plataforma
    boardConfig();

    // UART for debug messages
    debugPrintConfigUart( UART_USB, 115200 );

    debugPrintlnString( "Trabajo Final RTOS / Protocolos de Comunicación en SE\n" );
    
    // Inicializacion de AnalogIO
    adcConfig(ADC_ENABLE); // ADC

    // Crear cola en freeRTOS
    //queue_datos = xQueueCreate(3, sizeof(float));
    queue_datos = xQueueCreate(5, sizeof(uint16_t));
    if (queue_datos == NULL) {
        debugPrintlnString("La cola no puede ser creada");
    }
    queue_hum = xQueueCreate(5, sizeof(uint16_t));
    if (queue_hum == NULL) {
        debugPrintlnString("La cola no puede ser creada");
    }

    // Crear tareas en freeRTOS
    
    // Creacion de la tarea que imprime en el LCD
    xTaskCreate(
    	tarea_lcd,                     	// Funcion de la tarea a ejecutar
		//( const char * )
		"TAREA LCD",   					// Nombre de la tarea como String amigable para el usuario
		configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea, se puede usar tambien 200
		0,                          	// Parametros de tarea, se puede usar NULL
		tskIDLE_PRIORITY+1,         	// Prioridad de la tarea -> Queremos que este un nivel encima de IDLE, se puede usar 1
		0                          		// Puntero a la tarea creada en el sistema, se puede usar NULL
    );

    // Creacion de la tarea que realiza lectura de temperatura
    xTaskCreate(
        tarea_tem,                    	 // Funcion de la tarea a ejecutar
        //( const char * )
		"TAREA TEMPERATURA",   			// Nombre de la tarea como String amigable para el usuario
        configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea, se puede usar tambien 200
        0,                          	// Parametros de tarea, se puede usar NULL
        tskIDLE_PRIORITY+1,         	// Prioridad de la tarea -> Queremos que este un nivel encima de IDLE, se puede usar 1
        0                          		// Puntero a la tarea creada en el sistema, se puede usar NULL
    );
    
    // Creacion de la tarea que realiza lectura de humedad
    xTaskCreate(
        tarea_hum,                    	 // Funcion de la tarea a ejecutar
        //( const char * )
		"TAREA HUMEDAD",   			// Nombre de la tarea como String amigable para el usuario
        configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea, se puede usar tambien 200
        0,                          	// Parametros de tarea, se puede usar NULL
        tskIDLE_PRIORITY+1,         	// Prioridad de la tarea -> Queremos que este un nivel encima de IDLE, se puede usar 1
        0                          		// Puntero a la tarea creada en el sistema, se puede usar NULL
    );

    // Creacion de la tarea que mantiene un led indicador de funcionamiento
    xTaskCreate(
        tarea_led,                     // Funcion de la tarea a ejecutar
        ( const char * )"TAREA LED",   	// Nombre de la tarea como String amigable para el usuario
        configMINIMAL_STACK_SIZE, 	// Cantidad de stack de la tarea, se puede usar tambien 200
        0,                          	// Parametros de tarea, se puede usar NULL
        tskIDLE_PRIORITY+1,         	// Prioridad de la tarea -> Queremos que este un nivel encima de IDLE, se puede usar 1
        0                          		// Puntero a la tarea creada en el sistema, se puede usar NULL
    );
    
    // Iniciar scheduler
    vTaskStartScheduler(); // Enciende tick | Crea idle y pone en ready | Evalua las tareas creadas | Prioridad mas alta pasa a running

    // ---------- REPETIR POR SIEMPRE --------------------------
    while( TRUE )
    {
        // Si cae en este while 1 significa que no pudo iniciar el scheduler
    }

    // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
    // directamenteno sobre un microcontroladore y no es llamado por ningun
    // Sistema Operativo, como en el caso de un programa para PC.
    return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/




//-----------------------------------------------------------
//---------------------- T E M P ----------------------------
//-----------------------------------------------------------
// Implementacion de funcion de la tarea
void tarea_tem(void * pvParameters) {
    
    // ---------- CONFIGURACIONES ------------------------------
    uint16_t temperatura_actual;
    //float temperatura_actual;

    // ---------- REPETIR POR SIEMPRE --------------------------
    while(1) {
        temperatura_actual = leer_temperatura();
        xQueueSend(queue_datos, &temperatura_actual, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//-----------------------------------------------------------
//---------------------- H U M ------------------------------
//-----------------------------------------------------------
// Implementacion de funcion de la tarea
void tarea_hum(void * pvParameters) {

    // ---------- CONFIGURACIONES ------------------------------
    uint16_t humedad_actual;
    //float temperatura_actual;

    // ---------- REPETIR POR SIEMPRE --------------------------
    while(1) {
        humedad_actual = leer_humedad();
        xQueueSend(queue_hum, &humedad_actual, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//-----------------------------------------------------------
//------------------------ L C D ----------------------------
//-----------------------------------------------------------
// Implementacion de funcion de la tarea
void tarea_lcd( void* pvParameters)
{
    // ---------- CONFIGURACIONES ------------------------------
    uint16_t temperatura;
    uint16_t humedad;
    //float temperatura;
    //char buffer[20];
    static char buffer[20];

    i2cInit(I2C0, 100000); 	// Inicialización del protocolo
    
    // Inicializar LCD de 16x2 (caracteres x lineas) con cada caracter de 5x2 pixeles
    lcdInit(16, 2, 5, 8);
    
    // Apaga el cursor
    lcdCursorSet(LCD_CURSOR_OFF);
    
    // ---------- REPETIR POR SIEMPRE --------------------------
    while( TRUE )
    {
        if (xQueueReceive(queue_datos, &temperatura, portMAX_DELAY) == pdPASS) {
        	lcdClear();
            lcdGoToXY(0, 0); // Poner cursor en 0, 0
            lcdSendStringRaw("Tem: ");
        	//sprintf(buffer,"%3.2f",temperatura);
            lcdGoToXY(0, 7);
            itoa( temperatura, buffer, 10 );
            lcdSendStringRaw(buffer);
            //lcdSendStringRaw("hola");
            vPrintStringAndNumber("Temperatura = ", temperatura);
        }
            else{
            	vPrintStringAndNumber("No hay datos para mostrar \r\n ", temperatura);

        }
        if (xQueueReceive(queue_hum, &humedad, portMAX_DELAY) == pdPASS) {
        	//lcdClear();
            lcdGoToXY(0, 1); // Poner cursor en 0, 0
            lcdSendStringRaw("Hum: ");
        	//sprintf(buffer,"%3.2f",temperatura);
            lcdGoToXY(1, 7);
            itoa( humedad, buffer, 10 );
            lcdSendStringRaw(buffer);
            //lcdSendStringRaw("hola");
            vPrintStringAndNumber("Temperatura = ", temperatura);
        }
            else{
            	vPrintStringAndNumber("No hay datos para mostrar \r\n ", temperatura);

        }
    }
}



//-----------------------------------------------------------
//------------------------ L E D ----------------------------
//-----------------------------------------------------------
// Tarea de led testigo o indicador de funcionamiento
static void tarea_led( void * pvParameters )
{
// Blink cada 200ms.
// Función extraida y adaptada de https://www.freertos.org/a00127.html
const TickType_t xDelay = 200 / portTICK_PERIOD_MS;

    for( ;; )
    {
    	// Prendo el led azul
    	gpioWrite( LEDB, ON );
    	vTaskDelay( xDelay );
    	// Apago el led azul
    	gpioWrite( LEDB, OFF );
        vTaskDelay( xDelay );
    }
}

//-----------------------------------------------------------
//---------------- S E N S O R    T E M P -------------------
//-----------------------------------------------------------
// Funcion para la lectura del sensor de temperatura
uint16_t leer_temperatura() {
    
    // Variable para almacenar el valor leido del ADC CH1
    uint16_t data;

    // Leo la Entrada Analogica AI0 - ADC0 CH1
    data = adcRead(CH1);
    
    return data;
}

//-----------------------------------------------------------
//---------------- S E N S O R    H U M ---------------------
//-----------------------------------------------------------
// Funcion para la lectura del sensor de temperatura
uint16_t leer_humedad() {

    // Variable para almacenar el valor leido del ADC CH2
    uint16_t data;

    // Leo la Entrada Analogica AI0 - ADC0 CH2
    data = adcRead(CH2);

    return data;
}
/*==================[fin del archivo]========================================*/
