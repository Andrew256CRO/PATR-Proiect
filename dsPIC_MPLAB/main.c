/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

/* Demo application includes. */
#include "BlockQ.h"
#include "crflash.h"
#include "blocktim.h"
#include "integer.h"
#include "comtest2.h"
#include "partest.h"
//#include "lcd.h"
#include "timertest.h"
#include "ds18s20.h"

// Includes proprii
#include "new_lcd.h"
#include "new_serial.h"
#include "libq.h"
//#include "serial.h"

/* Demo task priorities. */
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainCOM_TEST_PRIORITY				( 2 )

/* The check task may require a bit more stack as it calls sprintf(). */
#define mainCHECK_TAKS_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )

/* The execution period of the check task. */
#define mainCHECK_TASK_PERIOD				( ( portTickType ) 3000 / portTICK_RATE_MS )

/* The number of flash co-routines to create. */
#define mainNUM_FLASH_COROUTINES			( 5 )

/* Baud rate used by the comtest tasks. */
//#define mainCOM_TEST_BAUD_RATE				( 19200 )
#define mainCOM_TEST_BAUD_RATE				( 9600 )

// Definire lungime coada UART1
#define comBUFFER_LEN						( 10 )

/* We should find that each character can be queued for Tx immediately and we
don't have to block to send. */
#define comNO_BLOCK					( ( portTickType ) 0 )

/* The Rx task will block on the Rx queue for a long period. */
#define comRX_BLOCK_TIME			( ( portTickType ) 0xffff )

/* The LED used by the comtest tasks.  mainCOM_TEST_LED + 1 is also used.
See the comtest.c file for more information. */
#define mainCOM_TEST_LED					( 6 )

/* The frequency at which the "fast interrupt test" interrupt will occur. */
#define mainTEST_INTERRUPT_FREQUENCY		( 20000 )

/* The number of processor clocks we expect to occur between each "fast
interrupt test" interrupt. */
#define mainEXPECTED_CLOCKS_BETWEEN_INTERRUPTS ( configCPU_CLOCK_HZ / mainTEST_INTERRUPT_FREQUENCY )

/* The number of nano seconds between each processor clock. */
#define mainNS_PER_CLOCK ( ( unsigned short ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Dimension the buffer used to hold the value of the maximum jitter time when
it is converted to a string. */
#define mainMAX_STRING_LENGTH				( 20 )

// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC);
// Enable Clock Switching and Configure
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF);		// FRC + PLL
//_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);		// XT + PLL
_FWDT(FWDTEN_OFF); 		// Watchdog Timer Enabled/disabled by user software
/*
 * Setup the processor ready for the demo.
 */
static void prvSetupHardware( void );

/* The queue used to send messages to the LCD task. */
static xQueueHandle xUART1_Queue;
float tempe;
char temperature[8];

void initializareIntrerupere0(){
 _INT0IF = 0; // Resetem flagul coresp. intreruperii INT0
 _INT0IE = 1; // Se permite lucrul cu întreruperea INT0
 _INT0EP = 1; // Se stabileote pe ce front se genereaza INT0
}
void init_PWM1(void);

void TaskTemperature(void *params)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 100;

    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        tempe = ds1820_read();

        int zec = (int)(tempe * 100);
        sprintf(temperature, "%d.%02d", zec / 100, zec % 100);


		static char last_temp[8] = "";
		if (strcmp(last_temp, temperature) != 0) {
    		LCD_Goto(1,11);
    		LCD_printf("      "); 
            LCD_Goto(1,11);
            LCD_printf(temperature);
            strcpy(last_temp, temperature);
}

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskPWM(void *params) {
    portTickType xLastWakeTime = xTaskGetTickCount();
    const portTickType xFrequency = 1000;

    char voltage_text[16];
    float Vref = 3.3;
    float duty, Vout;

    for (;;) {
        duty = (float)P1DC3 / (float)P1TPER;
        Vout = duty * Vref;

        int zec = (int)(Vout * 100);  // de ex: 1.25V -> 125
        sprintf(voltage_text, "Voltage:  %d.%02dV", zec / 100, zec % 100);

        LCD_Goto(3, 1);
        LCD_printf("             "); // stergem linia
        LCD_Goto(3, 10);
        LCD_printf(voltage_text);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


int main( void )
{
	prvSetupHardware();
	xTaskCreate(TaskTemperature, "Temp", configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(TaskPWM, (signed portCHAR *) "pwm", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL);
	TRISBbits.TRISB10 = 0;
 	initPLL();
	initAdc1();
 	init_PWM1();
	TRISB = 0x0000;
	_TRISB7 = 1; // RB7 este setat ca intrare
	PORTB = 0xF000;
	initializareIntrerupere0();
	/* Finally start the scheduler. */
	vTaskStartScheduler();

	while(1);
}



/*-----------------------------------------------------------*/

void init_PWM1()
{
//lucram cu RB10 -> PWM1H3
 P1TCONbits.PTOPS = 0; // Timer base output scale
 P1TCONbits.PTMOD = 0; // Free running
 P1TMRbits.PTDIR = 0; // Numara in sus pana cand timerul = perioada
 P1TMRbits.PTMR = 0; // Baza de timp
 /*Tcy=25ns;
 T=10ms;
 fu=20%;
 Perioada obtinuta trebuie sa fie mai mica decat 65536/2.
 P1TPER=T/Tcy=10ms/25ns=...=400000;
 cu prescaler de 64=>400000/64=6250<65536/2=P1TPER;
 fu=(d/T)*100 => d=(fu*T)/100 =...=2ms;
 P1DC1=(d/Tcy)*2=...=160000
 cu prescaler 64=> 160000/64=2500<65536/2=P1DC1*/

 P1DC3 = 0x9C4;
 P1TPER= 0x186A;

 _PWM1MD = 0; //PWM1 module is enabled

 PWM1CON1bits.PMOD3 = 1; // Canalele PWM3H si PWM3L sunt independente
 PWM1CON1bits.PEN3H = 1; // Pinul PWM1H setat pe iesire PWM

 P1TCONbits.PTCKPS=0b11;

 PWM1CON2bits.UDIS = 0; // 1 da disable la update pentru semnale pwm

 P1TCONbits.PTEN = 1; /* Enable the PWM Module */
}

void initPLL(void)
{
// Configure PLL prescaler, PLL postscaler, PLL divisor
	PLLFBD = 41; 		// M = 43 FRC
	//PLLFBD = 30; 		// M = 32 XT
	CLKDIVbits.PLLPOST=0; 	// N1 = 2
	CLKDIVbits.PLLPRE=0; 	// N2 = 2

// Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
	__builtin_write_OSCCONH(0x01);	// FRC
	//__builtin_write_OSCCONH(0x03);	// XT
	__builtin_write_OSCCONL(0x01);

// Wait for Clock switch to occur
	while (OSCCONbits.COSC != 0b001);	// FRC
	//while (OSCCONbits.COSC != 0b011);	// XT

// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1) {};
}

static void prvSetupHardware( void )
{

	ADPCFG = 0xFFFF;				//make ADC pins all digital - adaugat
	vParTestInitialise();
	ONE_WIRE_PIN=1;
	output_float();
	_TRISB8 = 1;
	_CN6PUE=1;
	initPLL();
	LCD_init();
	LCD_line(1);
	LCD_printf("Temp:");
	LCD_line(2);
	LCD_printf("Mode:");
	LCD_line(3);
	//LCD_printf("Voltage:");
	LCD_line(4);
	LCD_printf("LastComm:");
	LCD_Goto(2,11);
	//LCD_printf(
	LCD_Goto(3,11);
	//LCD_printf
	LCD_Goto(4,11);
	//LCD_printf
	// Initializare interfata UART1
	xSerialPortInitMinimal( mainCOM_TEST_BAUD_RATE, comBUFFER_LEN );
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* Schedule the co-routines from within the idle task hook. */
	vCoRoutineSchedule();
}
/*-----------------------------------------------------------*/