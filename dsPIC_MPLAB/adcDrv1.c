#if defined(__dsPIC33F__)
#include "p33fxxxx.h"
#endif

#include "adcDrv1.h"

#define  SAMP_BUFF_SIZE	 		10		// Dimensiunea buffer-ului in care se salveaza rezultatele conversiei

//INITIALIZARE ADC PENTRU SCANARE CANAL AN4(RB2)
void initAdc1(void)
{
		AD1CON1bits.AD12B = 1; // conversie AD pe 12 biti
		AD1CON1bits.FORM = 0;  // rezultat conversie integer
		AD1CON1bits.SSRC = 2;  // timerul 3 starteaza conversia
		AD1CON1bits.ASAM = 1;  // incepe esantionarea noii valori imediat dupa terminarea
							   // unei conversii

		AD1CON2bits.CSCNA = 1; // scaneaza intrarile pe CH0+ in timpul achizitiei A
		AD1CON2bits.CHPS = 0;  // converteste doar CH0
		AD1CON2bits.SMPI = 0;  // incrementeaza adresa DMA dupa terminarea fiecarei
							   // conversii

		AD1CON3bits.ADRC = 0;  // foloseste ceasul magistralei
		AD1CON3bits.ADCS = 63; // Timpul necesar unei conversii este de 19.2 us
							   // Ceasul pentru conversia AD are formula Tad=Tcy*(adcs+1)
							   // Tad=Tcy*(adcs+1)=(1/40)*64=1.6us

// Se seteaza intrarile analogice
		AD1CSSLbits.CSS5 = 1;  // Selectam intrarea analogica AN5(RB3) pentru a fi scanata

// Scriem registrul de configurare al portului
// Se va folosi doar registrul low al portului de configurare deoarece dsPIC33fj128MC802
// nu are implementati mai mult de 6 pini pentru ADC
		AD1PCFGL=0xFFFF;       // Setam toti pinii portului ADC1 pe modul digital,
							   // si activeaza citirea la intrarea portului
		AD1PCFGLbits.PCFG5 = 0;// Setam pinul AN5(RB3) pe intrare analogica,
							   // ADC verifica voltajele pe acel pin (achizitie AD)
		IFS0bits.AD1IF = 0;      // Reseteaza flag-ul intreruperii convertorului AD
		IPC3bits.AD1IP = 6;      // Seteaza prioritatea intreruperii convertorului AD
	 	IEC0bits.AD1IE = 1;      // Permite intreruperea convertorului AD

		AD1CON1bits.ADON = 1;
}

// Timer-ul 3 este setat sa starteze conversia AD la fiecare 125 microsecunde (8Khz Rate).
/*void initTmr3() 
{
        TMR3 = 0;
        PR3 = 4999;
        T3CONbits.TON = 1;	// Start Timer 3
}
*/
int  ain4Buff[SAMP_BUFF_SIZE];
int  sampleCounter=0;

// rutina de tratare a intreruperii convertorului AD
void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void)
{
	ain4Buff[sampleCounter++]=ADC1BUF0; 
	if(sampleCounter==SAMP_BUFF_SIZE)
		sampleCounter=0;
    IFS0bits.AD1IF = 0;		// Achita intreruperea convertorului AD
}
