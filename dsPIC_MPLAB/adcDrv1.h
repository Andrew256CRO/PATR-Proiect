#ifndef __ADCDRV1_H__
#define __ADCDRV1_H__ 

// External Functions
extern void initAdc1(void);
extern void initTmr3();
extern void __attribute__((__interrupt__)) _ADC1Interrupt(void);

#endif
