#include "p33Fxxxx.h"
#include "string.h"
#include "lcd_driver.h"


void delayus(int us)
{
    int i;
    for(i = 0; i < us; i++)
    {
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    }
}

void delayms(int ms)
{
    int i;
    for(i = 0; i < ms; i++)
    {
        delayus(1000);
    }
}


void LCD_write_data(int data)
{	
    delayus(5);
    PORTB = (PORTB & 0x0FFF) | (data<<12);
    delayus(5);
    
    LCD_E = 1; delayus(50); LCD_E = 0;
}

void LCD_write_byte(int byte)
{
    LCD_write_data(byte >> 4);
	LCD_write_data(byte & 0x0F);

    delayms(3);
}

void LCD_write_char(int character)
{
    LCD_RS = 1;
    delayus(5);
    LCD_write_byte(character);
    delayus(5);
    LCD_RS = 0;
    
	delayms(1);
}

void LCD_printf(char *text)
{
 	int i;
	for (i = 0; i < strlen(text); i++)	
    {
        LCD_write_char(text[i]);
    }
}

void LCD_clear(void)
{
	LCD_write_byte(0x01);
}

void LCD_goto(int line, int col)
{
    char byte = 0;
    switch(line)
    {
        case 1: byte = 0x80; break;
        case 2: byte = 0xC0; break;
        case 3: byte = 0x94; break;
        case 4: byte = 0xD4; break;
    }
    
    LCD_write_byte(byte + col - 1);
}

void LCD_line(int line)
{
	LCD_goto(line, 1);
}

void LCD_init(void)
{
    PORTB = 0x0000;
    
	delayms(50);
	
    // initialize
	delayms(20);
	LCD_write_data(0x3);
    delayms(20);
	LCD_write_data(0x3);
    delayms(20);
    LCD_write_data(0x3);
    delayms(20);
	LCD_write_data(0x2);
    delayms(10);
    
    LCD_write_byte(0x28);
    LCD_write_byte(0x08);
    LCD_clear();
    LCD_write_byte(0x06);
    LCD_write_byte(0x0C);
    
	delayms(5);
}