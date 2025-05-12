#define LCD_E   _LATB4
#define LCD_RS  _LATB6
#define LCD_RW  _LATB5
// D4 D5 D6 D7 = RB12 RB13 RB14 RB15

void delayus(int us);
void delayms(int ms);

void LCD_write_data(int data);
void LCD_write_byte(int byte);
void LCD_write_char(int character);
void LCD_printf(char *text);
void LCD_clear(void);
void LCD_goto(int line, int col);
void LCD_line(int line);
void LCD_init(void);
