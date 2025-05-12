#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "new_serial.h"
#include <stdio.h>
#include <string.h>

/* Variabile globale din main.c */
extern volatile int mod_lucru;
extern float tempe;

/* Func?ii auxiliare */
void interogareModLucru(void) {
    const char *text;
    int i;
    if (mod_lucru == 0)
        text = "Mod: automat\r\n";
    else
        text = "Mod: manual\r\n";

    for (i = 0; i < strlen(text); i++) {
        xSerialPutChar(0, text[i], portMAX_DELAY);
    }
}

void comutareModLucru(void) {
    const char *text = "Modul de lucru a fost comutat.\r\n";
    int i;
    mod_lucru = !mod_lucru;

    for (i = 0; i < strlen(text); i++) {
        xSerialPutChar(0, text[i], portMAX_DELAY);
    }
}

void interogareTemperatura(void) {
    char buf[32];
    int i;
    sprintf(buf, "Temperatura: %.2f C\r\n", (double)tempe);

    for (i = 0; i < strlen(buf); i++) {
        xSerialPutChar(0, buf[i], portMAX_DELAY);
    }
}

/* Taskul de meniu */
void vSerialMenuTask(void *pvParameters) {
    char rxChar;
    const char *meniu =
        "\r\n--- Meniu Principal ---\r\n"
        "m - Interogare mod de lucru\r\n"
        "c - Comutare mod automat/manual\r\n"
        "t - Interogare temperatura\r\n"
        "h - Afisare ajutor\r\n"
        "------------------------\r\n";
    int i;

    for (i = 0; i < strlen(meniu); i++) {
        xSerialPutChar(0, meniu[i], portMAX_DELAY);
    }

    for (;;) {
        if (xSerialGetChar(0, &rxChar, portMAX_DELAY) == pdTRUE) {
            switch (rxChar) {
                case 'm':
                    interogareModLucru();
                    break;
                case 'c':
                    comutareModLucru();
                    break;
                case 't':
                    interogareTemperatura();
                    break;
                case 'h':
                    for (i = 0; i < strlen(meniu); i++) {
                        xSerialPutChar(0, meniu[i], portMAX_DELAY);
                    }
                    break;
                default: {
                    char buf[32];
                    sprintf(buf, "Comanda necunoscuta: %c\r\n", rxChar);
                    for (i = 0; i < strlen(buf); i++) {
                        xSerialPutChar(0, buf[i], portMAX_DELAY);
                    }
                    break;
                }
            }
        }
    }
}
