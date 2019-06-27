#ifndef ST7735R_H
#define ST7735R_H

void st7735rSendFB(const uint16_t *fb, int xs, int ys, int w, int h);
void st7735rInit();
void st7735rSetBrightness(int value);
void st7735rPowerDown();

#endif