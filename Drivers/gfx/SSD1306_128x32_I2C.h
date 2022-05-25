/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _SSD1306_128x32_I2C_H    /* Guard against multiple inclusion */
#define _SSD1306_128x32_I2C_H

#include "gfx.h"


#define SSD1306_WHITE           0
#define SSD1306_BLACK           1

#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          32
#define SSD1306_BPP             1   //Bits per pixel color

#define SSD1306_OK              0
#define SSD1306_ERROR           1
#define SSD1306_NO_FREE_MEM     2
#define SSD1306_GFX_ERROR       3
#define SSD1306_INVALID_ARG     4

typedef uint8_t (*SSD1306_LowLevelWriteFnPtr)(uint8_t* wdata, size_t wlength);

#ifdef __cplusplus
extern "C" {
#endif
    
    uint8_t SSD1306_Init(SSD1306_LowLevelWriteFnPtr pWrite);
    uint8_t SSD1306_DrawPixel(int16_t x, int16_t y, uint16_t color);
    uint8_t SSD1306_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
    uint8_t SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
    uint8_t SSD1306_SetCursor(int16_t x, int16_t y);
    uint8_t SSD1306_DrawChar(const char ch, const GfxFontTypeDef *font, uint16_t color);
    uint8_t SSD1306_DrawString(const char *string, const GfxFontTypeDef *font, uint16_t color);
    uint8_t SSD1306_DisplayUpdate(void);
    uint8_t SSD1306_DisplayClear(void);
    uint8_t SSD1306_DisplayOff(void);

#ifdef __cplusplus
}
#endif

#endif /* _SSD1306_128x32_I2C_H */

/* *****************************************************************************
 End of File
 */
