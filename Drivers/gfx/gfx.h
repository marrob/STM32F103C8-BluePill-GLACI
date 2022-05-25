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

#ifndef _GFX_H
#define _GFX_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GFX_OK  0

    typedef struct _GfxTypeDef
    {
        int16_t MaxWidth;
        int16_t MaxHeight;
        int16_t _width;       ///< Display width as modified by current rotation
        int16_t _height;      ///< Display height as modified by current rotation
        int16_t CursorX;      ///< x location to start print()ing text
        int16_t CursorY;      ///< y location to start print()ing text
        uint16_t textcolor;   ///< 16-bit background color for print()
        uint16_t textbgcolor; ///< 16-bit text color for print()
        uint8_t textsize_x;   ///< Desired magnification in X-axis of text to print()
        uint8_t textsize_y;   ///< Desired magnification in Y-axis of text to print()
        uint8_t rotation;     ///< Display rotation (0 thru 3)
        uint8_t wrap;         ///< If set, 'wrap' text at right edge of display
        uint8_t _cp437;       ///< If set, use correct CP437 charset (default is off)
        //GFXfont *gfxFont;   ///< Pointer to special font
        
        uint8_t (*DrawPixel)(int16_t x, int16_t, uint16_t color);
    }GfxTypeDef;
    
    typedef struct _GfxFontTypeDef
    {
        uint8_t Width;              /* Character width for storage         */
        uint8_t Height;             /* Character height for storage        */
        uint8_t SpaceWidth;
        const uint8_t *FontTable;       /* Font table start address in memory  */
    } GfxFontTypeDef;
    
    typedef struct _GfxBitmapTypeDef
    {
        struct _Size
        {
            uint16_t X;
            uint16_t Y;
        }Size;
        uint8_t Data[];
    }GfxBitmapTypeDef;

    extern GfxFontTypeDef GfxFont3x6;
    extern GfxFontTypeDef GfxFont5x8;
    extern GfxFontTypeDef GfxFont7x8;
    extern GfxFontTypeDef GfxFont8x12;
    
    uint8_t GfxInit(GfxTypeDef *context,  int16_t w, int16_t h);
    uint8_t GfxDrawLine(GfxTypeDef *context, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color); 
    uint8_t GfxDrawCircle(GfxTypeDef *context, int16_t x0, int16_t y0, int16_t r, uint16_t color);
    uint8_t GfxDrawChar(GfxTypeDef *context, const char ch, const GfxFontTypeDef *font, uint16_t color);
    uint8_t GfxDrawString(GfxTypeDef *context, const char *string, const GfxFontTypeDef *font, uint16_t color);

#ifdef __cplusplus
}
#endif

#endif /* _GFX_H */

/* *****************************************************************************
 End of File
 */
