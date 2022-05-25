/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#include "gfx.h"
#include <stdlib.h>


#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif


uint8_t GfxInit(GfxTypeDef *context, int16_t width, int16_t height)
{
    context->MaxWidth = width;
    context->MaxHeight = height;
    return GFX_OK;
}

uint8_t GfxDrawLine(GfxTypeDef *context, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
        
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) 
    {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }

    if (x0 > x1) 
    {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) 
    {
        ystep = 1;
    } 
    else 
    {
        ystep = -1;
    }

    for (; x0 <= x1; x0++) 
    {
        if (steep) 
        {
            context->DrawPixel(y0, x0, color);
        } 
        else    
        {
            context->DrawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) 
        {
            y0 += ystep;
            err += dx;
        }
    }
return GFX_OK;
}

uint8_t GfxDrawCircle(GfxTypeDef *context, int16_t x0, int16_t y0, int16_t r, uint16_t color) 
{

    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    context->DrawPixel(x0, y0 + r, color);
    context->DrawPixel(x0, y0 - r, color);
    context->DrawPixel(x0 + r, y0, color);
    context->DrawPixel(x0 - r, y0, color);

    while (x < y) 
    {
        if (f >= 0) 
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        context->DrawPixel(x0 + x, y0 + y, color);
        context->DrawPixel(x0 - x, y0 + y, color);
        context->DrawPixel(x0 + x, y0 - y, color);
        context->DrawPixel(x0 - x, y0 - y, color);
        context->DrawPixel(x0 + y, y0 + x, color);
        context->DrawPixel(x0 - y, y0 + x, color);
        context->DrawPixel(x0 + y, y0 - x, color);
        context->DrawPixel(x0 - y, y0 - x, color);
    }
return GFX_OK;
}

uint8_t GfxDrawChar(GfxTypeDef *context, const char ch, const GfxFontTypeDef *font, uint16_t color)
{
    int pointer,pointer1,pointer2;
    uint8_t kar = ch-0x20;
    uint8_t data;
    
    if(context->CursorX > ( context->MaxWidth - 1) - (font->Width) )
    {
        context->CursorX = 0;
        context->CursorY += font->Height;
    }

    for(int16_t xPtr = 0; xPtr < font->Width; xPtr++)
    {
        pointer1 = kar*(font->Width);
        pointer2 = (xPtr * 1);       
        pointer = pointer1 + pointer2;        
        data = (font->FontTable)[pointer];
        uint8_t mask = 0x01;

        for(int16_t yPtr = 0; yPtr < font->Height; yPtr++) 
        {
            if(data & mask)
                context->DrawPixel(context->CursorX + xPtr , context->CursorY + yPtr, color);    
            mask<<=1;
        }
    }

    return GFX_OK;
}


uint8_t GfxDrawString(GfxTypeDef *context, const char *string, const GfxFontTypeDef *font, uint16_t color)
{
    while(*string != 0)
    {
        if((*string)=='\n')
        { 
            context->CursorY += font->Height; 
            context->CursorX = 0;
        }
        else
        {
            GfxDrawChar(context,(*string), font, color);
            context->CursorX += font->Width + 1;
        }
        string++;
    }
    return GFX_OK;
} 