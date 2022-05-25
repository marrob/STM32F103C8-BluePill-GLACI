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
/* Includes ------------------------------------------------------------------*/
#include "SSD1306_128x32_I2C.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "gfx.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct _SSD1306TypeDef {
    GfxTypeDef Gfx;
    uint8_t FrameBuffer[SSD1306_WIDTH * (SSD1306_HEIGHT / 8) + 1]; // + 1 byte for 0x40 Co = 0, D/C = 0
    uint8_t BPP;
    SSD1306_LowLevelWriteFnPtr pWrite;
}SSD1306TypeDef;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static SSD1306TypeDef _disp;

/* Private function prototypes -----------------------------------------------*/
static uint8_t WriteCommand(uint8_t cmd);
static uint8_t WriteCommands(uint8_t *commands, size_t count);


/* Private user code ---------------------------------------------------------*/
uint8_t SSD1306_Init(SSD1306_LowLevelWriteFnPtr pWrite)
{
    _disp.Gfx.DrawPixel = &SSD1306_DrawPixel;
    GfxInit(&_disp.Gfx, SSD1306_WIDTH, SSD1306_HEIGHT);
    _disp.BPP = SSD1306_BPP;
    _disp.pWrite = pWrite;
    _disp.FrameBuffer[0x00] = 0x40; // Co = 0, D/C = 0
    
    WriteCommand(0xAE);//Display Off

    WriteCommand(0xD5);//SET DISPLAY CLOCK
    WriteCommand(0x80);//105Hz


    WriteCommand(0xA8);//Select Multiplex Ratio
    WriteCommand(0x1F);//Default => 0x3F (1/64 Duty) 0x1F(1/32 Duty)


    WriteCommand(0xD3);//Setting Display Offset
    WriteCommand(0x00);//00H Reset

    WriteCommand(0x40);//Set Display Start Line

    WriteCommand(0x8D);//Set Charge Pump
    WriteCommand(0x14);//Enable Charge Pump

    WriteCommand(0x20); //Set Memory Addressing Mode
    WriteCommand(0xA0); //Horizontal addressing mode
    
    WriteCommand(0x21); //Set Column Address
    WriteCommand(0x00); //Start: 0
    WriteCommand(0x7F); //End: 127
    

    WriteCommand(0x22); //Set Page Address
    WriteCommand(0x00); //Start: 0
    WriteCommand(0x04); //End: 4
  
    WriteCommand(0xA1);//Set Segment Re-Map Default

    WriteCommand(0xC8);//Set COM Output Scan Direction

    WriteCommand(0xDA);//Set COM Hardware Configuration
    WriteCommand(0x02);//Alternative COM Pin---See IC Spec page 34

    WriteCommand(0x81);//Set Contrast Control
    WriteCommand(0x8F);

    WriteCommand(0xD9);//Set Pre-Charge period
    WriteCommand(0x22);

    WriteCommand(0xDB);//Set Pre-Charge period
    WriteCommand(0x40);

    WriteCommand(0xA4);//Entire Display ON

    WriteCommand(0xA6);//Set Normal Display

    WriteCommand(0xAF);//Display ON

    return SSD1306_OK;
}

uint8_t SSD1306_DisplayOff(void){

    WriteCommand(0xAE);//Display Off
    
    WriteCommand(0x8D);//Set Charge Pump
    WriteCommand(0x10);//Disable Charge Pump
    
    return SSD1306_OK;
}

/*
 * x:0...127
 * y:0...31 
 */
uint8_t SSD1306_DrawPixel(int16_t x, int16_t y, uint16_t color)
{
    //printf("pos: %u, %u\r\n", x, y);
    
    if(x > SSD1306_WIDTH - 1)
        return SSD1306_INVALID_ARG;
    if(y > SSD1306_HEIGHT - 1 )
        return SSD1306_INVALID_ARG;
    
    if (_disp.BPP == 1)
    {
        switch (color) 
        {
            case SSD1306_WHITE:
            {
                //+1 byte for D/C byte
                _disp.FrameBuffer[x + (y / 8) * SSD1306_WIDTH + 1] |= (1 << (y & 7));            
                break;
            }
            case SSD1306_BLACK:
            {
                _disp.FrameBuffer[x + (y / 8) * SSD1306_WIDTH + 1] &= ~(1 << (y & 7));
                break;
            }
        }
    }
    return SSD1306_OK;
}

uint8_t WriteCommand(uint8_t cmd)
{
    uint8_t status = 0;
    uint8_t buf[2] = {0x00, cmd }; // Co = 0, D/C = 0
    status = _disp.pWrite(buf, sizeof(buf));
    return status;
}

static uint8_t WriteCommands(uint8_t *commands, size_t count)
{
    uint8_t *buffer = (uint8_t*)malloc(count + 1);
    if(!buffer)
        return SSD1306_NO_FREE_MEM;
    buffer[0] = 0x00; /* D/C byte -> Command */
    memcpy(buffer + 1, commands, count);
    _disp.pWrite(buffer, count + 1);
    free(buffer);
    return SSD1306_OK;
}

uint8_t SSD1306_DisplayUpdate(void)
{
    uint8_t cmds[]=
    {
        0x21,  //Set Column Address
        0x00,  //Start: 0
        0x7F,  //End:127
        0x22,  //Set Page Address
        0x00,  //Start: 0
        0x04,  //End: 4
    };
    WriteCommands(cmds, sizeof(cmds));
    uint16_t pixels = SSD1306_WIDTH * (SSD1306_HEIGHT / 8);
    _disp.pWrite(_disp.FrameBuffer, pixels + 1); 
    return SSD1306_OK;
}

uint8_t SSD1306_DisplayClear() 
{
    if(!_disp.FrameBuffer)
        return SSD1306_ERROR;
    memset(_disp.FrameBuffer + 1, 0, SSD1306_WIDTH * (SSD1306_HEIGHT / 8));
    return SSD1306_OK;
}

uint8_t SSD1306_SetCursor(int16_t x, int16_t y)
{
    _disp.Gfx.CursorX = x;
    _disp.Gfx.CursorY = y;
    return SSD1306_OK;
}

uint8_t SSD1306_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
   return GfxDrawLine(&_disp.Gfx, x0, y0, x1, y1, color);
}

 
uint8_t SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{   
    GfxDrawCircle(&_disp.Gfx, x0, y0, r, color);
    return 0;
}

uint8_t SSD1306_DrawChar(const char ch, const GfxFontTypeDef *font, uint16_t color)
{
    return GfxDrawChar(&_disp.Gfx, ch, font, color);
}

uint8_t SSD1306_DrawString(const char *string, const GfxFontTypeDef *font, uint16_t color)
{
    return GfxDrawString(&_disp.Gfx, string, font, color);
}
 


/* *****************************************************************************
 End of File
 */
