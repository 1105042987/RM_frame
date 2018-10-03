#include "visualscope.h"

//校验类型，只能定义一个，需要和上位机设置相同
//#define CHECK_CRC_16
#define CHECK_SUM



#ifdef CHECK_SUM	//VisualScope的求和校验
static u8 RS232_VisualScope_CHKSUM(u8 *Array,u16 Len)
{
    u8 sum=0;
    u8 i=0;
    for(i=0; i<Len; i++)
        sum+=Array[i];
    return sum;
}
#endif


//通过串口发送VisualScope识别的数据
void VisualScope(UART_HandleTypeDef *huart,int16_t CH1,int16_t CH2,int16_t CH3,int16_t CH4)	//通过串口显示四个通道的波形
{

    uint8_t Buffer[10];
    uint16_t Temp=0;

    Buffer[0]=CH1&0xff;
    Buffer[1]=CH1>>8;
    Buffer[2]=CH2&0xff;;
    Buffer[3]=CH2>>8;
    Buffer[4]=CH3&0xff;;
    Buffer[5]=CH3>>8;
    Buffer[6]=CH4&0xff;;
    Buffer[7]=CH4>>8;

#ifdef CHECK_SUM	//VisualScope的求和校验
    Temp = RS232_VisualScope_CHKSUM(Buffer, 8);
    Buffer[8] = Temp&0x00ff;

   
   HAL_UART_Transmit(huart,&Buffer[0],9,0xfff);
     
#endif
}

