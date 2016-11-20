#include "DS1822.h"

unsigned char DS1822_Start_Conversion_by_ROM (GPIO_TypeDef * GPIOx, unsigned char PINx, unsigned char (*Serial_Num)[DS1822_SERIAL_NUM_SIZE])
{
	unsigned char cnt;
	cnt=One_Wire_Reset(GPIOx, PINx);
	if (cnt!=One_Wire_Success) return cnt;
	One_Wire_Write_Byte(One_Wire_Match_ROM,GPIOx, PINx);
	for (cnt=0;cnt!=8;cnt++) One_Wire_Write_Byte((*Serial_Num)[cnt],GPIOx, PINx);
	One_Wire_Write_Byte(DS1822_CONVERT_T_CMD,GPIOx, PINx);
	return One_Wire_Success;
}

unsigned char DS1822_Get_Conversion_Result_by_ROM_CRC (GPIO_TypeDef * GPIOx, unsigned char PINx, 
	unsigned char (*Serial_Num)[DS1822_SERIAL_NUM_SIZE], unsigned int * temp_code)
{
	unsigned char cnt;
	unsigned char inbuff[DS1822_STRATCHPAD_SIZE];
	cnt=One_Wire_Reset(GPIOx, PINx);
	if (cnt!=One_Wire_Success) return cnt;
	One_Wire_Write_Byte(One_Wire_Match_ROM,GPIOx, PINx);
	for (cnt=0;cnt!=8;cnt++) One_Wire_Write_Byte((*Serial_Num)[cnt],GPIOx, PINx);
	One_Wire_Write_Byte(DS1822_READ_STRATCHPAD_CMD,GPIOx, PINx);
	for (cnt=0;cnt!=DS1822_STRATCHPAD_SIZE;cnt++) inbuff[cnt]=One_Wire_Read_Byte(GPIOx, PINx);
        PIN_OUT_PP(GPIOx, PINx); //supply voltage for sensors
        PIN_ON(GPIOx, PINx);
	if (Crc8Dallas(DS1822_STRATCHPAD_SIZE,inbuff)==0) *temp_code = inbuff[0]|(inbuff[1]<<8);
	else	return One_Wire_CRC_Error;
	return One_Wire_Success;
}

unsigned char DS1822_Search_Rom (GPIO_TypeDef * GPIOx, unsigned char PINx, unsigned char * devices_found, unsigned char (* SN_ROM)[One_Wire_Device_Number_MAX][DS1822_SERIAL_NUM_SIZE])
{
	unsigned long path,next,pos;                                                       
 	unsigned char bit,chk;                                
	unsigned char cnt_bit, cnt_byte, cnt_num,tmp;
 	path=0;                                     
 	cnt_num=0;
	do
	{                                         
 		tmp=One_Wire_Reset(GPIOx, PINx);
 			if (tmp!=One_Wire_Success) return tmp;
			One_Wire_Write_Byte(One_Wire_Search_ROM,GPIOx, PINx);
     	next=0;                                 
     	pos=1;                                                             
  		for (cnt_byte=0;cnt_byte!=8;cnt_byte++)
		{
			(*SN_ROM)[cnt_num][cnt_byte]=0;
			for (cnt_bit=0;cnt_bit!=8;cnt_bit++)
  		{                                     
				bit=One_Wire_Read_Bit(GPIOx, PINx);
				chk=One_Wire_Read_Bit(GPIOx, PINx);
        if(!bit && !chk)
				{                   
          if(pos&path) bit=1;            
            else next=(path&(pos-1))|pos;   
          pos<<=1;
        }
 			One_Wire_Write_Bit(bit, GPIOx, PINx);
			if (bit!=0) (*SN_ROM)[cnt_num][cnt_byte]|=(1<<cnt_bit);
     	}
		}
    path=next;
		cnt_num++;
 	}while(path);
	* devices_found = cnt_num;
	return One_Wire_Success;
}

unsigned char DS1822_Set_Resolution (GPIO_TypeDef * GPIOx, unsigned char PINx, 
        unsigned char (*Serial_Num)[DS1822_SERIAL_NUM_SIZE], 
        DS1822_Resolution_TypeDef Resolution)
{
  	unsigned char cnt;
	cnt=One_Wire_Reset(GPIOx, PINx);
	if (cnt!=One_Wire_Success) return cnt;
	One_Wire_Write_Byte(One_Wire_Match_ROM,GPIOx, PINx);
	for (cnt=0;cnt!=8;cnt++) One_Wire_Write_Byte((*Serial_Num)[cnt],GPIOx, PINx);
	One_Wire_Write_Byte(DS1822_WRITE_STRATCHPAD_CMD,GPIOx, PINx);
        One_Wire_Write_Byte(DS1822_TH_REGISTER_BASE_VALUE,GPIOx, PINx);
        One_Wire_Write_Byte(DS1822_TL_REGISTER_BASE_VALUE,GPIOx, PINx);
        One_Wire_Write_Byte((unsigned char)Resolution,GPIOx, PINx);
	return One_Wire_Success;
}

unsigned char DS1822_Start_Conversion_Skip_ROM (GPIO_TypeDef * GPIOx, unsigned char PINx)
{
  unsigned char tmp=One_Wire_Reset(GPIOx, PINx);
  if (tmp!=One_Wire_Success) return tmp;
  One_Wire_Write_Byte(One_Wire_Skip_ROM,GPIOx, PINx);
  One_Wire_Write_Byte(DS1822_CONVERT_T_CMD,GPIOx, PINx);
  return One_Wire_Success;
}

unsigned char DS1822_Get_Conversion_Result_Skip_ROM_CRC (GPIO_TypeDef * GPIOx, unsigned char PINx,  unsigned int * temp_code)
{
	unsigned char cnt;
	unsigned char inbuff[DS1822_STRATCHPAD_SIZE];
	cnt=One_Wire_Reset(GPIOx, PINx);
	if (cnt!=One_Wire_Success) return cnt;
	One_Wire_Write_Byte(One_Wire_Skip_ROM,GPIOx, PINx);
	One_Wire_Write_Byte(DS1822_READ_STRATCHPAD_CMD,GPIOx, PINx);
	for (cnt=0;cnt!=DS1822_STRATCHPAD_SIZE;cnt++) inbuff[cnt]=One_Wire_Read_Byte(GPIOx, PINx);
        PIN_OUT_PP(GPIOx, PINx); //supply voltage for sensors
        PIN_ON(GPIOx, PINx);
	if (Crc8Dallas(DS1822_STRATCHPAD_SIZE,inbuff)==0) *temp_code = inbuff[0]|(inbuff[1]<<8);
          else	return One_Wire_CRC_Error;
	return One_Wire_Success;
}