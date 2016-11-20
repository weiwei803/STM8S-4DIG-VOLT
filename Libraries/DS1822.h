#include "stm8s.h"
#include "1-wire.h"
#include "CRC.h"

#define DS1822_CONVERT_T_CMD				0x44
#define DS1822_WRITE_STRATCHPAD_CMD			0x4E
#define DS1822_READ_STRATCHPAD_CMD			0xBE
#define DS1822_COPY_STRATCHPAD_CMD			0x48
#define DS1822_RECALL_E_CMD				0xB8
#define DS1822_READ_POWER_SUPPLY_CMD		        0xB4

#define DS1822_STRATCHPAD_SIZE				0x09
#define DS1822_SERIAL_NUM_SIZE				0x08

#define DS1822_TH_REGISTER_BASE_VALUE                   0xFF
#define DS1822_TL_REGISTER_BASE_VALUE                   0xFF

#define One_Wire_Device_Number_MAX			4	//maximum number of 1-wire devices on bus

typedef enum
{
  DS1822_9_BIT_RESOLUTION  = (unsigned char) 0x1F,
  DS1822_10_BIT_RESOLUTION = (unsigned char) 0x3F,
  DS1822_11_BIT_RESOLUTION = (unsigned char) 0x5F,
  DS1822_12_BIT_RESOLUTION = (unsigned char) 0x7F
}DS1822_Resolution_TypeDef;

unsigned char DS1822_Start_Conversion_by_ROM (GPIO_TypeDef * GPIOx, unsigned char PINx, unsigned char (*Serial_Num)[DS1822_SERIAL_NUM_SIZE]);
unsigned char DS1822_Get_Conversion_Result_by_ROM_CRC (GPIO_TypeDef * GPIOx, unsigned char PINx, unsigned char (*Serial_Num)[DS1822_SERIAL_NUM_SIZE], unsigned int * temp_code);
unsigned char DS1822_Search_Rom (GPIO_TypeDef * GPIOx, unsigned char PINx, unsigned char * devices_found, unsigned char (* SN_ROM)[One_Wire_Device_Number_MAX][DS1822_SERIAL_NUM_SIZE]);
unsigned char DS1822_Set_Resolution (GPIO_TypeDef * GPIOx, unsigned char PINx, unsigned char (*Serial_Num)[DS1822_SERIAL_NUM_SIZE], DS1822_Resolution_TypeDef Resolution);
unsigned char DS1822_Start_Conversion_Skip_ROM (GPIO_TypeDef * GPIOx, unsigned char PINx);
unsigned char DS1822_Get_Conversion_Result_Skip_ROM_CRC (GPIO_TypeDef * GPIOx, unsigned char PINx,  unsigned int * temp_code);

