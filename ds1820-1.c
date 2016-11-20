#include "stm8s.h"
#include "iostm8s103f3.h" // bits access
//#include "iostm8s003f3.h" // bits access
#include "STM8_GPIO.h"    // samopal lock

#define THERM_PORT GPIOA
#define THERM_PIN  GPIO_PIN_2
 
#define THERM_INPUT_MODE()  THERM_PORT->DDR &= ~THERM_PIN
#define THERM_OUTPUT_MODE() THERM_PORT->DDR |=  THERM_PIN
#define THERM_LOW()         THERM_PORT->ODR &= (u8)(~THERM_PIN)
#define THERM_HIGH()        THERM_PORT->ODR |= (u8)THERM_PIN
#define THERM_READ()        (THERM_PORT->IDR & (vu8)THERM_PIN)

inline void x_delay(unsigned int i) 
{ 
  while (--i) asm("nop");
} 
 
#define delay_1us()   asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
#define delay_15us()  x_delay(42)
#define delay_45us()  x_delay(127)
#define delay_60us()  x_delay(170)
#define delay_480us() x_delay(1360)
 
volatile char iCurrentTemp = 0;
//� ���������� iCurrentTemp ����� ���������� �����������. 
//��� ����� ���� ��� �������������, ��� � �������������. 
//����������� ���� char �������� ��������� ��� ����������� � ��������� �180�

// �������
#define THERM_CMD_CONVERTTEMP 0x44
#define THERM_CMD_RSCRATCHPAD 0xBE
#define THERM_CMD_WSCRATCHPAD 0x4E
#define THERM_CMD_CPYSCRATCHPAD 0x48
#define THERM_CMD_RECEEPROM 0xB8
#define THERM_CMD_RPWRSUPPLY 0xB4
#define THERM_CMD_SEARCHROM 0xF0
#define THERM_CMD_READROM 0x33
#define THERM_CMD_MATCHROM 0x55
#define THERM_CMD_SKIPROM 0xCC
#define THERM_CMD_ALARMSEARCH 0xEC
 
#define THERM_DECIMAL_STEPS_12BIT 625 //.0625
 
// ������ ������. ��-��������� 12 ���.
typedef enum
{
  THERM_MODE_9BIT  = 0x1F,
  THERM_MODE_10BIT = 0x3F,
  THERM_MODE_11BIT = 0x5F,
  THERM_MODE_12BIT = 0x7F
} THERM_MODE;
 
 
/**
  * @brief ����� ������������
  * @par
  * ��������� �����������
  * @retval
  * false - �������������; true - �����
*/
bool therm_reset(void)
{
  unsigned char i = 0xFF;
 
  THERM_OUTPUT_MODE();
  THERM_LOW();
 
  delay_480us();
 
  THERM_INPUT_MODE();
 
  delay_60us();
 
  i = THERM_READ();
 
  delay_480us();
  // 0 �������� ���������� �����, 1 - ������
  return (0 == i) ? true : false;
}
 
/**
  * @brief ������ ����
  * @param[in] bBit ���
  * @retval
  * ������������ �������� �����������
*/
void therm_write_bit(bool bBit)
{
  THERM_OUTPUT_MODE();
  THERM_LOW();
 
  delay_1us();
 
  if (bBit) 
  {
    THERM_INPUT_MODE();
  }
   
  delay_60us();
   
  THERM_INPUT_MODE();
}
 
/**
  * @brief ������ ����
  * @par
  * ��������� �����������
  * @retval
  * �������� ����.
*/
bool therm_read_bit(void)
{
  bool bBit = 0;
 
  THERM_OUTPUT_MODE();
  THERM_LOW(); 
 
  delay_1us();
 
  THERM_INPUT_MODE();
 
  delay_15us();
   
  if (THERM_READ()) 
  {
    bBit = true;
  }
 
  delay_45us();
 
  return bBit;
}
 
/**
  * @brief ������ �����
  * @par
  * ��������� �����������
  * @retval
  * �������� �����.
*/
static unsigned char therm_read_byte(void)
{
  unsigned char i = 8;
  unsigned char n = 0;
  while (i--)
  {    
    // �������� �� ���� ������� ������ � ��������� �������� ����
    n >>= 1;
    n |= (therm_read_bit() << 7);
  }
  return n;
}
 
/**
  * @brief ������ �����
  * @param[in] byte ����
  * @retval
  * ������������ �������� �����������
*/
void therm_write_byte(unsigned char byte)
{
  unsigned char i = 8;
 
  while (i--)
  {
    // �������� ������� ��� � �������� �� 1 ������� ������
    // ��� ������� � ���������� ����
    therm_write_bit(byte & 1);
    byte >>= 1;
  }  
}
 
/**
  * @brief ���������� ����� ������ ������������
  * @param[in] mode ����� ������
  * @retval
  * ������������ �������� �����������
*/
void therm_init_mode(THERM_MODE mode)
{
  therm_reset();
  therm_write_byte(THERM_CMD_SKIPROM);
  therm_write_byte(THERM_CMD_WSCRATCHPAD);
  therm_write_byte(0);
  therm_write_byte(0);
  therm_write_byte(mode);
}

/**
  * @brief ������ �����������.
  * @par
  * ��������� �����������
  * @retval
  * �����������.
*/
char GetTemperature(void)
{
  char iResult = 0;
  unsigned char temperature[2] = {0, 0};
  unsigned char digit;
  short iReadLimit;
 
  // ������ ���������� �� ����� ������ �������
  disableInterrupts();  
   
  // ����� � ����� ������� � �������������� �����������
  iResult = therm_reset();
  therm_write_byte(THERM_CMD_SKIPROM);
  therm_write_byte(THERM_CMD_CONVERTTEMP);
   
  // �������� ���������� ��������������
  iReadLimit = 10;
  while (!therm_read_bit() && (--iReadLimit > 0)) { ; }
   
  // ����� � ������ ���� �����������
  therm_reset();
  therm_write_byte(THERM_CMD_SKIPROM);
  therm_write_byte(THERM_CMD_RSCRATCHPAD);  
  temperature[0] = therm_read_byte();
  temperature[1] = therm_read_byte();
 
  // ��������� ����������
  enableInterrupts();  
   
  digit = 0;
 
  digit  = (  temperature[0]         >> 4 ) & 0x0F;  
  digit |= ( (temperature[1] & 0x0F) << 4 ) & 0xF0;  
 
  // ������������� ����������� - ������������� � ��������� 1
  if (temperature[1] & 0x80)
  {
    iResult = ~digit; 
    iResult++;
  } else
  {
    iResult = digit; 
  }
   
  iCurrentTemp = iResult;
   
  return iResult;
}