
/*
 * bits no     7   6   5   4   3   2   1   0
 * dec value  128  64  32  16  8   4   2   1
 */

/********** current variant **********/
/*
 * One digit:                          TABLE:
 *   ***A***                   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F  -  h
 *   *     *         (F) PD2   0  1  1  1  0  0  0  1  0  0  0  0  0  1  0  0  1  0
 *   F     B         (B) PD1   0  0  0  0  0  1  1  0  0  0  0  1  1  0  1  1  1  1
 *   *     *         (A) PD3   0  1  0  0  1  0  0  0  0  0  0  1  0  1  0  0  1  1
 *   ***G***         (G) PC7   1  1  0  0  0  0  0  1  0  0  0  0  1  0  0  0  0  0
 *   *     *         (C) PC4   0  0  1  0  0  0  0  0  0  0  0  0  1  0  1  1  1  0
 *   E     C         (DP)PC3   1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1
 *   *     *   **    (D) PB4   0  1  0  0  1  0  0  1  0  0  1  0  0  0  0  1  1  1
 *   ***D***  *DP*   (E) PB5   0  1  0  1  1  1  0  1  0  1  0  0  0  0  0  0  1  0
 *             **
 */
/*
 * One digit:                          TABLE:
 *   ***A***                   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F  -  h
 *   *     *         (DP)PC3   1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1
 *   F     B         (C) PC4   0  0  1  0  0  0  0  0  0  0  0  0  1  0  1  1  1  0
 *   *     *         (G) PC7   1  1  0  0  0  0  0  1  0  0  0  0  1  0  0  0  0  0
 *   ***G***         (D) PB4   0  1  0  0  1  0  0  1  0  0  1  0  0  0  0  1  1  1
 *   *     *         (E) PB5   0  1  0  1  1  1  0  1  0  1  0  0  0  0  0  0  1  0
 *   E     C         (B) PD1   0  0  0  0  0  1  1  0  0  0  0  1  1  0  1  1  1  1
 *   *     *   **    (F) PD2   0  1  1  1  0  0  0  1  0  0  0  0  0  1  0  0  1  0
 *   ***D***  *DP*   (A) PD3   0  1  0  0  1  0  0  0  0  0  0  1  0  1  0  0  1  1
 *             **
 */


/*
 * Number of digit on indicator with common anode
 * digis 0..2: PA3, PC5, PC6, PA2
 */
#define Digit1 GPIOA,GPIO_PIN_3 // 10 --PA3 - старший самый
#define Digit2 GPIOC,GPIO_PIN_5 // 15 --PC5
#define Digit3 GPIOC,GPIO_PIN_6 // 16 --PC6
#define Digit4 GPIOA,GPIO_PIN_2 // 6  --PA2 - младший

#define Dot    GPIOC,GPIO_PIN_3 // 13 --PC3

#define MaskLen 21 // number of chars - единица светодиод не горит!
const unsigned char MaskB[MaskLen]={0xCF,0xFF,0xCF,0xEF,0xFF,0xEF,0xCF,0xFF,0xCF,0xEF,0xCF,0xFF,0xDF,0x6F,0xDF,0xDF,0xFF,0xDF,0xCF,0xFF,0xFF};
const unsigned char MaskC[MaskLen]={0xEF,0xEF,0x7F,0x6F,0x6F,0x6F,0x6F,0xEF,0x6F,0x6F,0x6F,0x2F,0x3F,0x2F,0x3F,0x7F,0xFF,0xBF,0xAF,0xFF,0xFF};
const unsigned char MaskD[MaskLen]={0xF1,0xFD,0xF5,0xF5,0xF9,0xF3,0xF3,0xF5,0xF1,0xF1,0xF7,0xF7,0xFF,0xF7,0xF7,0xF7,0xF7,0xFF,0xFF,0xFF,0xFF};

const unsigned char BaseValueA = 0;
const unsigned char BaseValueB = (1<<5)|(1<<4); //ed
const unsigned char BaseValueC = (1<<7)|(1<<4); //gc
const unsigned char BaseValueD = (1<<3)|(1<<2)|(1<<1); //afb
const unsigned char BaseValueDot = (1<<3); //dot

u8 DigitLightning=0;                            // какая горит
volatile u8 DrawDigitFlag=0;                    //
u8 DigitLCD[4]={0,0,0,0};                       // буфер 1234


u8 DotPosition=0x1;     // 0-1-2-3 
u8 DotOn=1;             // признак включения точки, он каждые 0.5с устанавливается в 1 
u8 Dot_Clock=1;         // enable 0.5 sec dot blink

void DrawDigit (u8 Dig, u8 DigitPos)
{
u8 DotMask=0xFF; // нет точки по умолчанию
  if ((DotPosition==DigitPos)&(DotOn==1)) DotMask=0x00; //позиция точки? и .5 sec count? зажигаем!
    switch (DigitPos){
    case 0: PIN_OFF_LATCH(Digit4); break;
    case 1: PIN_OFF_LATCH(Digit1); break;
    case 2: PIN_OFF_LATCH(Digit2); break;
    case 3: PIN_OFF_LATCH(Digit3); break;
    default: break; }
//    GPIOA->ODR=BaseValueA&MaskA[Dig]; // No pins there
    GPIOB->ODR=BaseValueB&MaskB[Dig];
    GPIOC->ODR=(BaseValueC&MaskC[Dig])|(BaseValueDot&DotMask);
    GPIOD->ODR=BaseValueD&MaskD[Dig];
    switch (DigitPos){
    case 0: PIN_ON_LATCH(Digit1); break;
    case 1: PIN_ON_LATCH(Digit2); break;
    case 2: PIN_ON_LATCH(Digit3); break;
    case 3: PIN_ON_LATCH(Digit4); break;
    default: break; }
}

void Display_Handle (void)
{
  if(DrawDigitFlag) 
    {
      DrawDigit(DigitLCD[DigitLightning], DigitLightning);
      DrawDigitFlag=0;
    }
}

void LED_Init(void) {
  GPIOA->DDR=BaseValueA|(1<<2)|(1<<3); // M4 M1
  GPIOB->DDR=BaseValueB; //
  GPIOC->DDR=BaseValueC|(1<<5)|(1<<6)|BaseValueDot; // M2 M3 DOT
  GPIOD->DDR=BaseValueD; //
 // PC_DDR_bit.DDR7 = 1; //Настраиваем 7й пин порта C на выход
 // PC_CR1_bit.C17 = 1; //Переключаем его в режим push-pull (это когда он может выдавать 
 //и низкий и высокий уровень), а то по-умолчанию он прикидывается пинов с открытым стоком
 //(это когда может выдавать только низкий уровень, а вместо высокого переключается на вход)
  GPIO_Init(Digit1, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(Digit2, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(Digit3, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(Digit4, GPIO_MODE_OUT_PP_LOW_FAST); 
 
  GPIOA->CR2 = 0; //GPIO_Pin _1; //Прерывание разрешено
  GPIOB->CR2 = 0; //GPIO_Pin _1; //Прерывание разрешено
  GPIOC->CR2 = 0; //GPIO_Pin _1; //Прерывание разрешено
  GPIOD->CR2 = 0; //GPIO_Pin _1; //Прерывание разрешено  
}

static void putnumXXXX(unsigned int n)
{
  unsigned int o; // 270F = 9999
  unsigned int p;
  unsigned int q;
          o=n%10; n/=10;
          p=n%10; n/=10;
          q=n%10; n/=10;
          if (n==0) DigitLCD[0]=20; else DigitLCD[0]=n; // первый 0? если да то пробел
          if ((n==0)&(q==0)) DigitLCD[1]=20; else DigitLCD[1]=q; // и второй 0? если да то пробел
          if ((n==0)&(q==0)&(p==0)) DigitLCD[2]=20; else DigitLCD[2]=p; // и третий 0? если да то пробел
          DigitLCD[3]=o;
          }

static void putnumXX__(unsigned int n)
{
  unsigned int q;
          q=n%10; n/=10;
          if (n==0) DigitLCD[0]=20; else DigitLCD[0]=n;
          if ((n==0)&(q==0)) DigitLCD[1]=20; else DigitLCD[1]=q;
          }

static void putnum__XX(unsigned int n)
{
  unsigned int q;
          q=n%10; n/=10;
          DigitLCD[2]=n;
          DigitLCD[3]=q;
          }



void ShowFloatValueLowerRes (unsigned int Value)
{
  DigitLCD[0]=0;
  DigitLCD[1]=0;
  DigitLCD[2]=0;
  DigitLCD[3]=0;
  if (Value>0x0550) // 270F = 9999
  {
//    Value=0xFFFF-Value;
    
    DigitLCD[0]=8;
    DigitLCD[1]=7;
    DigitLCD[2]=7;
    DigitLCD[3]=8;
    
  } else {
//  DotPosition=2;
  Value=(Value*10)>>4;
  while(Value>=100)
  {
    Value-=100;
    DigitLCD[1]++;
  }
  while(Value>=10)
  {
    Value-=10;
    DigitLCD[2]++;
  }
  DigitLCD[3]=Value;   }
}