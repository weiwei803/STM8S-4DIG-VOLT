
#include "stm8s.h"
//#include "iostm8s103f3.h" // bits access

//#include "iostm8s003f3.h" // bits access
//#include "STM8_GPIO.h"    // samopal lock
 
#include "time.h"
#include "1-wire.h"
#include "DS1822.h"
#include "ds1820-1.c"
#include "bin_.h"    // работа с двоичными числами
#include "LED_4.c"  // четырёх знаковый экран


#define OneWirePin GPIOA, GPIO_PIN_1
//#define OneWirePin GPIOD, GPIO_PIN_4
#define DS1822_Resolution DS1822_10_BIT_RESOLUTION
unsigned char DS1822_SN[One_Wire_Device_Number_MAX][DS1822_SERIAL_NUM_SIZE];




#define AUTORELOAD	0x8000			        // sampling period definition (trigg period)
#define NUMB_SAMP	16	        		// number of samples for SW filtering
#define PEAK_FILTER	8				//  floating spot fall speed filter
#define AD_STAB		20				// AD stabilization [TIM1 increments] (~10us)
#define ALL_LEDs ((u8)0x0F)				// LEDs mask 
/* Private macro -------------------------------------------------------------*/
#define switch_all_LEDs_on	{ GPIOB->ODR|= ALL_LEDs; }      //LEDs control : all on
#define switch_all_LEDs_off	{ GPIOB->ODR&=~ALL_LEDs; }      //LEDs control : all off
/* Private variables ---------------------------------------------------------*/






u8 NumberOfSensorsFound;
volatile u8 TimerSecondFlag=0;
u16 TimerCounter=0;
u8 TimerRedrawCounter=0;
u16 Timer4Counter=0;
u16 Timer4Counter1=0;
u8 Timer4Counter2=0;
u8 DataConvertedFlag=0;
u8 ShowResultSensorNumber=0;
u8 ShowingSensorNumberFlag=1;
u16 TimerCounterSecondValue=2; //default value

struct tm t;

//Задержка на цикле. Выбрана на глаз и равна примерно половине секунды.
void SomeDelay()
{
  for (unsigned long delay_count=0; delay_count<3000; delay_count++); // 300000
};  

/* comments

Если используется стандартная библиотека, то OPT уже занят:
STM8S_StdPeriph_Driver\inc\stm8s.h 2514 #define OPT ((OPT_TypeDef *) OPT_BaseAddress) 

У меня была похожая проблема при подключении стандартной библиотеки.
Решил довольно просто:

1. Идем в меню Projects -> Options -> C/C++ Compiler. 
Во вкладке Preprocessor указываем путь к директориям где сидят библиотечные файлы. 
Что-то типа этого:
$PROJ_DIR$\Libraries\STM8S_StdPeriph_Driver\inc
$PROJ_DIR$\Libraries\STM8S_StdPeriph_Driver\src
2. Идем в меню Projects -> Add files, и добавляем файлы библиотек.
3. Снова в меню Projects -> Add group, и создаём папку для заголовков inc. 
В эту папку добавляем заголовочные файлы этих самых библиотек (как в п.2).

На другом форуме подсказали: 
Нужно #define USE_UART2 добавить в глобальные дефайны проекта. После этого все заработало.

clk_init и port_init вы сами напишете.
TIM1 работает в полностью автоматическом режиме с автозагрузкой без прерываний - дрыгает ногой TIM1_CH1,
TIM2 - в полностью ручном одноимпульсном режиме с прерыванием (запустили, прервался и тут же заглох).

чтобы разрешить TIM1 дрыгать ногами: TIM1_BKR|=TIM1_BKR_MOE; //enable pwmoutputs
чтобы запретить: TIM1_BKR&=(uint8_t)(~TIM1_BKR_MOE); //disable pwmoutputs
чтобы переключить: TIM1_BKR^=TIM1_BKR_MOE; //toggle pwmoutputs

http://radiokot.ru/forum/viewtopic.php?f=20&t=35768&start=180
заведомо рабочий код:

void tim1_init()
{
  CLK_PCKENR1|=CLK_PCKENR1_TIM1;
  TIM1_SR1&=(uint8_t)(~TIM1_SR1_UIF);  //Сбросим признак прерывания
  TIM1_PSCRH=0; // Prescaler 1MHz @ 16MHz /2 (cauze of half-period)
  TIM1_PSCRL=8;
  TIM1_ARRH=0x00;
  TIM1_ARRL=0x01; // Auto-reload value
  TIM1_CCMR1|=0x30; //toggle mode
  TIM1_CCER1|=TIM1_CCER1_CC1E;
  TIM1_CR1|=TIM1_CR1_CEN; // Enable timer
}

void tim2_init()
{
  CLK_PCKENR1|=CLK_PCKENR1_TIM2;
  TIM2_SR1&=~TIM2_SR1_UIF;  //Сбросим признак прерывания
  TIM2_PSCR=0x09; // Prescaler=512 31.25kHz @ 16MHz (1/31.25k=32us per step)
  TIM2_CNTRH=(uint8_t)(tmp_delay >> 8);
  TIM2_CNTRL=(uint8_t)(tmp_delay);
  TIM2_IER|=TIM2_IER_UIE; // Enable interrupt
  TIM2_CR1|=TIM2_CR1_CEN | TIM2_CR1_OPM | TIM2_CR1_URS; // Enable timer
}
*/

	u8	ADInit;					// flag for ADC initialized
	u8	ADSampRdy;				// flag for filed of samples ready
	u8  AD_samp;					// counter of stored samples
	u16 AD_sample[NUMB_SAMP];	                // store samples field 
	u16 AD_avg_value;				// average of ADC result


//extern u8	ADInit;						// flag for the first conversion
//extern u8	ADSampRdy;					// flag for filed of samples ready
//extern u8 AD_samp;						// counter of raugh samples
//extern u16 AD_sample[NUMB_SAMP];	// store field of raugh samples
//extern u16 AD_avg_value;				// filtered ADC result


void ADC1_init(void) {
// 							*** TIM1 INITIALIZATION ***
        TIM1->ARRL= (u8)(AUTORELOAD);
        TIM1->ARRH= (u8)(AUTORELOAD >> 8);		// set autoreload register for trigger period
	TIM1->CCR1H= (u8)((AUTORELOAD-AD_STAB) >> 8);   // set compare register for trigger period
	TIM1->CCR1L= (u8)(AUTORELOAD-AD_STAB);
	TIM1->CR1 |= MASK_TIM1_CR1_ARPE;			// auto reload register is buferred
	TIM1->CR2= (4<<4) & TIM1_CR2_MMS;		// CC1REF is used as TRGO
	TIM1->CCMR1= (6<<4) & TIM1_CCMR_OCM;	        // CC1REF in PWM 1 mode
	TIM1->IER|= MASK_TIM1_IER_CC1IE;			// CC1 interrupt enable
	TIM1->CCER1 |= MASK_TIM1_CCER1_CC1P;			// CC1 negative polarity
	TIM1->CCER1 |= MASK_TIM1_CCER1_CC1E;			// CC1 output enable
	TIM1->BKR |= MASK_TIM1_BKR_MOE;												
	TIM1->SMCR |=  MASK_TIM1_SMCR_MSM;			// synchronization of TRGO with ADC
	TIM1->CR1 |= MASK_TIM1_CR1_CEN;			// timer 1 enable
       
// init ADC variables
	AD_samp= 0;					// number of stored samples 0
	ADInit= TRUE;                                   // ADC initialized 
	ADSampRdy= FALSE;                               // No sample 	

  	GPIOD->CR1|= B8(00100000);				// PD.5 (AIN5) as a floating input
	GPIOD->DDR|= 0x20;	

        // *** ADC INITIALIZATION ***		
	ADC1->TDRH|= 0;					// disable Schmitt trigger on AD input 9 (0x20)
	ADC1->TDRL|= B8(00100000);                       // disable Schmitt trigger on AD input 5 (AIN5)
	ADC1->CSR= ADC1_CSR_EOCIE | (5 & ADC1_CSR_CH);  // ADC EOC interrupt enable, channel 5
	ADC1->CR1= ADC1_PRESSEL_FCPU_D8;		// master clock/8, single conversion
	ADC1->CR2= ADC1_CR2_EXTTRIG | ADC1_ALIGN_RIGHT; // external trigger on timer 1 TRGO & alignment
	ADC1->CR1|= ADC1_CR1_ADON;			// ADC on -- Пинаем АЦП, чтобы он проснулся
        ADC1->CR1|= ADC1_CR1_ADON;			// ADC on -- Пинаем АЦП, чтобы он проснулся
}


void Init(void)
{
  CLK->CKDIVR=0;  //16Mhz OSC RC
//  GPIOB->DDR=GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_1 | GPIO_Pin_0;

  /*     
  TIM2_TimeBaseInit(TIM2_PRESCALER_8192, 10);
  TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
  TIM2_Cmd(ENABLE);
  
                             //Настройка Таймера1
  TIM1_CR2 = 0;                 // Синхронизация как ведущий с периферией отключена
  TIM1_SMCR = 0;                // Синхронизация как ведомый с периферией отключена
  TIM1_ETR = 0;                 // Внешнее тактирование отключено
  TIM1_IER = MASK_TIM1_IER_UIE; // Прерывание по обновлению счетного регистра разрешено
  TIM1_PSCRH = 0;               // Предделитель - 0 ВАЖНО!!!
  TIM1_PSCRL = 0;               // Порядок установки предделителя - старший регистр, потом младший
  TIM1_CR1 = (MASK_TIM1_CR1_URS+MASK_TIM1_CR1_CEN);// Режим непрерывного счета по возрастанию
 */                              // Прерывание по переполнению разрешено и таймер запущен
    
  
//  TIM4 на 1 миллисекунду (@16MHz) с прерываниями и авторелоадом:
    TIM4->SR1&=~TIM4_SR1_UIF;  //Сбросим признак прерывания
    TIM4->PSCR = 0x07; //TIM4_Prescaler_128; // Prescaler
    TIM4->ARR = 124; // Auto-reload value why not 125? can it be 250 for 0.5 ms? or 0x08 & 124?
    TIM4_IER |= MASK_TIM4_IER_UIE; // Enable interrupt
    TIM4_CR1 |= MASK_TIM4_CR1_CEN; // Enable timer
	

 
}

void DisplaySensorNumber (u8 SenseNum)
{
  DigitLCD[0]=5;
  DigitLCD[1]=20; //was19
  if (SenseNum<10) DigitLCD[2]=20;
    else
    {
      SenseNum=SenseNum-10;
      DigitLCD[2]=1;
    }
  DigitLCD[3]=SenseNum;
//  DotPosition=1;
}

void DisplayErrorNumber (u8 ErrorNum)
{
  DigitLCD[0]=3;
  DigitLCD[1]=3;
  DigitLCD[2]=3;
  DigitLCD[3]=ErrorNum;
//  DotPosition=1;
}

u8 ErrorHandle (u8 ErrorNum)
{
 if (ErrorNum==One_Wire_Success) return 0; 
 else DisplayErrorNumber (ErrorNum);
 return 1;
}

u8 SearchSensors (void)
{
  u8 tmp;
  disableInterrupts();
  tmp=ErrorHandle (DS1822_Search_Rom(OneWirePin, &NumberOfSensorsFound, &DS1822_SN));
  enableInterrupts();
  return tmp;
}

u8 StartConversionSingleSesnor (void)
{
  u8 err=0;
  u8 tmp;
  if (DataConvertedFlag) return 0;
  disableInterrupts();
  tmp=ErrorHandle (DS1822_Start_Conversion_by_ROM(OneWirePin, &DS1822_SN[ShowResultSensorNumber]));
  PIN_OUT_PP(OneWirePin);
  PIN_ON(OneWirePin);
  DataConvertedFlag=1;
  enableInterrupts();
  if (tmp) err=tmp;
  return err;
}

u8 TempMeagureTask (void)
{
  u8 tmp=0;
  if (TimerSecondFlag)
  {
    tmp=StartConversionSingleSesnor();
    TimerSecondFlag=0;
  }
  return tmp;
}

u8 GetResultTask (void)
{
  u8 tmp=0;
  unsigned int result;
  if (DataConvertedFlag==0) return 0;
  if (TimerSecondFlag==0) return 0;
/*  
  if (ShowingSensorNumberFlag)
  {
    if (NumberOfSensorsFound!=1) DisplaySensorNumber(ShowResultSensorNumber+1);
    ShowingSensorNumberFlag=0;
    return 0;
  }
  else ShowingSensorNumberFlag=1;
*/  
  disableInterrupts();

  tmp=ErrorHandle (DS1822_Get_Conversion_Result_by_ROM_CRC(OneWirePin, 
                                &DS1822_SN[ShowResultSensorNumber], &result));
  enableInterrupts();
  if (tmp)
  {
    DisplayErrorNumber(tmp);
    return tmp;
  }
  ShowFloatValueLowerRes(result); 
  ShowResultSensorNumber++;
  DigitLCD[0]=ShowResultSensorNumber;
  if (ShowResultSensorNumber>=NumberOfSensorsFound) ShowResultSensorNumber=0; 
  DataConvertedFlag=0;
  TimerSecondFlag=0;
  return tmp;
}

u8 ResolutionSet (void)
{
  u8 cnt;
  u8 err=0;
  disableInterrupts();
  for (cnt=0; cnt!=NumberOfSensorsFound; cnt++)
  {
    err=DS1822_Set_Resolution(OneWirePin, &DS1822_SN[cnt], DS1822_Resolution);
    if (err) 
    {
      DisplayErrorNumber(err);
      enableInterrupts();
      return err;
    }
  }
  enableInterrupts();
  return 0;
}

u16 u16_average(u16 *p, u8 smp) {  // вычисление среднего значения
	u8 i;
	u16 sum;
	for(i=0, sum= 0; i < smp; ++i)
		sum+= *p++;		
	return sum / smp;
}
/*Программа ищет подключенные датчики и далее работает с ними.
Максимальное количество датчиков - 8 штук.
Режим паразитного питания пока не испытан окончательно.
Коды ошибок:
1 - нет сигнала RESPONSE. Ни одно устройство 1-Wire не обнаружено на шине.
2 - низкое состояние линии. Короткое замыкание на землю. Нет pull-up резистора.
3 - утройство занято. По каким-то причинам время преобразования затянулось. Ошибка во время тестов ниразу не встретилась.
4 - ошибка CRC. Ошибка передачи данных. Помехи на линии, плохой контакт.
Любой обмен на шине идет с проверкой CRC.
*/

int main( void )
{
  // u16 n=0; 
  u8 err=0;
                                                       // Set initial time - there is no UI for this
    t.tm_hour = 23;                                    // Hour
    t.tm_min = 58;                                     // Minute
    t.tm_sec = 18;                                     // Second
    t.tm_mon = 11;                                      // Month (0 based!)
    t.tm_mday = 19;                                    // Day of Month
    t.tm_year = 2016 - 1900;                           // Year
    t.tm_wday = 6;                                     // Day of Week - Not used by mktime()
    t.tm_yday = 0;                                     // Not used by mktime()
    t.tm_isdst = 0;                                    // DST flag - Not used by rtc_tick()
  
    Init();
    LED_Init();
    ADC1_init();
    GPIO_Init(OneWirePin, GPIO_MODE_OUT_PP_HIGH_FAST);
    enableInterrupts();				// enable all interrupts
//   err=SearchSensors();
//   err=ResolutionSet();
    asm("rim"); //Разрешаем прерывания
    asm("nop");  //Пустая операция
    DigitLCD[0]=NumberOfSensorsFound;
    Display_Handle();
    SomeDelay();
  while (1) //В цикле будем работать
  {  Display_Handle();
    if (ADSampRdy == TRUE) {				   // field of ADC samples is ready?
		AD_avg_value= u16_average(&AD_sample[0], AD_samp); // average of samples
		AD_samp= 0;					   // reinitalize ADC variables
		ADSampRdy= FALSE;
		};   
    if (Timer4Counter1==1) { // раз в секунду делаем это
      Timer4Counter1=0;
      if (AD_avg_value < 92) {putnumXXXX(AD_avg_value*109); DotPosition=0;} // DotPosition=  0-1-2-3 
      else if (AD_avg_value < 910) {putnumXXXX(AD_avg_value*11); DotPosition=1;} // clock : position
      else {putnumXXXX(9999); DotPosition=2;}

  };
};
}
  //  PC_ODR_bit.ODR7 = 1; //Переключаем пин в высокий уровень - светодиод горит
 //  SomeDelay();  //Задержка в 0.5 сек
 //  PC_ODR_bit.ODR7 = 0; //Пин в низкий уровень - светик тухнет
//   if (Timer4Counter2==1) { 
//     Timer4Counter2=0;
//     n++; if (n==100) n=0;
//    putnumXXXX(n);  
   
//   if (err==0) err=TempMeagureTask();
//   if (err==0) err=GetResultTask();
//   err=0;
//     if ((t.tm_sec>=10)&(t.tm_sec<=30)) {
//       putnum__XX(t.tm_sec);
//   //    putnumXX__(t.tm_min); Clock_Dot=2; }
//   //      putnumXX__(59-t.tm_sec); Clock_Dot=2; }
//       DigitLCD[0]=(t.tm_sec-10);
//       DigitLCD[1]=(t.tm_sec-10); }
//     else { putnum__XX(t.tm_min); putnumXX__(t.tm_hour); Clock_Dot=1; }
//        } 

//   if (AD_avg_value > 99) putnumXXXX(AD_avg_value); else {
//     DigitLCD[2]=20; DigitLCD[3]=20; putnumXX__(t.tm_sec);
//   };
   
   
   //   putnumXX__(t.tm_sec);




//void TIM2Handler(void) 
#pragma vector = TIM2_OVR_UIF_vector
__interrupt void TIM2_OVR_UIF_handler(void)
{
  if (TIM2_SR1_UIF==1) {
//  DigitLightning++;
//  if (DigitLightning>3) DigitLightning=0;
//  DrawDigitFlag=1;
//  TimerCounter++;
//  if (TimerCounter>=TimerCounterSecondValue)
//  {
//    TimerSecondFlag=1;
//    TimerCounter=0;
  asm("nop"); }
  TIM2_SR1_UIF = 0; // if
}

static const signed char dayinmonth[12][2] = {          // Number of days in month for non-leap year and leap year
        31,     31,                                     // January
        28,     29,                                     // February
        31,     31,                                     // March
        30,     30,                                     // April
        31,     31,                                     // May
        30,     30,                                     // June
        31,     31,                                     // July
        31,     31,                                     // August
        30,     30,                                     // September
        31,     31,                                     // October
        30,     30,                                     // November
        31,     31                                      // December
    };



static int is_leap_year(const int y)
{
    if(y & 3) return 0;         // Not divisible by 4
    switch(y % 400) {           // Divisible by 100, but not by 400 (1900, 2100, 2200, 2300, 2500, 2600)
        case 100: case 200: case 300: return 0; 
    }
    return 1;                   // Divisible by 4 and !(100 and !400)
}

//void TIM4Handler(void) TIM4 на 1 миллисекунду (@16MHz) с прерываниями и авторелоадом:
#pragma vector = TIM4_OVR_UIF_vector
__interrupt void TIM4_OVR_UIF_handler(void)
{
  if (TIM4_SR1_UIF==1) 
  { 
    TimerCounter++; if (TimerCounter>=1200)  { TimerSecondFlag=1; TimerCounter=0; }  // зачем
    TimerRedrawCounter++; if (TimerRedrawCounter>=5) {
      DigitLightning++; if (DigitLightning>3) DigitLightning=0;
    DrawDigitFlag=1; TimerRedrawCounter=0; }
  Timer4Counter++; 
  if (Timer4Counter==501) DotOn=1;       // .5 sec count? если Clock_Dot то точку обозначаем
  if (Timer4Counter>=1001) { 
    Timer4Counter=0; 
    Timer4Counter1=1;                // флаг новой секунды для main
    if (Dot_Clock>=1) DotOn=0;    //если Clock_Dot расзрешен то точку отключаем
  
      if(++t.tm_sec > 59) {                               // Increment seconds, check for overflow
        t.tm_sec = 0;                                   // Reset seconds
        if(++t.tm_min > 59) {                           // Increment minutes, check for overflow
            t.tm_min = 0;                               // Reset minutes
            if(++t.tm_hour > 23) {                      // Increment hours, check for overflow
                t.tm_hour = 0;                          // Reset hours
                ++t.tm_yday;                            // Increment day of year
                if(++t.tm_wday > 6)                     // Increment day of week, check for overflow
                    t.tm_wday = 0;                      // Reset day of week
                                                        // Increment day of month, check for overflow
                if(++t.tm_mday > dayinmonth[t.tm_mon][is_leap_year(t.tm_year + 1900)]) {
                    t.tm_mday = 1;                      // Reset day of month
                    if(++t.tm_mon > 11) {               // Increment month, check for overflow
                        t.tm_mon = 0;                   // Reset month
                        t.tm_yday = 0;                  // Reset day of year
                        ++t.tm_year;                    // Increment year
                    }                                   // - year       
                }                                       // - month
            }                                           // - day
        }                                               // - hour
    }                                                   // - minute
                                                        //
  
  
  
  } // one sec dot off 
  } // if
  TIM4_SR_UIF = 0;
}

#pragma vector = ADC1_EOC_vector
__interrupt void ADC1_EOC_handler(void)
{	
	if(ADInit == FALSE  &&  ADSampRdy == FALSE) {
        AD_sample[AD_samp]= ADC1->DRL;          // in right-alignment mode we should first read LSB
	AD_sample[AD_samp]|= ADC1->DRH << 8;
		if(++AD_samp >= NUMB_SAMP)	// AD_smaple field is full?
			ADSampRdy= TRUE;	// YES - set field ready flag for main loop
	}
	else 
		ADInit= FALSE;			// NO - ignore sample, wait for next one
	ADC1->CSR&=~ADC1_CSR_EOC;		// clear end of conversion flag
	ADC1->CR1&=~ADC1_CR1_ADON;	        // stop ADC
  return;
}


// Вектор прерывания по обновлению или переполнению Таймера1
#pragma vector = TIM1_CAPCOM_CC1IF_vector
__interrupt void TIM1_CAPCOM_CC1IF_handler(void)
{
	ADC1->CR1|= ADC1_CR1_ADON;		 // Wake-up/trigg the ADC 
	TIM1->SR1&=~MASK_TIM1_SR1_CC1IF;	   // clear compare flag
  return;  
 
}

/*-------------------------------------------------------------------------
Interrupt vector numbers 
C:\Program Files (x86)\IAR Systems\Embedded Workbench 6.5\stm8\inc\iostm8s103f3.h  
*-----------------------------------------------------------------------
#define AWU_vector                           0x03
#define SPI_TXE_vector                       0x0C
#define SPI_RXNE_vector                      0x0C
#define SPI_WKUP_vector                      0x0C
#define SPI_CRCERR_vector                    0x0C
#define SPI_OVR_vector                       0x0C
#define SPI_MODF_vector                      0x0C
#define TIM1_OVR_UIF_vector                  0x0D
#define TIM1_CAPCOM_BIF_vector               0x0D
#define TIM1_CAPCOM_TIF_vector               0x0D
#define TIM1_CAPCOM_CC1IF_vector             0x0E
#define TIM1_CAPCOM_CC2IF_vector             0x0E
#define TIM1_CAPCOM_CC3IF_vector             0x0E
#define TIM1_CAPCOM_CC4IF_vector             0x0E
#define TIM1_CAPCOM_COMIF_vector             0x0E
#define TIM2_OVR_UIF_vector                  0x0F
#define TIM2_CAPCOM_CC1IF_vector             0x10
#define TIM2_CAPCOM_TIF_vector               0x10
#define TIM2_CAPCOM_CC2IF_vector             0x10
#define TIM2_CAPCOM_CC3IF_vector             0x10
#define UART1_T_TXE_vector                   0x13
#define UART1_T_TC_vector                    0x13
#define UART1_R_OR_vector                    0x14
#define UART1_R_RXNE_vector                  0x14
#define UART1_R_IDLE_vector                  0x14
#define UART1_R_PE_vector                    0x14
#define UART1_R_LBDF_vector                  0x14
#define I2C_ADD10_vector                     0x15
#define I2C_ADDR_vector                      0x15
#define I2C_OVR_vector                       0x15
#define I2C_STOPF_vector                     0x15
#define I2C_BTF_vector                       0x15
#define I2C_WUFH_vector                      0x15
#define I2C_RXNE_vector                      0x15
#define I2C_TXE_vector                       0x15
#define I2C_BERR_vector                      0x15
#define I2C_ARLO_vector                      0x15
#define I2C_AF_vector                        0x15
#define I2C_SB_vector                        0x15
#define ADC1_AWS0_vector                     0x18
#define ADC1_AWS1_vector                     0x18
#define ADC1_AWS2_vector                     0x18
#define ADC1_AWS3_vector                     0x18
#define ADC1_AWS4_vector                     0x18
#define ADC1_AWS5_vector                     0x18
#define ADC1_AWS6_vector                     0x18
#define ADC1_EOC_vector                      0x18
#define ADC1_AWS8_vector                     0x18
#define ADC1_AWS9_vector                     0x18
#define ADC1_AWDG_vector                     0x18
#define ADC1_AWS7_vector                     0x18
#define TIM4_OVR_UIF_vector                  0x19
#define FLASH_EOP_vector                     0x1A
#define FLASH_WR_PG_DIS_vector               0x1A
*----------------------------------------------
*      End of file
*--------------------------------------------*/