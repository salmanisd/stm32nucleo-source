#include <stm32f4xx.h>
#include <command_struct.h>

//Prototypes
void ms_delay(int ms);
//uint8_t SPIsend(uint8_t data);
unsigned short SPIsend(unsigned short data);

int set_pin_AF(int p);
int AF_sel(int p);
int set_ospeed(int p);
int set_pulldir(int p);


void suspend_SPITX_DMA(void);
void resume_SPITX_DMA(void);

void suspend_SPIRX_DMA(void);
void resume_SPIRX_DMA(void);

void spi_cs_enable(void);
void spi_cs_disable(void);

void enable_spi(void);
void disable_spi(void);

static void SystemClock_Config(void);

void enable_int_spi(void);
void disable_int_spi(void);

__irq void DMA2_Stream4_IRQHandler(void);
__irq void DMA2_Stream3_IRQHandler(void);
__irq void DMA2_Stream2_IRQHandler(void);

 __irq void SPI1_IRQHandler(void);


void ms_delay(int ms) {
   while (ms-- > 0) {
      volatile int x=5971;
      while (x-- > 0)
         __asm("nop");
   }
}


//GLOBAL VARIABLES
static short j=10;
unsigned short adc_resultA[350];
unsigned short adc_resultB[351];

unsigned int adc_ht_done=0;
unsigned int adc_tc_done=0;
   
unsigned int adc_htcnt=0;
unsigned int adc_tccnt=0;

unsigned short spi_cnt=0;
static int flag=0;

static int spdtxdma=0;
static int restxdma=0;
static int spirxINT=0;
static int flaggg=0;
volatile int h=0;
volatile int reset_flag=0;

 static short *ptr;
unsigned short recv_data[10];
unsigned short recv_cmd[20];

static void SystemClock_Config(void)

{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitTypeDef RCC_OscInitStruct;



  /* Enable Power Control clock */

  __PWR_CLK_ENABLE();

  

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  

  /* Enable HSI Oscillator and activate PLL with HSI as source */

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;

  RCC_OscInitStruct.HSIState = RCC_HSI_ON;

  RCC_OscInitStruct.HSICalibrationValue = 0x10;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

  RCC_OscInitStruct.PLL.PLLM = 16;

  RCC_OscInitStruct.PLL.PLLN = 336;

  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;

  RCC_OscInitStruct.PLL.PLLQ = 7;

  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)

  {



  }

 

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  

  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  

  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)

  {



  }

}
void enable_int_spi(void)
{
          SPI1->CR2|=SPI_CR2_TXDMAEN; //DMA request when TX empty flag set
  //    SPI1->CR2|=SPI_CR2_RXDMAEN; //Rx Buffer DMA Enable 
        
       SPI1->CR2|= SPI_CR2_RXNEIE;
}
void disable_int_spi(void)
{
   SPI1->CR2&=0xFFBF; //disable interrupt RXNEIE bit

}
void enable_spi(void)
{
  
  SPI1->CR1 &=0x00000000;
	SPI1->CR1 |=SPI_CR1_DFF; //16 bit data frame
	SPI1->CR1 |=SPI_CR1_BR_0; // Baud Rate as  fpclk/4 (21 Mhz) where fpclk is APB2 clock=84Mhz

	SPI1->CR1 |= SPI_CR1_SSM ;
	SPI1->CR1 |= SPI_CR1_SSI;                       
	SPI1->CR1 |=SPI_CR1_MSTR;
        

}



void disable_spi(void)
{
 // while(!(SPI1->SR & SPI_SR_RXNE));
  while(!(SPI1->SR & SPI_SR_TXE));

	while (SPI1->SR & SPI_SR_BSY);
          SPI1->CR1 &=0xFFFFFFFE;

}



	int set_pin_AF(int p)
	{
		p=2<<(p*2);
		return p;
	}

	int AF_sel(int p)
	{
		p=5<<(p*4); //5(0101) for AF=SPI
		return p;
	}

	int set_ospeed(int p)
	{
		p=1<<(p*2);
		return p;
	}

	
	int set_pulldir(int p) //Pin->Pulledup
	{
		p=1<<(p*2);
		return p;
	}
 
 
void spi_cs_enable(void)
{
    GPIOB->BSRRH|=0x0040 ; //CS Enable (low)
}

 void spi_cs_disable(void)
{
      GPIOB->BSRRL|=0x0040;                  //CS Disable (high) 
}


 __irq void DMA2_Stream3_IRQHandler()
{
//       if (DMA2->LISR & DMA_LISR_HTIF3)
//   {
//     adc_ht_done=0;
//     DMA2->LIFCR=DMA_LIFCR_CHTIF3;}
//   
//    if (DMA2->LISR & DMA_LISR_TCIF3)
//    {
//      adc_tc_done=0;
//      DMA2->LIFCR=DMA_LIFCR_CTCIF3;
//    }
        DMA2->LIFCR|=DMA_LIFCR_CTCIF3; //clear interrupt
        DMA2->LIFCR|=DMA_LIFCR_CHTIF3;
        
      spi_cnt++;


}
 __irq void DMA2_Stream4_IRQHandler()
 {
   
 
   
   if (DMA2->HISR & DMA_HISR_HTIF4)
   {  
     adc_ht_done=1;
     adc_htcnt++;
     DMA2->HIFCR=DMA_HIFCR_CHTIF4;
   }
   
    if (DMA2->HISR & DMA_HISR_TCIF4)
    {
      adc_tc_done=1;
        adc_resultA[349]=adc_resultA[349]+1;
        adc_tccnt++;
      DMA2->HIFCR=DMA_HIFCR_CTCIF4;
    }
  }

 
 
void main () {
  SystemClock_Config();

  adc_resultA[0]=0xA5A5;
  adc_resultA[1]=0xA5A5;
  adc_resultA[348]=0xB9B9;
//  adc_resultA[349]=0xB9B9;
adc_resultA[349]=0;

		//APB2=No predivisor=Max Clock=84Mhz
	//peripheral clock enable register ,enable SPI1 clock
	RCC->APB2ENR |=  RCC_APB2ENR_SPI1EN ; 
	
	//Enable ADC1 clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN ;                 
	
        //DMA2 Clock
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	
        //Enable TIM3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	//* Enbale GPIOA clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//* Enbale GPIOB clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
       



NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);


NVIC_SetPriority ( DMA2_Stream3_IRQn,4); 
NVIC_EnableIRQ (DMA2_Stream3_IRQn); 
//
NVIC_SetPriority ( DMA2_Stream4_IRQn,4); 
NVIC_EnableIRQ (DMA2_Stream4_IRQn);        




	GPIOA->MODER |=set_pin_AF(5);  //SPICLK
	GPIOA->AFR[0]|=AF_sel(5);
	GPIOA->OTYPER |=0;
	GPIOA->OSPEEDR |=set_ospeed(5); 
//	GPIOA->PUPDR|=set_pulldir(5);

	GPIOA->MODER |=set_pin_AF(6);    //MISO
	GPIOA->AFR[0]|=AF_sel(6);
	GPIOA->OTYPER |=0;
	GPIOA->OSPEEDR |=set_ospeed(6); 
//	GPIOA->PUPDR|=set_pulldir(6);
//GPIOA->PUPDR|=0x1000; //pullup when Slave missing
//GPIOA->PUPDR|=0x2000;// pulldown


	GPIOA->MODER |=set_pin_AF(7);   //MOSI
	GPIOA->AFR[0]|=AF_sel(7);
	GPIOA->OTYPER |=0;
	GPIOA->OSPEEDR |=set_ospeed(7); 
//	GPIOA->PUPDR|=set_pulldir(7);

	GPIOB->MODER |=(0x01<<12);
	GPIOB->OTYPER |=0;
	GPIOB->OSPEEDR |=set_ospeed(6); 
	GPIOB->BSRRL|=0x0040;                  //set portB pin6 as output=1,CS Disable (high)

  //      GPIOB->PUPDR|=0x1000; //pullup;
 //       GPIOB->PUPDR|=0x2000;//pulldown
        
	GPIOA->MODER |= 0X0000000F; //FOR ADC MCU pins PA0 and PA1 set to analog mode


              
/****************************************************************************************************/
        /* DMA Config For ADC */
/****************************************************************************************************/            
             
      	//DMA CONFIG FOR ADC
                DMA2_Stream4->CR &= 0;
		while ( (DMA2_Stream4->CR !=0));
                
		DMA2->LISR &= 0;
		DMA2->HISR &= 0;
       
                DMA2_Stream4->PAR |= (uint32_t)&ADC1->DR;
                DMA2_Stream4->M0AR = (uint32_t)&adc_resultA[2];
                DMA2_Stream4->NDTR =346; //46 readings transfer
         
              DMA2_Stream4->CR |=DMA_SxCR_TCIE; //full transfer interrupt enabled
               DMA2_Stream4->CR |=DMA_SxCR_HTIE;//half transfer interrupt enabled
                
                DMA2_Stream4->CR |=(1<<11);   //Set Peripheral data size to 16bits
		DMA2_Stream4->CR |=DMA_SxCR_PL_1;     // High prority
          //      DMA2_Stream4->CR |=DMA_SxCR_PL_0; 
		DMA2_Stream4->CR |=(0<<25); //select channel 0
            DMA2_Stream4->CR |=DMA_SxCR_CIRC; //circular mode set
                DMA2_Stream4->CR |=DMA_SxCR_MINC; //memory increment
		//	DMA2_Stream4->CR |=(1<<6); //direction
                //DMA2_Stream4->CR |= (1<<5) ; //[perh is flowcontroller
                 //Emable DMA Stream for ADC
               DMA2_Stream4->CR |=DMA_SxCR_EN;
        
/****************************************************************************************************/
              
        
        
        ADC->CCR |= ADC_CCR_ADCPRE_0;
        ADC->CCR |=  ADC_CCR_ADCPRE_1;
	ADC1->SQR3|=0x00000001;
	
	ADC1->CR2 |= ADC_CR2_ADON;  //ADC ON
        ADC1->CR2 |= ADC_CR2_CONT;   //continous conversion until bit cleared
        ADC1->CR2 |=ADC_CR2_DMA; //use DMA for data transfer
	ADC1->CR2 |=ADC_CR2_DDS; //DMA requests are issued as long as data is converted and DMA=1
        ADC1->CR2 |=ADC_CR2_SWSTART;	//Start conversion
        while ((ADC_SR_EOC)==0);  //end of conversion,(EOC=0) not completed
	
    
							
	 enable_spi();


/****************************************************************************************************/
        /* DMA Config For SPI_TX */
/****************************************************************************************************/
		DMA2_Stream3->CR &= 0;
		while ( (DMA2_Stream3->CR !=0));
	
                //DMA CONFIG for SPI_TX
		DMA2_Stream3->PAR |= (uint32_t)&SPI1->DR;
               DMA2_Stream3->M0AR |= (uint32_t)&adc_resultA[0]; 
		DMA2_Stream3->NDTR =175;
		//DMA DOUBLE BUFFER
        //        DMA2_Stream3->CR |= DMA_SxCR_DBM; //Buffer switiching enabeld
               DMA2_Stream3->CR |=DMA_SxCR_TCIE; //FUll transfer interrupt enabled
      //      DMA2_Stream3->CR |=DMA_SxCR_HTIE;//half transfer interrupt enabled
		DMA2_Stream3->CR |=(1<<11);   //Set Peripheral data size to 16bits
		DMA2_Stream3->CR |=DMA_SxCR_PL_1;     //Very High prority
                DMA2_Stream3->CR |=DMA_SxCR_PL_0; 
		DMA2_Stream3->CR |=(3<<25); //select channel 3         
		DMA2_Stream3->CR |=DMA_SxCR_MINC;
	//	DMA2_Stream3->CR |=DMA_SxCR_CIRC; //circular mode set for SPI
		DMA2_Stream3->CR |=(1<<6); //direction
		//      DMA2_Stream3->CR |= (1<<5) ; //[perh is flowcontroller
		
         
                //Emable DMA Stream for SPI
       //     DMA2_Stream3->CR |=DMA_SxCR_EN;
                
/****************************************************************************************************/              
   
             

         enable_int_spi();
//	Enable SPI
	SPI1->CR1|=SPI_CR1_SPE;
                spi_cs_enable();



	






 while(1){
   
   if (adc_ht_done==1)
     
    {
      DMA2_Stream3->M0AR = (uint32_t)&adc_resultA[0]; 
      DMA2_Stream3->NDTR =175;
      DMA2_Stream3->CR |=DMA_SxCR_EN;
      SPI1->CR1|=SPI_CR1_SPE;
      adc_ht_done=0;
    }
    
     if (adc_tc_done==1)
    {
      DMA2_Stream3->M0AR = (uint32_t)&adc_resultA[175]; 
      DMA2_Stream3->NDTR =175;
      DMA2_Stream3->CR |=DMA_SxCR_EN;
      SPI1->CR1|=SPI_CR1_SPE;
      adc_tc_done=0;
    }
                }
}