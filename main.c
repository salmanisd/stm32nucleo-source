#include <stm32f4xx.h>

//Prototypes
void ms_delay(int ms);
unsigned int SPIsend(unsigned int data);
int set_pin_AF(int p);
int AF_sel(int p);
int set_ospeed(int p);
int set_pulldir(int p);
void SystemClock_Config(void);
__irq void DMA2_Stream4_IRQHandler(void);
__irq void DMA2_Stream3_IRQHandler(void);




void ms_delay(int ms) {
   while (ms-- > 0) {
      volatile int x=5971;
      while (x-- > 0)
         __asm("nop");
   }
}


//GLOBAL VARIABLES
static short j=10;
unsigned short adc_resultA[50];
unsigned short adc_resultB[50];

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


 unsigned int SPIsend(unsigned int data)
{
	int dummy=0;
	SPI1->DR=data;

	while( !(SPI1->SR & SPI_SR_TXE) );
/*	if( SPI1->DR & SPI_SR_OVR )
		while(1)
			;*/
	while(! (SPI1->SR & SPI_SR_RXNE));
	//
	
	//ms_delay(1);

	return (SPI1->DR);
	
	
}


__irq void DMA2_Stream3_IRQHandler(void)
{
  
  
  
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
 
 
 
 
void main () {

int i;
 SystemClock_Config();
  for(i=0;i<50;i++)
  {
    adc_resultA[i]=0xFF;
  }
    for(i=0;i<50;i++)
  {
    adc_resultB[i]=0xAA;
  }
  
  
    adc_resultA[0]=0xA5A5;
    adc_resultA[1]=0xA5A5;
  adc_resultA[48]=0xB9B9;
    adc_resultA[49]=0xB9B9;
    
    
    adc_resultB[0]=0x3838;
    adc_resultB[1]=0x3838;
  adc_resultB[48]=0x6565;
    adc_resultB[49]=0x6565;
  
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
        



	GPIOA->MODER |=set_pin_AF(5);
	GPIOA->AFR[0]|=AF_sel(5);
	GPIOA->OTYPER |=0;
	GPIOA->OSPEEDR |=set_ospeed(5); 
//	GPIOA->PUPDR|=set_pulldir(5);

	GPIOA->MODER |=set_pin_AF(6);
	GPIOA->AFR[0]|=AF_sel(6);
	GPIOA->OTYPER |=0;
	GPIOA->OSPEEDR |=set_ospeed(6); 
//	GPIOA->PUPDR|=set_pulldir(6);

	GPIOA->MODER |=set_pin_AF(7);
	GPIOA->AFR[0]|=AF_sel(7);
	GPIOA->OTYPER |=0;
	GPIOA->OSPEEDR |=set_ospeed(7); 
//	GPIOA->PUPDR|=set_pulldir(7);

	GPIOB->MODER |=(0x01<<12);
	GPIOB->OTYPER |=0;
	GPIOB->OSPEEDR |=set_ospeed(6); 
	GPIOB->BSRRL|=0x0040;                  //set portB pin6 as output=1,CS Disable (high)



//	GPIOB->BSRRL|=0x0040;                  //CS Disable (high)

   
     
    
	
	SPI1->CR1 |=SPI_CR1_DFF; //16 bit data frame
	SPI1->CR1 |=(0x0001<<3); // Baud Rate as  fpclk/8 (10.5Mhz) where fpclk is APB2 clock=84Mhz
		//	SPI1->CR1 |= SPI_CR1_SSM ;
	//	SPI1->CR1 |= SPI_CR1_SSI;                       
	SPI1->CR1 |=SPI_CR1_MSTR;						
		
	SPI1->CR2|=SPI_CR2_TXDMAEN; //DMA request when TX empty flag set
	SPI1->CR2|=SPI_CR2_SSOE;
	

		DMA2_Stream3->CR &= 0;
		while ( (DMA2_Stream3->CR !=0));
	
                //DMA CONFIG for SPI
		DMA2_Stream3->PAR |= (uint32_t)&SPI1->DR;
                DMA2_Stream3->M0AR |= (uint32_t)&adc_resultA[0]; 
              DMA2_Stream3->M1AR |= (uint32_t)&adc_resultB[0];
		DMA2_Stream3->NDTR =100;
		//DMA DOUBLE BUFFER
             DMA2_Stream3->CR |= DMA_SxCR_DBM; //Buffer switiching enabeld
//                DMA2_Stream3->CR |=DMA_SxCR_TCIE; //FUll transfer interrupt enabled
		DMA2_Stream3->CR |=(1<<11);   //Set Peripheral data size to 16bits
		DMA2_Stream3->CR |=(3<<16); //high prority
		DMA2_Stream3->CR |=(3<<25); //select channel 3         
		DMA2_Stream3->CR |=DMA_SxCR_MINC;
	//	DMA2_Stream3->CR |=DMA_SxCR_CIRC; //circular mode set for SPI
		DMA2_Stream3->CR |=(1<<6); //direction
		//      DMA2_Stream3->CR |= (1<<5) ; //[perh is flowcontroller
		
         
                //Emable DMA Stream for SPI
                DMA2_Stream3->CR |=DMA_SxCR_EN;

//	Enable SPI
	SPI1->CR1|=SPI_CR1_SPE;
	
     
        

    GPIOB->BSRRH|=0x0040 ; //CS Enable (low)

	
/*
while (1){
	
ms_delay(500);
    
SPIsend(ADC1->DR);
	  while (SPI1->SR & SPI_SR_BSY);
}*/
//	GPIOB->BSRRL|=0x0040;                  //CS Disable (high)
	
	
	
	
	
	
	
	
	
	
while(1);

}