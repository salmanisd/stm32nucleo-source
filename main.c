#include <stm32f4xx.h>

void ms_delay(int ms);
 unsigned int SPIsend(unsigned int data);
 int set_pin_AF(int p);
   int AF_sel(int p);
     int set_ospeed(int p);
       int set_pulldir(int p);
   void DMA2_Stream4_IRQHandler(void);
/////
	void ms_delay(int ms) {
   while (ms-- > 0) {
      volatile int x=5971;
      while (x-- > 0)
         __asm("nop");
   }
}


//GLOBAL VARIABLES
static int j=10;
short adc_resultA[50];
short adc_resultB[50];



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

 

//void DMA2_Stream4_IRQHandler(void)
//{
//HAL_DMA_IRQHandler();
//}
__irq void IRQHandler (void);

__irq void IRQHandler ()
{
   //volatile unsigned int *base = (unsigned int *) 0x80000000;
  //   if (*base == 0x000000B4)   
	//	 {			 // which interrupt was it?
        TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
			ADC1->CR2 |= 0xFFFF; 
	//	 }   
    
	//	if(TIM3->SR & TIM_SR_UIF) // if UIF flag is set	
		
}
void main () {
static int dummy=0;
	
	//peripheral clock enable register ,enable SPI1 clock
	RCC->APB2ENR |=  RCC_APB2ENR_SPI1EN ; 
	
	//Enable ADC1 clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN ;                 
	
        //DMA2 Clock
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;


TIM3->PSC = 23999;	        // Set prescaler to 24 000 (PSC + 1)
TIM3->ARR = 3000;	          // Auto reload value 1000
TIM3->DIER |= TIM_DIER_UIE; // Enable update interrupt (timer level)
TIM3->CR1 |= TIM_CR1_CEN;   // Enable timer


HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
HAL_NVIC_SetPriority ( TIM3_IRQn, 0, 0); 
HAL_NVIC_EnableIRQ (TIM3_IRQn); 
            

		//APB2=No predivisor=Max Clock=84Mhz
	
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

	GPIOA->MODER |= 0X0000000F; //FOR ADC MCU pins PA0 and PA1 set to analog mode


//	GPIOB->BSRRL|=0x0040;                  //CS Disable (high)


						
		//DMA CONFIG FOR ADC
                DMA2_Stream4->CR &= 0;
		while ( (DMA2_Stream4->CR !=0));
                
		DMA2->LISR &= 0;
		DMA2->HISR &= 0;
       
                DMA2_Stream4->PAR |= (uint32_t)&ADC1->DR;
                DMA2_Stream4->M0AR |= (uint32_t)&adc_resultA[0];
                DMA2_Stream4->M1AR |= (uint32_t)&adc_resultB[0];
                DMA2_Stream4->NDTR |=0x320; //50 x 16 items transfer
		        
               //DMA DOUBLE BUFFER
        //        DMA2_Stream4->CR |= DMA_SxCR_DBM; //Buffer switiching enabeld
                
                DMA2_Stream4->CR |=(1<<11);   //Set Peripheral data size to 16bits
		DMA2_Stream4->CR |=(3<<16); //high prority
		DMA2_Stream4->CR |=(0<<25); //select channel 0
                DMA2_Stream4->CR |=DMA_SxCR_CIRC; //circular mode set
                DMA2_Stream4->CR |=DMA_SxCR_MINC; //memory increment
							//	DMA2_Stream4->CR |=(1<<6); //direction
                //DMA2_Stream4->CR |= (1<<5) ; //[perh is flowcontroller
                 //Emable DMA Stream for ADC
               DMA2_Stream4->CR |=DMA_SxCR_EN;
        

               
        
        
	ADC->CCR|=0x00030000; //ADC Prescaler set to 8 (PCLK2/8) where pclk2 is 84Mhz
	ADC1->SQR3|=0x00000001;
	
	ADC1->CR2 |= ADC_CR2_ADON;  //ADC ON
        ADC1->CR2 |= ADC_CR2_CONT;   //continous conversion until bit cleared
        ADC1->CR2 |=ADC_CR2_DMA; //use DMA for data transfer
	ADC1->CR2 |=ADC_CR2_DDS; //DMA requests are issued as long as data is converted and DMA=1
        ADC1->CR2 |=ADC_CR2_SWSTART;	//Start conversion
        while ((ADC_SR_EOC)==0);  //end of conversion,(EOC=0) not completed
	
       
	
	
        SPI1->CR1 |=SPI_CR1_DFF; //16 bit data frame
	SPI1->CR1 |=(0x0002<<3); // Baud Rate as  fpclk/8 (10.5Mhz) where fpclk is APB2 clock=84Mhz
		//	SPI1->CR1 |= SPI_CR1_SSM ;
		//	SPI1->CR1 |= SPI_CR1_SSI;                       
	SPI1->CR1 |=SPI_CR1_MSTR;						
		
	SPI1->CR2|=SPI_CR2_TXDMAEN; //DMA request when TX empty flag set
	SPI1->CR2|=   SPI_CR2_SSOE;
	
	char dma_str[]="Testing SPI and then wireless communication from STM to  CC3200";
	
             
		
		DMA2_Stream3->CR &= 0;
		while ( (DMA2_Stream3->CR !=0));
	
                //DMA CONFIG for SPI
		DMA2_Stream3->PAR |= (uint32_t)&SPI1->DR;
                DMA2_Stream3->M0AR |= (uint32_t)&adc_resultA[0];//dma_str[0];
		DMA2_Stream3->NDTR |=0x10;
		
		DMA2_Stream3->CR |=(1<<11);   //Set Peripheral data size to 16bits
		DMA2_Stream3->CR |=(3<<16); //high prority
		DMA2_Stream3->CR |=(3<<25); //select channel 3         
		//	DMA2_Stream3->CR |=DMA_SxCR_MINC;
		DMA2_Stream3->CR |=DMA_SxCR_CIRC; //circular mode set for SPI
		DMA2_Stream3->CR |=(1<<6); //direction
		//      DMA2_Stream3->CR |= (1<<5) ; //[perh is flowcontroller
		
         
                //Emable DMA Stream for SPI
              DMA2_Stream3->CR |=DMA_SxCR_EN;
	

//	Enable SPI
	SPI1->CR1|=SPI_CR1_SPE;
        

  //  GPIOB->BSRRH|=0x0040 ; //CS Enable (low)

	
/*
while (1){
	
ms_delay(500);
    

SPIsend(ADC1->DR);
	  while (SPI1->SR & SPI_SR_BSY);

}*/
/*
	int j=0;
	int i=0;
	const char *str="Testing SPI-1 communication from STM32 Nucleo on Keil to CC3200 on CCS-SPI Clock 10 Mhz ";
	
	 while(str[i] != 0) {
        SPIsend(str[i++]);	
    }
	  while (SPI1->SR & SPI_SR_BSY);
	
*/

//	GPIOB->BSRRL|=0x0040;                  //CS Disable (high)
	
	
	
	
	
	
	
	
	
	
while(1);


}
