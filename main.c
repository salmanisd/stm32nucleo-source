#include <stm32f4xx.h>

//Prototypes
void ms_delay(int ms);
unsigned int SPIsend(unsigned int data);
int set_pin_AF(int p);
int AF_sel(int p);
int set_ospeed(int p);
int set_pulldir(int p);

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

__irq void DMA2_Stream4_IRQHandler()
{

    //     ADC1->CR2^=(0x00000001);
         
	//TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
  TIM3->CR1 ^= (0x00000001);
	DMA2->HIFCR|=DMA_HIFCR_CTCIF4; //clear interrupt
    //    DMA2->HIFCR|=DMA_HIFCR_CHTIF4;//clear half transfer int
       
      
        
      if (  !(DMA2_Stream4->CR)&(DMA_SxCR_CT) ) //if current target is M0
    {
 DMA2_Stream3->M0AR = (uint32_t)&adc_resultA[0];
 
        }
      
       if (  (DMA2_Stream4->CR)&(DMA_SxCR_CT) )
        {
          DMA2_Stream3->M1AR = (uint32_t)&adc_resultA[0];
        }
				
			
}

void main () {

int i;
/*
  for(i=0;i<48;i++)
  {
    adc_resultA[i]=0xFF;
  }
  
   for(i=0;i<48;i++)
  {
    adc_resultB[i]=0xAA;
  }
  */
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
        

TIM3->PSC = 23999;	        // Set prescaler to 24 000 (PSC + 1)
TIM3->ARR = 4000;	          // Auto reload value 1000
TIM3->DIER |= TIM_DIER_UIE; // Enable update interrupt (timer level)
TIM3->CR1 |= TIM_CR1_CEN;   // Enable timer
         
   //      while (!(TIM3->SR & TIM_SR_UIF)); 

NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
NVIC_SetPriority ( DMA2_Stream4_IRQn,0); 
NVIC_EnableIRQ (DMA2_Stream4_IRQn); 
            

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
                DMA2_Stream4->NDTR |=0x00000032; //50 readings transfer
               //DMA DOUBLE BUFFER
                DMA2_Stream4->CR |= DMA_SxCR_DBM; //Buffer switiching enabeld
                DMA2_Stream4->CR |=DMA_SxCR_TCIE; //full transfer interrupt enabled
            //    DMA2_Stream4->CR |=DMA_SxCR_HTIE;//half transfer interrupt enabled
                
                DMA2_Stream4->CR |=(1<<11);   //Set Peripheral data size to 16bits
		DMA2_Stream4->CR |=(3<<16); //high prority
		DMA2_Stream4->CR |=(0<<25); //select channel 0
          //      DMA2_Stream4->CR |=DMA_SxCR_CIRC; //circular mode set
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
	SPI1->CR2|=SPI_CR2_SSOE;
	

		DMA2_Stream3->CR &= 0;
		while ( (DMA2_Stream3->CR !=0));
	
                //DMA CONFIG for SPI
		DMA2_Stream3->PAR |= (uint32_t)&SPI1->DR;
                DMA2_Stream3->M0AR |= (uint32_t)&adc_resultA[0];//dma_str[0];
      //          DMA2_Stream3->M1AR |= (uint32_t)&adc_resultB[0];
		DMA2_Stream3->NDTR |=0x0000000C;
		//DMA DOUBLE BUFFER
         //       DMA2_Stream3->CR |= DMA_SxCR_DBM; //Buffer switiching enabeld
   //             DMA2_Stream3->CR |=DMA_SxCR_TCIE; //Half transfer interrupt enabled
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
//	GPIOB->BSRRL|=0x0040;                  //CS Disable (high)
	
	
	
	
	
	
	
	
	
	
while(1);

}
