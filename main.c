#include <stm32f4xx.h>

//Prototypes
void ms_delay(int ms);
//uint8_t SPIsend(uint8_t data);
unsigned char SPIsend(unsigned char data);

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

void spi_master(void);
void spi_slave(void);

void submode0 (void);
void submode1 (void);
void submode2 (void);
void submode3 (void);

__irq void DMA2_Stream4_IRQHandler(void);
__irq void DMA2_Stream3_IRQHandler(void);
__irq void DMA2_Stream2_IRQHandler(void);




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
short adc_resultC[50];

short test_bufA[50];
short test_bufB[50];

short recv_data[50];

unsigned long shadow_ndtr;
unsigned long shadow_ndtr1;
unsigned int suspend_flag=0;
unsigned int resume_flag=0;

void spi_master(void)
{
  
        GPIOB->MODER |=(0x01<<12);
	GPIOB->OTYPER |=0;
	GPIOB->OSPEEDR |=set_ospeed(6); 
	GPIOB->BSRRL|=0x0040;                  //set portB pin6 as output=1,CS Disable (high)
  
	SPI1->CR1 |=SPI_CR1_BR_0; // Baud Rate as  fpclk/4 (21 Mhz) where fpclk is APB2 clock=84Mhz
	SPI1->CR1 |= SPI_CR1_SSM ; //SSM=0 for STM Slave mode
	SPI1->CR1 |= SPI_CR1_SSI;                       
	SPI1->CR1 |=SPI_CR1_MSTR;
       	SPI1->CR2|=SPI_CR2_SSOE;

}



void submode0 (void)
{
  //       SPI1->CR1 |=SPI_CR1_CPHA;
 //    SPI1->CR1 |=SPI_CR1_CPOL;
}
void submode1 (void)
{
         SPI1->CR1 |=SPI_CR1_CPHA;
 //    SPI1->CR1 |=SPI_CR1_CPOL;
}
void submode2 (void)
{
  //       SPI1->CR1 |=SPI_CR1_CPHA;
     SPI1->CR1 |=SPI_CR1_CPOL;
}
void submode3 (void)
{
         SPI1->CR1 |=SPI_CR1_CPHA;
     SPI1->CR1 |=SPI_CR1_CPOL;
}
void suspend_SPITX_DMA(void)
{
//Emable DMA Stream for SPI
              DMA2_Stream3->CR &=0xFFFFFFFE;  //toggle EN bit from 1 to 0
         //         DMA2_Stream3->CR ^=DMA_SxCR_EN;  //toggle EN bit from 1 to 0
while ( DMA2_Stream3->CR & DMA_SxCR_EN ); //break out when DMA_SxCR_EN==0

shadow_ndtr=DMA2_Stream3->NDTR;
                

                 
}
void resume_SPITX_DMA(void)
{
                DMA2->LIFCR=DMA_LIFCR_CTCIF3; //clear interrupt
                DMA2->LIFCR=DMA_LIFCR_CFEIF3; //clear interrupt
                
                  DMA2_Stream3->M0AR |= (uint32_t)&adc_resultA[0]; 
                DMA2_Stream3->M1AR |= (uint32_t)&adc_resultB[0];
		DMA2_Stream3->NDTR |=0x00000032;
                
                      shadow_ndtr1=DMA2_Stream3->NDTR;

           resume_flag++;

                DMA2_Stream3->CR |=DMA_SxCR_EN;  //toggle EN bit from 0 to 1
                
      //          while (! (DMA2_Stream3->CR & DMA_SxCR_EN) ); //break out when DMA_SxCR_EN==1
             
}

void suspend_SPIRX_DMA(void)
{
//Emable DMA Stream for SPI
              DMA2_Stream2->CR &=0xFFFFFFFE;  //toggle EN bit from 1 to 0
         //         DMA2_Stream2->CR ^=DMA_SxCR_EN;  //toggle EN bit from 1 to 0
while ( DMA2_Stream2->CR & DMA_SxCR_EN ); //break out when DMA_SxCR_EN==0

               
                 
}

void resume_SPIRX_DMA(void)
{
                DMA2->LIFCR=DMA_LIFCR_CTCIF2; //clear interrupt
                DMA2->LIFCR=DMA_LIFCR_CFEIF2; //clear interrupt
                		
                DMA2_Stream2->PAR |= (uint32_t)&SPI1->DR;
                DMA2_Stream2->M0AR |= (uint32_t)&recv_data[0]; 
		DMA2_Stream2->NDTR |=0x0000004;

                DMA2_Stream2->CR |=DMA_SxCR_EN;  //toggle EN bit from 0 to 1
                
              while (! (DMA2_Stream2->CR & DMA_SxCR_EN) ); //break out when DMA_SxCR_EN==1
             
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

 /*
__irq void DMA2_Stream4_IRQHandler()
{
    //     ADC1->CR2^=(0x00000001);
         
	//TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
  TIM3->CR1 ^= (0x00000001);
	DMA2->HIFCR|=DMA_HIFCR_CTCIF4; //clear interrupt
        DMA2->LIFCR|=DMA_LIFCR_CTEIF3; //clear interrupt
        DMA2_Stream3->CR |=DMA_SxCR_EN;
    //    DMA2->HIFCR|=DMA_HIFCR_CHTIF4;//clear half transfer int
      //    DMA2_Stream3->M1AR |= (uint32_t)&adc_resultA[0];
        
        if (  !(DMA2_Stream4->CR)&(DMA_SxCR_CT) ) //if ADC current target is M0(CT==0),DMA_SxM1AR(ADC Buffer B) can be given to SPI DMA
    {   
         if (  !(DMA2_Stream3->CR)&(DMA_SxCR_CT) )//if SPI current target is M0(CT==0),give ADC DMA_SxM1AR to SPI DMA M1AR
         {  
            DMA2_Stream3->M1AR&=0;
            DMA2_Stream3->M1AR |= (uint32_t)&adc_resultB[0];
            DMA2_Stream3->NDTR |=0x00000032;
        }
        else
        {
            DMA2_Stream3->M0AR&=0;
            DMA2_Stream3->M0AR |= (uint32_t)&adc_resultB[0];
            DMA2_Stream3->NDTR |=0x00000032;
       
        }
}
       if (  (DMA2_Stream4->CR)&(DMA_SxCR_CT) )//if ADC current target is M1(CT==1),DMA_SxM0AR(ADC buffer A) can be given to SPI DMA
       {       
         if (  !(DMA2_Stream3->CR)&(DMA_SxCR_CT) )//if SPI current target is M0(CT==0),give ADC DMA_SxM1AR to SPI DMA M1AR
         {  
            DMA2_Stream3->M1AR&=0;
            DMA2_Stream3->M1AR |= (uint32_t)&adc_resultA[0];
            DMA2_Stream3->NDTR |=0x00000032;
        }
        else
        {
            DMA2_Stream3->M0AR&=0;
            DMA2_Stream3->M0AR |= (uint32_t)&adc_resultA[0];
            DMA2_Stream3->NDTR |=0x00000032;
       
        }
      
       }			
			
}
*/
 __irq void DMA2_Stream3_IRQHandler()
{
        DMA2->LIFCR=DMA_LIFCR_CTCIF3; //clear interrupt
        while(!(SPI1->SR & SPI_SR_TXE)); //when NDTR==0,this interrupt IRQ is called but the last word is still under transmission 
                                         //so wait for it to complete and when TXE is set,proceed with SPIsend transmission
  const char *test="Talaria"; 
    int i=0;
     while(test[i]!=0)
     {
       SPIsend(test[i++]);
     }
}

__irq void DMA2_Stream2_IRQHandler()
{
   	DMA2->LIFCR|=DMA_LIFCR_CTCIF2; //clear interrupt
        DMA2->LIFCR|=DMA_LIFCR_CHTIF2; //clear interrupt
        DMA2->LIFCR|=DMA_LIFCR_CFEIF2; //clear interrupt
                  suspend_flag++;

     if (recv_data[0]==0x4142)
		 {
			 ms_delay(3000);

       suspend_SPITX_DMA();
     }
    

}
 
 unsigned char SPIsend(unsigned char data)

{
  unsigned char temp;
   while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR=data;
  //while (SPI1->SR & SPI_SR_BSY);

   
     while(!(SPI1->SR & SPI_SR_RXNE));
temp=SPI1->DR ;
  
        return temp;
}
void main () {

int i=0;

  for(i=0;i<48;i++)
  {
    adc_resultA[i]=0xFF;
  }
  
   for(i=0;i<48;i++)
  {
    adc_resultB[i]=0xAA;
  }
   for(i=0;i<48;i++)
  {
    adc_resultC[i]=0x33;
  }
  
    for(i=0;i<50;i++)
  {
  test_bufA[i]=i+1;
  }
     for(i=0;i<50;i++)
  {
  test_bufB[i]=i+201;
  }
  
  
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

NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
NVIC_SetPriority ( DMA2_Stream3_IRQn,4); 
NVIC_EnableIRQ (DMA2_Stream3_IRQn); 
NVIC_SetPriority ( DMA2_Stream2_IRQn,4); 
NVIC_EnableIRQ (DMA2_Stream2_IRQn);            

	GPIOA->MODER |=set_pin_AF(5);
	GPIOA->AFR[0]|=AF_sel(5);
	GPIOA->OTYPER |=0;
	GPIOA->OSPEEDR |=set_ospeed(5); 
//	GPIOA->PUPDR|=set_pulldir(5);
//GPIOA->PUPDR|=0x400; //pullup
//GPIOA->PUPDR|=0x800; //pulldown

	GPIOA->MODER |=set_pin_AF(6);
	GPIOA->AFR[0]|=AF_sel(6);
	GPIOA->OTYPER |=0;
//        GPIOA->OTYPER |=0x40; //output open drain
	GPIOA->OSPEEDR |=set_ospeed(6); 
 //       GPIOA->OSPEEDR |=0x2000;
//	GPIOA->PUPDR|=set_pulldir(6);
//GPIOA->PUPDR|=0x2000; //pull down
//GPIOA->PUPDR|=0x1000;//pull up
	
        GPIOA->MODER |=set_pin_AF(7);
	GPIOA->AFR[0]|=AF_sel(7);
	GPIOA->OTYPER |=0;
	GPIOA->OSPEEDR |=set_ospeed(7); 
//	GPIOA->PUPDR|=set_pulldir(7);

	

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
        
/****************************************************************************************************/
              
        
        
	ADC->CCR|=0x00030000; //ADC Prescaler set to 8 (PCLK2/8) where pclk2 is 84Mhz
	ADC1->SQR3|=0x00000001;
	
	ADC1->CR2 |= ADC_CR2_ADON;  //ADC ON
        ADC1->CR2 |= ADC_CR2_CONT;   //continous conversion until bit cleared
        ADC1->CR2 |=ADC_CR2_DMA; //use DMA for data transfer
	ADC1->CR2 |=ADC_CR2_DDS; //DMA requests are issued as long as data is converted and DMA=1
        ADC1->CR2 |=ADC_CR2_SWSTART;	//Start conversion
        while ((ADC_SR_EOC)==0);  //end of conversion,(EOC=0) not completed
	
    SPI1->CR1 &=0x00000000;
	SPI1->CR1 |=SPI_CR1_DFF; //16 bit data frame
   	            

      SPI1->CR2|=SPI_CR2_TXDMAEN; //DMA request when TX empty flag set
     SPI1->CR2|=SPI_CR2_RXDMAEN; //Rx Buffer DMA Enable 

	
/****************************************************************************************************/
        /* DMA Config For SPI_TX */
/****************************************************************************************************/
		DMA2_Stream3->CR &= 0;
		while ( (DMA2_Stream3->CR !=0));
	
                //DMA CONFIG for SPI_TX
		DMA2_Stream3->PAR |= (uint32_t)&SPI1->DR;
              DMA2_Stream3->M0AR |= (uint32_t)&adc_resultA[0]; 
               DMA2_Stream3->M1AR |= (uint32_t)&adc_resultB[0];
      //          DMA2_Stream3->M0AR |= (uint32_t)&test_bufA[0]; 
      //          DMA2_Stream3->M1AR |= (uint32_t)&test_bufB[0];
		DMA2_Stream3->NDTR |=50;
		//DMA DOUBLE BUFFER
                DMA2_Stream3->CR |= DMA_SxCR_DBM; //Buffer switiching enabeld
            //    DMA2_Stream3->CR |=DMA_SxCR_TCIE; //FUll transfer interrupt enabled
		DMA2_Stream3->CR |=(1<<11);   //Set Peripheral data size to 16bits
		DMA2_Stream3->CR |=(3<<16); //high prority
		DMA2_Stream3->CR |=(3<<25); //select channel 3         
		DMA2_Stream3->CR |=DMA_SxCR_MINC;
	//	DMA2_Stream3->CR |=DMA_SxCR_CIRC; //circular mode set for SPI
		DMA2_Stream3->CR |=(1<<6); //direction
		//      DMA2_Stream3->CR |= (1<<5) ; //[perh is flowcontroller
		
         
                //Emable DMA Stream for SPI
         //       DMA2_Stream3->CR |=DMA_SxCR_EN;
                
/****************************************************************************************************/              
   
                
                
/****************************************************************************************************/
        /* DMA Config For SPI_RX */
/****************************************************************************************************/               
                DMA2_Stream2->CR &= 0;
		while ( (DMA2_Stream2->CR !=0));
                
                //DMA CONFIG for SPI_RX
		DMA2_Stream2->PAR |= (uint32_t)&SPI1->DR;
              DMA2_Stream2->M0AR |= (uint32_t)&recv_data[0]; 
		DMA2_Stream2->NDTR |=50;
		//DMA DOUBLE BUFFER
        //        DMA2_Stream2->CR |=DMA_SxCR_TCIE; //FUll transfer interrupt enabled
		DMA2_Stream2->CR |=(1<<11);   //Set Peripheral data size to 16bits
		DMA2_Stream2->CR |=(3<<16); //high prority
		DMA2_Stream2->CR |=(3<<25); //select channel 3         
		DMA2_Stream2->CR |=DMA_SxCR_MINC;
                
		DMA2_Stream2->CR |=DMA_SxCR_CIRC; //circular mode set for SPI
	//	DMA2_Stream2->CR |=(1<<6); //direction
	//      DMA2_Stream2->CR |= (1<<5) ; //[perh is flowcontroller
		
         
                //Emable DMA Stream for SPIRX
        //        DMA2_Stream2->CR |=DMA_SxCR_EN;
         //        while (! (DMA2_Stream2->CR & DMA_SxCR_EN) ); //break out when DMA_SxCR_EN==1
/****************************************************************************************************/                
    //        spi_master();
                
    //            spi_cs_enable();
 //      SPI1->CR1 |=SPI_CR1_CPHA;

       
        
//	Enable SPI
	SPI1->CR1|=SPI_CR1_SPE;
        
          resume_SPIRX_DMA();
      resume_SPITX_DMA();
//
//        SPI1->DR=0x46;
//            while(!(SPI1->SR & SPI_SR_RXNE));
//recv_data[0]=SPI1->DR ;
//        while(!(SPI1->SR & SPI_SR_TXE));
//  	
//SPI1->DR=0xED;
//            while(!(SPI1->SR & SPI_SR_RXNE));
//recv_data[1]=SPI1->DR ;
   

 

	//        SPIsend(11);	
  //     resume_SPIRX_DMA();
//       resume_SPITX_DMA();
    //    SPIsend(0x00);
  //      SPI1->DR=0x00F6;
        
 //       recv_data[0]=    SPIsend(0xED);
         //   recv_data[1]=    SPIsend(0x0021);
      //          recv_data[3]=    SPIsend(0x7C9D);
      //    SPIsend(0x01);
 
       

	
while(1);
}