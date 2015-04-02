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
unsigned short adc_resultA[351];
unsigned short adc_resultB[351];

short test_bufA[50];
short test_bufB[50];
   
unsigned int saveNDTR;
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
	SPI1->CR1 |=SPI_CR1_BR_1; // Baud Rate as  fpclk/4 (21 Mhz) where fpclk is APB2 clock=84Mhz

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
void suspend_SPITX_DMA(void)
{
    
  spdtxdma++;
//while(!(SPI1->SR & SPI_SR_TXE));
// while(!(SPI1->SR & SPI_SR_RXNE));

 //Emable DMA Stream for SPI
              DMA2_Stream3->CR &=0xFFFFFFFE;  //toggle EN bit from 1 to 0
         //         DMA2_Stream3->CR ^=DMA_SxCR_EN;  //toggle EN bit from 1 to 0
while ( DMA2_Stream3->CR & DMA_SxCR_EN ); //break out when DMA_SxCR_EN==0

                

                 
}
void resume_SPITX_DMA(void)
{ 
  restxdma++;
                DMA2->LIFCR=DMA_LIFCR_CTCIF3; //clear interrupt
                DMA2->LIFCR=DMA_LIFCR_CFEIF3; //clear interrupt
           
                  DMA2_Stream3->M0AR = (uint32_t)&adc_resultA[0]; 
                DMA2_Stream3->M1AR = (uint32_t)&adc_resultB[0];
//	
////                                DMA2_Stream3->M0AR = (uint32_t)&test_bufA[0]; 
////                               DMA2_Stream3->M1AR = (uint32_t)&test_bufB[0]; 
//
//
               DMA2_Stream3->NDTR =351;
                 saveNDTR=DMA2_Stream3->NDTR;


                DMA2_Stream3->CR |=DMA_SxCR_EN;  //toggle EN bit from 0 to 1
                
              while (! (DMA2_Stream3->CR & DMA_SxCR_EN) ); //break out when DMA_SxCR_EN==1
             
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
                		
                DMA2_Stream2->PAR = (uint32_t)&SPI1->DR;
                DMA2_Stream2->M0AR = (uint32_t)&recv_data[0]; 
		DMA2_Stream2->NDTR |=10;

                DMA2_Stream2->CR |=DMA_SxCR_EN;  //toggle EN bit from 0 to 1
                
              while (! (DMA2_Stream2->CR & DMA_SxCR_EN) ); //break out when DMA_SxCR_EN==1
             
}



unsigned short SPIsend(unsigned short data)
{
  while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR=data;

	

	while(!(SPI1->SR & SPI_SR_RXNE));
//	while (SPI1->SR & SPI_SR_BSY);

	return (SPI1->DR);
//	return 0;
	
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
        DMA2->LIFCR|=DMA_LIFCR_CTCIF3; //clear interrupt
        DMA2->LIFCR|=DMA_LIFCR_CHTIF3;
//        while(!(SPI1->SR & SPI_SR_TXE)); //when NDTR==0,this interrupt IRQ is called but the last word is still under transmission 
//                                         //so wait for it to complete and when TXE is set,proceed with SPIsend transmission
//  
//	while(!(SPI1->SR & SPI_SR_RXNE));
//spi_cs_disable();
        if ( (DMA2_Stream4->CR)&(DMA_SxCR_CT) ) //ct=adc_resultA
             adc_resultB[350]=adc_resultB[350]+1;
        else //ct=adc_resultB
             adc_resultA[350]=adc_resultA[350]+1;

        
        

}

__irq void DMA2_Stream2_IRQHandler()
{
   	DMA2->LIFCR|=DMA_LIFCR_CTCIF2; //clear interrupt
        DMA2->LIFCR|=DMA_LIFCR_CHTIF2; //clear interrupt
        DMA2->LIFCR|=DMA_LIFCR_CFEIF2; //clear interrupt

        if( (recv_data[0]!=0x0000) )
     {
      h=1;
      reset_flag=1;
      spirxINT++;
     }
   else
   {
     h=0;
   }

}
 __irq void SPI1_IRQHandler()
 {
   
 // SPI1->CR2&=0xFFBF;
   *ptr=SPI1->DR;
 
  // SPI1->CR2&=0xFFBF; //disable interrupt RXNEIE bit


if (*ptr!=0x0000)
{
   //disable interrupt RXNEIE bit
  h=1;
 // ptr++;
}
else
{
  h=0;
//SPI1->CR2|= SPI_CR2_RXNEIE;  
}
          
 //SPI1->CR2|= SPI_CR2_RXNEIE;
 }
 
void main () {
  SystemClock_Config();
ptr=&recv_data[0];
int i=0;

//  for(i=0;i<48;i++)
//  {
//    adc_resultA[i]=0xFF;
//  }
//  
//   for(i=0;i<48;i++)
//  {
//    adc_resultB[i]=0xAA;
//  }
//   for(i=0;i<48;i++)
//  {
//    adc_resultC[i]=0x33;
//  }
//  
//    for(i=2;i<50;i++)
//  {
//  test_bufA[i]=i;
//  }
//     for(i=0;i<50;i++)
//  {
//  test_bufB[i]=i+51;
//  }
  adc_resultA[0]=0xA5A5;
  adc_resultA[1]=0xA5A5;
   adc_resultA[350]=0;

  adc_resultB[0]=0xB9B9;
 adc_resultB[1]=0xB9B9;
   adc_resultB[350]=0;

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
       

//TIM3->PSC = 23999;	        // Set prescaler to 24 000 (PSC + 1)
//TIM3->ARR = 4000;	          // Auto reload value 1000
//TIM3->DIER |= TIM_DIER_UIE; // Enable update interrupt (timer level)
//TIM3->CR1 |= TIM_CR1_CEN;   // Enable timer
         
   //      while (!(TIM3->SR & TIM_SR_UIF)); 

NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);


NVIC_SetPriority ( DMA2_Stream3_IRQn,4); 
NVIC_EnableIRQ (DMA2_Stream3_IRQn); 
//
//NVIC_SetPriority ( DMA2_Stream2_IRQn,4); 
//NVIC_EnableIRQ (DMA2_Stream2_IRQn);        

NVIC_SetPriority ( SPI1_IRQn,4); 
NVIC_EnableIRQ (SPI1_IRQn); 


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
              DMA2_Stream4->M1AR = (uint32_t)&adc_resultB[2];
                DMA2_Stream4->NDTR =348; //46 readings transfer
               //DMA DOUBLE BUFFER
             DMA2_Stream4->CR |= DMA_SxCR_DBM; //Buffer switiching enabeld
       //         DMA2_Stream4->CR |=DMA_SxCR_TCIE; //full transfer interrupt enabled
            //    DMA2_Stream4->CR |=DMA_SxCR_HTIE;//half transfer interrupt enabled
                
                DMA2_Stream4->CR |=(1<<11);   //Set Peripheral data size to 16bits
		DMA2_Stream4->CR |=(3<<16); //high prority
		DMA2_Stream4->CR |=(0<<25); //select channel 0
          //     DMA2_Stream4->CR |=DMA_SxCR_CIRC; //circular mode set
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
	
    
							
	 enable_spi();


/****************************************************************************************************/
        /* DMA Config For SPI_TX */
/****************************************************************************************************/
		DMA2_Stream3->CR &= 0;
		while ( (DMA2_Stream3->CR !=0));
	
                //DMA CONFIG for SPI_TX
		DMA2_Stream3->PAR |= (uint32_t)&SPI1->DR;
               DMA2_Stream3->M0AR |= (uint32_t)&adc_resultA[0]; 
                DMA2_Stream3->M1AR |= (uint32_t)&adc_resultB[0];
//               DMA2_Stream3->M0AR |= (uint32_t)&test_bufA[0]; 
//               DMA2_Stream3->M1AR |= (uint32_t)&test_bufB[0];
		DMA2_Stream3->NDTR =351;
		//DMA DOUBLE BUFFER
                DMA2_Stream3->CR |= DMA_SxCR_DBM; //Buffer switiching enabeld
               DMA2_Stream3->CR |=DMA_SxCR_TCIE; //FUll transfer interrupt enabled
		DMA2_Stream3->CR |=(1<<11);   //Set Peripheral data size to 16bits
		DMA2_Stream3->CR |=DMA_SxCR_PL_1;     //Very High prority
                DMA2_Stream3->CR |=DMA_SxCR_PL_0; 
		DMA2_Stream3->CR |=(3<<25); //select channel 3         
		DMA2_Stream3->CR |=DMA_SxCR_MINC;
//		DMA2_Stream3->CR |=DMA_SxCR_CIRC; //circular mode set for SPI
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
		DMA2_Stream2->NDTR =10;
		//DMA DOUBLE BUFFER
          //      DMA2_Stream2->CR |=DMA_SxCR_TCIE; //FUll transfer interrupt enabled
		DMA2_Stream2->CR |=(1<<11);   //Set Peripheral data size to 16bits
		DMA2_Stream2->CR |=DMA_SxCR_PL_1; //High Priority
		DMA2_Stream2->CR |=(3<<25); //select channel 3         
		DMA2_Stream2->CR |=DMA_SxCR_MINC;
                
		DMA2_Stream2->CR |=DMA_SxCR_CIRC; //circular mode set for SPI
	//	DMA2_Stream2->CR |=(1<<6); //direction
	//      DMA2_Stream2->CR |= (1<<5) ; //[perh is flowcontroller
		
         
                //Emable DMA Stream for SPIRX
       //         DMA2_Stream2->CR |=DMA_SxCR_EN;
      //           while (! (DMA2_Stream2->CR & DMA_SxCR_EN) ); //break out when DMA_SxCR_EN==1
/****************************************************************************************************/                

         enable_int_spi();
//	Enable SPI
	SPI1->CR1|=SPI_CR1_SPE;
                spi_cs_enable();
        			


     //Emable DMA Stream for SPI
               DMA2_Stream3->CR |=DMA_SxCR_EN;

while(1)
{
  if  (h==1) //(reset_flag==1)
  {
    h=0;
    flaggg++;
    unsigned int mosi_high=0;
    
       suspend_SPITX_DMA();
  //     suspend_SPIRX_DMA();
        spi_cs_disable();

   
    disable_spi();
    enable_spi();
   //        disable_int_spi();

     SPI1->CR1|=SPI_CR1_SPE;  
  spi_cs_enable();
  
  for(mosi_high=0;mosi_high<2;mosi_high++)
  {
     while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR=0xFFFF;  
 

  }   
 ms_delay(10);


  
  
  for(mosi_high=0;mosi_high<10;mosi_high++)
  {
     while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR=0xFFFF;  
   // while(!(SPI1->SR & SPI_SR_RXNE));
    recv_cmd[mosi_high]=SPI1->DR;
    
   ms_delay(2); //Giving 1ms for slave to prepare next CMD element
  }   
 
 process_cmd(&recv_cmd[1]);
  
            spi_cs_disable();
      
              spi_cs_enable();

  resume_SPITX_DMA();

recv_data[0]=0x0000;

  }
h=0;
}
	






while(1);
}