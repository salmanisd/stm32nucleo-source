#include <stm32f4xx.h>


/////
int j=6;
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
	

void main () {

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


	//peripheral clock enable register ,enable SPI1 clock
	RCC->APB2ENR |=  RCC_APB2ENR_SPI1EN ; 

	
	SPI1->CR1 |=(0x0002<<3); // Baud Rate as  fpclk/8 (10.5Mhz) where fpclk is APB2 clock=84Mhz
	SPI1->CR1 |= SPI_CR1_SSM ;
	SPI1->CR1 |= SPI_CR1_SSI;                       
	SPI1->CR1 |=SPI_CR1_MSTR;						
	
	SPI1->CR2|=SPI_CR2_TXDMAEN; //DMA request when TX empty flag set
	
	char dma_str[]="hello my name is salman ahmed what is your name ...Testing GIT2";
	
	
	//DMA2 STream3
		
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
		
		DMA2_Stream3->CR &= 0;
		while ( (DMA2_Stream3->CR !=0));
	
		DMA2->LISR &= 0;
		DMA2->HISR &= 0;
		
		DMA2_Stream3->PAR |= (uint32_t)&SPI1->DR;
    DMA2_Stream3->M0AR |= (uint32_t)&dma_str[0];
		DMA2_Stream3->NDTR |=0x50;
		
		DMA2_Stream3->CR |=(3<<16); //high prority
		DMA2_Stream3->CR |=(3<<25); //select channel 3
		DMA2_Stream3->CR |=DMA_SxCR_MINC;
		DMA2_Stream3->CR |=(1<<6); //direction
//DMA2_Stream3->CR |= (1<<5) ; //[perh is flowcontroller
		
		DMA2_Stream3->CR |=DMA_SxCR_EN;
	


//	Enable SPI
	SPI1->CR1|=SPI_CR1_SPE;



	GPIOB->BSRRH|=0x0040 ; //CS Enable (low)
	
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
