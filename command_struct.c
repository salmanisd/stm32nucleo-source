#include <command_struct.h>
#include <stm32f4xx.h>

struct Command {
	unsigned short opcode;
	unsigned short len;
	unsigned short descriptor;
};

//struct Command CMD_01;
//struct Command CMD_02;
//
//    CMD_01.opcode=0x0001; //1 to start transmission
//    CMD_01.len=0x0040; //length 64 bits
//    CMD_01.descriptor=0x6789; //fixed to 6789 for now
//    
//    
//    CMD_02.opcode=0x0002; //2 to stop transmission
//    CMD_02.len=0x0040; //length 64 bits
//    CMD_02.descriptor=0x6789; //fixed to 6789 for now

void process_cmd(unsigned short* opcode)
{
switch (*opcode)
{
    case  0x0001:
      TIM3->PSC = 0x2710;
 // start transmission;
    break;
  
    case 0x0002:
      TIM3->PSC = 0x4E20;
    break;
      
    default:
      TIM3->PSC = 30000;  
      break;
}
}