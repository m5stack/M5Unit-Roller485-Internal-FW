#ifndef __FLASH_H
#define __FLASH_H

#include "stm32g4xx.h"
#include <stdbool.h>
#include "stm32g4xx_hal_flash_ex.h"

//Message head
#define EEPPROM_PACKAGEHEAD 0xAA55//

//Flash page head
#define STM32G0xx_PAGE_SIZE 0x800
#define STM32G0xx_FLASH_PAGE0_STARTADDR 0x8000000
#define STM32G0xx_FLASH_PAGE1_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE2_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+2*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE3_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+3*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE4_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+4*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE5_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+5*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE6_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+6*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE7_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+7*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE8_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+8*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE9_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+9*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE10_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+10*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE11_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+11*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE12_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+12*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE13_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+13*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE14_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+14*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE15_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+15*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE59_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+59*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE60_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+60*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE61_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+61*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE62_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+62*STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE63_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR+63*STM32G0xx_PAGE_SIZE)


#define MIN(A,B) (A<B?A:B)


// void flashReadWriteTest( void ) ;
bool writeMessageToFlash( uint8_t *buff , uint16_t length);
uint16_t readPackedMessageFromFlash( uint8_t *buff , uint16_t length);

#endif
